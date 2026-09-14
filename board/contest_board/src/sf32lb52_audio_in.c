/****************************************************************************
 * board/contest_board/src/sf32lb52_audio_in.c
 *
 * SF32LB52 录音通路封装（麦克风输入）
 *
 * 只做三件事：start（open + CONFIGURE + START）、read、stop（STOP + close）。
 * 内部用一个全局 fd 表示"当前这次录音会话"，用一把 nxmutex 保护 fd 与状态。
 *
 * 线程约定（重要）：
 *   - audio_in_read() 是阻塞的，**读的时候不持锁**，只短暂取锁拿 fd 快照。
 *     否则"一个任务在读、另一个任务调 audio_in_stop() 发 STOP 救场"的
 *     经典用法会直接死锁（stop 拿不到锁 -> 发不出 STOP -> read 永远不返回）。
 *   - 只有 fd 的创建/销毁（start/stop）和状态读写在锁里。
 *
 * 录音参数固定 16 kHz / 单声道 / 16bit：这是驱动与本板麦克风一路验证过的组合。
 * 详细说明与完整示例见 docs/audio_driver_usage.md 第 9 节。
 *
 * 残留会话自愈（重要）：
 *   单一大镜像意味着 g_audio_in_fd 是所有 app 共享的一份全局。某个 app 录音
 *   中途挂掉/退出时没人替它清这份状态，后面的 app 就会一直拿到 -16（EBUSY）。
 *   所以除了 fd 还记下"开它的线程 id"，audio_in_start() 在 EBUSY 时会先确认
 *   持有者线程是否还存在，消失了的就当残留丢掉再重开一次。
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/audio/audio.h>
#include <nuttx/mutex.h>
#include <nuttx/sched.h>

#include "sf32lb52_audio_in.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 驱动 codec 时钟表里有的档位（sf32lb52_audio.c:167-186 的
 * codec_dac_clk_config / codec_adc_clk_config）。
 * 其余采样率驱动会在 CONFIGURE 时返回 -EINVAL，这里提前拦掉。 */

#define AUDIO_IN_RATE_8K      8000
#define AUDIO_IN_RATE_16K     16000
#define AUDIO_IN_RATE_44K1    44100
#define AUDIO_IN_RATE_48K     48000

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* 当前录音会话的 fd，-1 = 未打开 */

static int  g_audio_in_fd = -1;

/* 开这次会话的线程 id（nxsched_gettid()）。
 *
 * 为什么需要它：整机是单一大镜像，这份全局被**所有 app 共享**，某个 app
 * 录音中途挂掉/退出时既不会调 audio_in_stop()，也不会有人替它清 g_audio_in_fd，
 * 于是后面每个 app 都拿到 -16（EBUSY）、而 ps 里已经找不到那个 app 了。
 * 记下持有者之后，start() 就能分辨"别人真在录"和"死人留下的残局"。 */

static pid_t g_audio_in_owner = (pid_t)-1;

/* 开这次会话的**任务组** id（getpid()：NuttX 里 pthread 拿到的是本组组长的 pid）。
 *
 * 为什么还要记组号：fd 号是**每个 task group 各一套**的。整机是单一大镜像，这份
 * 全局被所有 app 共享，于是"另一个 app 的线程"完全可能读到这个 fd 号并拿去
 * ioctl/close —— 那个数字在它自己的组里指的是**另一个 file 对象**。实测真机事故
 * 就是这条路径：robot_ui 的播报线程替 hello_app 调 audio_in_stop()，结果
 * ioctl(AUDIOIOC_STOP) 问到了 robot_ui 组里另一个东西上，inode 上没有 ioctl
 * （VFS 返回 -ENOTTY=25），STOP 没生效、hello_app 的录音线程还阻塞在 read() 里；
 * 紧接着的 close(fd) 关掉的也是 robot_ui 自己的某个无关 fd。
 *
 * 注意判据不能停在"组号不同就拒绝"：task_create() 出来的子任务（hw_test 的读任务
 * 就是）会复制父任务的 fd 表，它的同号 fd 指向同一个 file 对象，对音频设备完全
 * 有效。所以组号只用来走快路 + 打日志，真正决定"能不能动这个 fd"的是下面那个
 * 身份探测（audio_in_fd_is_audio_device）。 */

static pid_t g_audio_in_owner_group = (pid_t)-1;

/* fd 与状态的保护锁。只保护 start/stop 的临界区，不覆盖阻塞的 read。 */

static mutex_t g_audio_in_lock = NXMUTEX_INITIALIZER;

/* "有线程正在停这次会话"。停在锁外做（ioctl 不能持锁），所以用一个标志防并发：
 * 两个线程同时看到同一个 fd 各自 close 一次，第二次关掉的可能是刚被别人复用
 * 的槽位。只由 audio_in_stop() 读写，配着全局一起看；标志带线程 id，好让
 * "停到一半线程就没了"留下的残留能被下一次调用识别并清掉（和 start() 里那套
 * "持有者还在不在"的自愈判据同一个套路）。 */

static bool  g_audio_in_stop_pending;
static pid_t g_audio_in_stop_owner = (pid_t)-1;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**
 * @brief  采样率是否是驱动支持的档位
 */

static bool audio_in_rate_supported(int sample_rate)
{
  return sample_rate == AUDIO_IN_RATE_8K  ||
         sample_rate == AUDIO_IN_RATE_16K ||
         sample_rate == AUDIO_IN_RATE_44K1 ||
         sample_rate == AUDIO_IN_RATE_48K;
}

/**
 * @brief  fd 解析到的到底是不是本板音频设备
 *
 * 用 GETCAPS(QUERY) 探测：这条命令**只读不写**（驱动侧只填 caps 结构，不改任何
 * 状态、不动寄存器），问别的东西也不会造成副作用。音频设备一定会答"支持输入
 * （麦克风）"；普通文件、别的设备要么直接失败（-ENOTTY / -EBADF），要么答不上来。
 *
 * 为什么要这层探测而不是盲目发 STOP：拿一个不属于本组的 fd 号去发音频命令，
 * 命令号会落到**那个设备自己的 ioctl 处理函数**上（这正是本次事故的形态）。
 */

static bool audio_in_fd_is_audio_device(int fd)
{
  struct audio_caps_s caps;

  memset(&caps, 0, sizeof(caps));
  caps.ac_len     = sizeof(struct audio_caps_s);
  caps.ac_type    = AUDIO_TYPE_QUERY;
  caps.ac_subtype = AUDIO_TYPE_QUERY;

  if (ioctl(fd, AUDIOIOC_GETCAPS, (unsigned long)&caps) < 0)
    {
      return false;
    }

  return caps.ac_len >= sizeof(struct audio_caps_s) &&
         (caps.ac_controls.b[0] & AUDIO_TYPE_INPUT) != 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: audio_in_start
 ****************************************************************************/

int audio_in_start(int sample_rate, int channels, int bits)
{
  struct audio_caps_desc_s capdesc;
  FAR struct tcb_s *owner_tcb;
  pid_t self;
  int fd;

  /* 参数校验：不做隐式纠正，非法就直接拒绝，免得录出格式不对的数据 */

  if (!audio_in_rate_supported(sample_rate) ||
      channels != 1 || bits != AUDIO_IN_DEFAULT_BITS)
    {
      syslog(LOG_ERR,
             "AUDIO_IN: bad params (rate=%d ch=%d bits=%d), "
             "only 8k/16k/44.1k/48k, mono, 16bit\n",
             sample_rate, channels, bits);
      return -EINVAL;
    }

  if (nxmutex_lock(&g_audio_in_lock) < 0)
    {
      return -EINVAL;                 /* 锁坏掉了，属于不可能的状态 */
    }

  /* 已经有会话：先分辨是哪一种"忙"，不能一律 -EBUSY。
   *
   * 分三种：
   *   1) 持有者就是当前线程 —— 同一线程里真的在录，如实返回 -EBUSY；
   *   2) 持有者线程还在（nxsched_get_tcb() 拿得到 TCB）—— 别人真在录，
   *      返回 -EBUSY，不打断；
   *   3) 持有者线程已经不存在 —— 死掉的 app 留下的残留（单一大镜像里
   *      这份全局没人替它清），清掉残留后照常重开一次。
   */

  self = nxsched_gettid();

  if (g_audio_in_fd >= 0)
    {
      if (g_audio_in_owner == self)
        {
          nxmutex_unlock(&g_audio_in_lock);
          syslog(LOG_WARNING,
                 "AUDIO_IN: 本线程 %d 已在录音，拒绝重入（-EBUSY）\n",
                 (int)self);
          return -EBUSY;
        }

      owner_tcb = nxsched_get_tcb(g_audio_in_owner);

      if (owner_tcb != NULL)
        {
          /* nxsched_get_tcb() 会加 TCB 引用计数，必须配对释放 */

          nxsched_put_tcb(owner_tcb);
          nxmutex_unlock(&g_audio_in_lock);
          syslog(LOG_WARNING,
                 "AUDIO_IN: 录音设备仍被线程 %d 占用（-EBUSY）\n",
                 (int)g_audio_in_owner);
          return -EBUSY;
        }

      syslog(LOG_WARNING,
             "AUDIO_IN: 持有线程 %d 已消失，判定为残留录音会话（fd=%d），"
             "丢弃后重开一次\n",
             (int)g_audio_in_owner, g_audio_in_fd);

      /* **只清全局状态，不碰那个 fd 号**：fd 号是线程自己 fd 表里的槽位，
       * 线程死了它的 fd 早被内层回收，这个数字随时可能被当前任务里另一个
       * 打开的文件占着 —— 对它做 close()/ioctl() 会误伤别人，所以这里
       * 一个系统调用都不发，把收尾交给下面的完整 open + START：
       * 驱动侧 sf32lb52_audio_start() 会先做一次干净 stop 再真启动。 */

      g_audio_in_fd          = -1;
      g_audio_in_owner       = (pid_t)-1;
      g_audio_in_owner_group = (pid_t)-1;
    }

  /* 走到这里只有两种可能：本来没有会话，或刚把残留清掉。
   * 两种情况都完整走一遍 open + CONFIGURE + START。 */

  fd = open(AUDIO_IN_DEV, O_RDONLY);
  if (fd < 0)
    {
      int errcode = errno;

      nxmutex_unlock(&g_audio_in_lock);
      syslog(LOG_ERR, "AUDIO_IN: open %s failed: %d\n",
             AUDIO_IN_DEV, errcode);
      return -errcode;
    }

  memset(&capdesc, 0, sizeof(capdesc));
  capdesc.caps.ac_len            = sizeof(struct audio_caps_s);
  capdesc.caps.ac_type           = AUDIO_TYPE_INPUT;
  capdesc.caps.ac_channels       = (uint8_t)channels;
  capdesc.caps.ac_controls.hw[0] = (uint16_t)sample_rate;
  capdesc.caps.ac_controls.b[2]  = (uint8_t)bits;

  if (ioctl(fd, AUDIOIOC_CONFIGURE, (unsigned long)&capdesc) < 0)
    {
      int errcode = errno;

      close(fd);
      nxmutex_unlock(&g_audio_in_lock);
      syslog(LOG_ERR, "AUDIO_IN: AUDIOIOC_CONFIGURE failed: %d\n", errcode);
      return -errcode;
    }

  if (ioctl(fd, AUDIOIOC_START, 0) < 0)
    {
      int errcode = errno;

      close(fd);
      nxmutex_unlock(&g_audio_in_lock);
      syslog(LOG_ERR, "AUDIO_IN: AUDIOIOC_START failed: %d\n", errcode);
      return -errcode;
    }

  /* 最后一步才发布 fd：中途失败时全局状态始终是"未打开"，
   * 别人不会拿到一个半配置好的会话。持有者线程 + 持有者任务组 + fd 一起发布，
   * 保证"fd >= 0"时这两个 id 一定是有效值（公开函数全靠它们判断能不能动 fd）。 */

  g_audio_in_fd          = fd;
  g_audio_in_owner       = self;
  g_audio_in_owner_group = getpid();

  nxmutex_unlock(&g_audio_in_lock);

  syslog(LOG_INFO, "AUDIO_IN: started (%d Hz, %d ch, %d bit)\n",
         sample_rate, channels, bits);
  return OK;
}

/****************************************************************************
 * Name: audio_in_read
 ****************************************************************************/

ssize_t audio_in_read(FAR void *buf, size_t len)
{
  int fd;
  ssize_t n;

  if (buf == NULL || len == 0)
    {
      return -EINVAL;
    }

  /* 只拿一个 fd 快照，**不持锁**做阻塞读：否则别的任务没法调
   * audio_in_stop() 来发 STOP 唤醒这里的 read()。 */

  if (nxmutex_lock(&g_audio_in_lock) < 0)
    {
      return -EINVAL;
    }

  fd = g_audio_in_fd;

  nxmutex_unlock(&g_audio_in_lock);

  if (fd < 0)
    {
      return -EINVAL;                 /* 没 start，或者已经被 stop 了 */
    }

  /* 这里**不**做跨组校验：fd 号只在开它的那个组里有意义，而 task_create() 出来的
   * 子任务（比如 hw_test 的读任务）会复制父任务的 fd 表（group_setuptaskfiles ->
   * fdlist_copy），它的同一个 fd 号指向**同一个 file 对象**，读本板音频设备完全
   * 有效。能读到什么由那个 file 对象决定，不是由组号决定，所以这里不做判断。 */

  n = read(fd, buf, len);

  /* 这里是**原样转发**，一个字都不吞：上层的判据全靠这个返回值。
   *
   * read() 的语义（本板音频设备 dev/audio/audio0）：
   *   正数 = 真的读到了这么多字节；
   *   0    = EOF：被 AUDIOIOC_STOP 打断 / 设备没在跑 / 会话换代 ——
   *          也就是驱动 sf32lb52_audio_read 里"这一代会话真的结束了"那三个判据；
   *   负值 = 错误或超时。其中"下层分片等待超时"是驱动特意区分出来的
   *          （驱动返回 -ETIMEDOUT），别的负值是真错误（没 start、fd 失效等）。
   *
   * 注意这里调的是 NuttX 的 POSIX read()：驱动返回的负值会被 libc 那层翻成
   * -1 并把真正的错误号放进 errno（fs/vfs/fs_read.c 的 readv()：先
   * set_errno(-ret)、再 return -1）。所以调用方要分"超时"和"别的错误"，
   * 看的是 errno == ETIMEDOUT，不是返回值本身；而且调用前得先把 errno 清 0，
   * 否则会读到上一次调用留下的旧值。本函数自己返回 -EINVAL 的那几条路
   * （没 start / buf 为空 / len 为 0）不碰 errno。 */

  return n;
}

/****************************************************************************
 * Name: audio_in_stop
 ****************************************************************************/

int audio_in_stop(void)
{
  pid_t owner;
  pid_t owner_group;
  int fd;
  int errcode;

  if (nxmutex_lock(&g_audio_in_lock) < 0)
    {
      return -EINVAL;
    }

  fd = g_audio_in_fd;

  if (fd < 0)
    {
      nxmutex_unlock(&g_audio_in_lock);
      return OK;                      /* 没在录音，安全空操作 */
    }

  if (g_audio_in_stop_pending)
    {
      /* 已经有一个线程正在停这次会话（全局还没清就说明它还没停完）。
       * 放行会让两个线程拿到同一个 fd 各 close 一次 —— 第二次 close 关掉的
       * 可能是这个槽位刚被别人复用的 fd，所以这里直接空转返回。
       *
       * 但要先确认那个线程还在：停的动作在锁外做（ioctl 不能持锁），万一它在
       * 中途被删/被杀，标志就永远是残留、别人再也停不掉这个会话了。判据沿用
       * start() 里那套"持有者线程还在不在"的做法，自愈。 */

      FAR struct tcb_s *stopper = nxsched_get_tcb(g_audio_in_stop_owner);

      if (stopper != NULL)
        {
          nxsched_put_tcb(stopper);
          nxmutex_unlock(&g_audio_in_lock);
          return OK;
        }

      syslog(LOG_WARNING,
             "AUDIO_IN: 上一次停会话的线程 %d 已消失，清掉残留的 stop 标志\n",
             (int)g_audio_in_stop_owner);
      g_audio_in_stop_pending = false;
    }

  owner       = g_audio_in_owner;
  owner_group = g_audio_in_owner_group;
  g_audio_in_stop_pending = true;
  g_audio_in_stop_owner   = nxsched_gettid();

  nxmutex_unlock(&g_audio_in_lock);

  /* 跨组调用要先确认这个 fd 到底通到哪个设备，再决定动不动它。
   *
   * 为什么不能只看组号就拒绝：task_create() 出来的子任务（hw_test 的读任务就是）
   * 会复制父任务的 fd 表，它的同号 fd 指向**同一个 file 对象**，对本板音频设备
   * 完全有效 —— 一刀切拒绝会把这种合法用法也挡掉。
   * 为什么不能不看就发 STOP：fd 号只在开它的那个组里保证有意义。别组的同号 fd
   * 是另一个 file 对象，音频命令号会落到**那个设备自己的 ioctl 处理函数**上 ——
   * 真机事故（AUDIOIOC_STOP failed: 25）正是这么来的：命令问到了 robot_ui 组里
   * 另一个东西上，音频驱动压根没收到。
   * 所以这里先做一次只读探测（GETCAPS(QUERY)，不改任何状态），探测不通过就
   * 一个系统调用都不发、全局也不动。 */

  if (owner_group != getpid() && !audio_in_fd_is_audio_device(fd))
    {
      syslog(LOG_ERR,
             "AUDIO_IN: stop 被别的 task group 调用，且 fd=%d 在本组里不是音频设备"
             "（会话属于 group %d / 线程 %d，调用者是 group %d 线程 %d）：已拒绝，"
             "STOP 没发出去、fd 没关、全局状态没动。\n"
             "  说明：fd 号在每个 task group 里各有一套含义，跨组拿别人的 fd 号"
             "做 ioctl/close 只会打到本组自己的文件上（这就是 AUDIOIOC_STOP "
             "failed: 25 / ENOTTY 的来源）。跨 app 让路请改成"
             "\"登记请求 + 持有者自己的线程执行\"。\n",
             fd, (int)owner_group, (int)owner, (int)getpid(),
             (int)nxsched_gettid());

      if (nxmutex_lock(&g_audio_in_lock) == 0)
        {
          g_audio_in_stop_pending = false;
          g_audio_in_stop_owner   = (pid_t)-1;
          nxmutex_unlock(&g_audio_in_lock);
        }

      return -EPERM;
    }

  /* 顺序很关键：先 STOP（把阻塞在 read() 里的任务唤醒），再 close。
   * close() 只在"最后一个 fd"时才走到驱动的 shutdown()；本模块只在文件里
   * 用到 STOP + close（见 docs/audio_driver_usage.md 第 8 节）。 */

  if (ioctl(fd, AUDIOIOC_STOP, 0) < 0)
    {
      errcode = errno;

      /* 关键：这一步失败时**什么都不动** —— 不清全局、不 close。
       *
       * 为什么不能在失败后照样 close + 清全局（原来就是这么写的）：
       * fd 号只在"开它的那个 task group"里有意义，别组的同号 fd 是另一个
       * file 对象，把它 close 掉就是误伤别人的 fd；而全局一旦被清成"已停"，
       * 真正的持有者就丢了唯一的句柄 —— 它的设备可能还在跑，却再也停不掉了
       * （真机现象：AUDIOIOC_STOP failed: 25 之后紧接着打印 "stopped"，
       *  而 hello_app 的录音线程仍阻塞在 read() 里，300ms 收不了尾）。
       *
       * 下面这条按 errno 分诊，直接告诉看日志的人"这个 fd 是不是音频设备"：
       *   ENOTTY(25) / EBADF(9) → 这个 fd 号在本组里根本不是音频设备（别人的
       *   fd 号），STOP 压根没送到驱动；其他 errno → 是本组自己的设备，但
       *   驱动当时拒绝了这次 STOP。 */

      syslog(LOG_ERR,
             "AUDIO_IN: AUDIOIOC_STOP failed: %d（fd=%d，会话属于 group %d / "
             "线程 %d，调用者 group %d 线程 %d）%s"
             "全局状态与 fd 都没动：设备可能仍在跑，持有者可以重试\n",
             errcode, fd, (int)owner_group, (int)owner,
             (int)getpid(), (int)nxsched_gettid(),
             (errcode == ENOTTY || errcode == EBADF) ?
               "—— 这个 fd 号在本组里不是音频设备（很可能是别的 app 的 fd 号），"
               "STOP 没有送到音频驱动，hw_stop() 没执行，阻塞在 read() 里的线程"
               "也等不到唤醒。" :
               "—— 音频设备在，但这次 STOP 被拒了。");

      if (nxmutex_lock(&g_audio_in_lock) == 0)
        {
          g_audio_in_stop_pending = false;
          g_audio_in_stop_owner   = (pid_t)-1;
          nxmutex_unlock(&g_audio_in_lock);
        }

      return -errcode;
    }

  /* STOP 成功才敢认定"这个 fd 就是音频设备"（也可能是 task_create 复制出来的
   * 同一个 file 对象，两者 close 掉都是安全的：file 是引用计数的）。
   * 这时候才清全局、才 close —— 而且只在"会话还是刚才那一个"时清，
   * 免得把期间新开的一次会话顶掉。 */

  if (nxmutex_lock(&g_audio_in_lock) == 0)
    {
      if (g_audio_in_fd == fd)
        {
          g_audio_in_fd          = -1;
          g_audio_in_owner       = (pid_t)-1;
          g_audio_in_owner_group = (pid_t)-1;
        }

      g_audio_in_stop_pending = false;
      g_audio_in_stop_owner   = (pid_t)-1;
      nxmutex_unlock(&g_audio_in_lock);
    }

  close(fd);

  if (owner_group != getpid())
    {
      /* 跨组的合法调用（task_create 复制的 fd 表）：事后必须能一眼看到，
       * 因为"谁在停别人的会话"正是本次要根治的那件事。 */

      syslog(LOG_WARNING,
             "AUDIO_IN: stopped（由别的 task group %d 代停：会话 group=%d / "
             "线程 %d）\n",
             (int)getpid(), (int)owner_group, (int)owner);
      return OK;
    }

  syslog(LOG_INFO, "AUDIO_IN: stopped\n");
  return OK;
}
