/****************************************************************************
 * boards/sf32lb52/sf32lb52_devkit_lcd/src/sf32lb52_audio.c
 *
 * SF32LB52 音频设备驱动（audio_lowerhalf 实现）
 *
 * 架构（参考 SiFli SDK audprc 例程，RT-Thread 侧已验证）：
 *   - AUDCODEC：模拟前端（DAC/ADC 模拟通路，不走 DMA）
 *   - AUDPRC  ：数字音频处理器，数据通路（TX0=播放 DMA / RX0=录音 DMA）
 *
 * 数据流：
 *   播放：内存 → AUDPRC TX0 DMA → AUDPRC → codec DAC 模拟 → AW8155 功放 → 喇叭
 *   录音：麦克风 → codec ADC 模拟 → AUDPRC → AUDPRC RX0 DMA → 内存
 *
 * 注册为 NuttX 音频设备 /dev/audio0。
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <unistd.h>              /* getpid()：记会话持有者的任务组 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <syslog.h>
#include <math.h>

#include <nuttx/kmalloc.h>
#include <nuttx/audio/audio.h>
#include <nuttx/sched.h>        /* nxsched_gettid()/nxsched_get_tcb()：查持有者线程还在不在 */
#include <nuttx/semaphore.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "bf0_hal_audcodec.h"
#include "bf0_hal_audprc.h"
#include "bf0_hal_pmu.h"
#include "bf0_hal_rcc.h"
#include "bf0_hal_gpio.h"
#include "register.h"
#include "dma_config.h"
#include "sf32lb52_audio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SF32LB52_AUDIO_CLK_TAB_NUM  9

/* 通道选择：单通道（DAC0 播放 / ADC0 录音；AUDPRC TX0 / RX0）
 * 注意：codec 的 Config_TChanel/RChanel 用通道索引（0/1），
 * 不是 HAL_AUDCODEC_*_CHx 枚举（DAC_CH0=0, ADC_CH0=2） */

#define SF32LB52_AUDIO_DAC_CH       0
#define SF32LB52_AUDIO_ADC_CH       0
#define SF32LB52_AUDIO_PRC_TX_CH    HAL_AUDPRC_TX_CH0
#define SF32LB52_AUDIO_PRC_RX_CH    HAL_AUDPRC_RX_CH0

/* 功放使能 GPIO：PA10 = AU_PA_EN（bsp_pinmux 已配置为 GPIO 功能） */

#define SF32LB52_AUDIO_PA_GPIO      ((GPIO_TypeDef *)hwp_gpio1)
#define SF32LB52_AUDIO_PA_PIN       10

/* AUDPRC 通路选择（MUX/MIX 源），参考 SDK audprc 例程 out_sel=0x5050 */

#define SF32LB52_AUDIO_OUT_SEL      0x5050

/* 默认音量（dB） */

#define SF32LB52_AUDIO_DEFAULT_VOL  (-6)

/* 录音通路数字增益（dB，有效范围 -36 ~ +60）。
 * 原来设 0dB，实测正常说话只有 peak≈2400/32767 ≈ -23dBFS，回放听着很小、
 * 对 VAD/ASR 也不利。+18dB 会削顶（peak 打满 32768），所以取 +12dB（约 4 倍）。 */
#define SF32LB52_AUDIO_ADC_VOL      (12)

/* 标准音量接口把 0~1000 映射到这个 dB 区间（HAL 只接受 -36~+60） */
#define SF32LB52_AUDIO_VOL_MIN      (-36)
#define SF32LB52_AUDIO_VOL_MAX      (6)

/* 开/关功放时先把 DAC 拉到最小音量并等通路静态电平稳定，
 * 避免功放使能/断开那一瞬间把 DAC 输出的跳变放大成"啪"的爆音。
 * HAL 的下限就是 -36（没有真正的静音位），这是它能给的最大衰减。 */
#define SF32LB52_AUDIO_VOL_MUTE      SF32LB52_AUDIO_VOL_MIN
#define SF32LB52_AUDIO_SETTLE_MS     30

/* RX 自愈日志的限频窗口。触发一次完整恢复的前提是"整整 5 秒零完成中断"，
 * 真出复位风暴时就是每 5 秒一条 —— 不限频会把串口刷满、把现场其它线索冲掉，
 * 所以窗口内只累计次数，出窗口时把被吞掉的次数一起报出来。 */
#define SF32LB52_AUDIO_RECOVER_LOG_MS   30000

/* "复位风暴"的判据：两次自愈的间隔 <= 这个值。
 * 一次自愈本身就要等满 5 秒，所以间隔短于它等于"复位之后连几个 5 秒窗口都没
 * 撑过" —— 那是复位只压住了症状、根因还在；干净的一次自愈之后，间隔必然是
 * "正常录音直到下次真的出问题"，远大于它。 */
#define SF32LB52_AUDIO_RECOVER_STORM_MS 15000

/* 会话切换的宽限期：START 成功不到这么久的会话**绝不**被判为残留强停。
 *
 * 误伤窗口永远在会话开头：本板存在"起会话的线程是短命线程"的真实用法（见
 * struct 里 holder_tid 的说明），那种线程 START 完就走，而真正读数据的线程
 * 往往还没进第一次 read()（user_tid 还是 -1）。在这条会话刚起来的那一小段
 * 时间里，两条线程证据都是空的，会话却是活的 —— 光看线程就会把它当残局拆了。
 * 宽限期把这一段单独掐掉：宁可按"有人在用"回 -EBUSY（交给上层重试），也不去
 * 拆一条刚开始的会话。 */
#define SF32LB52_AUDIO_SWITCH_GRACE_MS  3000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sf32lb52_audio_s
{
  struct audio_lowerhalf_s dev;         /* NuttX 音频低半部分设备 */
  AUDCODEC_HandleTypeDef  codec;        /* 音频 codec（模拟前端） */
  AUDPRC_HandleTypeDef    aprc;         /* 音频处理器（数据通路 DMA） */
  bool                    running;      /* 设备是否在运行 */
  bool                    playback;     /* 已提交方向：true=播放 false=录音（只在 running 为真时有意义） */
  bool                    pending_playback; /* 最近一次 CONFIGURE 记下的方向意图，START 成功才提交 */
  int                     playback_db; /* 当前播放音量(dB)，标准接口可改 */

  /* 会话持有者身份 —— 用来区分"反方向那条通路是真的被别人占着"和"上一个
   * 持有者早就死了、只剩一堆没人清的状态"。
   *
   * holder_tid：hw_start 里记下的、把这次会话起起来的线程。AUDIOIOC_START 走的是
   *             调用者的上下文（nuttx/audio/audio.c 的 audio_start 直接调
   *             lower->ops->start），所以它就是调 start 的那个线程。
   * user_tid  ：最近一次 read()/write() 的线程。为什么要多这一条：起会话的线程
   *             可能是短命的工作线程 —— 本板真的有这种用法（hello_app 的
   *             audio_record_start() 在调用者线程里 audio_in_start()，真正读数据
   *             的是它随后起的录音线程）。只认 holder_tid 会把"会话明明还有人在
   *             读"误判成残留，那就变成替一个活着的会话拆台了。
   * holder_pid：起会话的任务组（getpid()，NuttX 里组长的 pid 就是任务组）。
   *             判"残留"时必须**同时**满足它也查不到 TCB —— 那才等价于"整个
   *             app 没了"。线程死了但组还在时按"有人"处理：本板真有"起会话的
   *             线程是短命线程"的用法（robot_ui 的 DETACHED 开麦线程、
   *             ai_audio 在播放线程里恢复录音后播放线程退出），只认线程会在
   *             "新会话刚 START 成功、读数据的线程还没进第一次 read"的窗口里
   *             把活会话判成残局强停（详见 sf32lb52_audio_holder_alive 的注释）。
   *             **这是有意的取舍**：代价是"app 还活着但那条播放线程真死了"这种
   *             残局会退化成 -EBUSY 等上层重试/由那个 app 自己 close 收尾，
   *             换来的是绝不误停活会话。
   * holder_since：这次会话在 hw_start 里提交成功的时刻，判"会话刚开始"用
   *             （SF32LB52_AUDIO_SWITCH_GRACE_MS，见 sf32lb52_audio_start）。
   *
   * 四个都只在 running 为真时有意义：前三个在 hw_start 里记、hw_stop/hw_shutdown
   * 清 running 时一起清。-1 = 没记过。 */

  pid_t                   holder_tid;
  pid_t                   holder_pid;
  pid_t                   user_tid;
  clock_t                 holder_since;

  /* 真实通路状态位 —— 这是"硬件到底开着哪几条模拟级"的唯一真相，
   * **不是**方向标签。
   *
   * 为什么不能拿 playback 当真相：整机是单一大镜像，priv 全局只有一份，
   * 两个 app（hello_app 常开麦录音 / robot_ui 偶尔播报）都在用它。某一个 app
   * 只做一次 CONFIGURE(OUTPUT)（例如 ai_audio.c 的 audio_hw_set_volume()，
   * 它 open + CONFIGURE + close、**不 START**）就会把正在录音的那个会话的
   * 方向标签翻成"播放"。方向标签一旦是谎话，收尾时就只有两种坏法：
   *   去关一条从没打开过的模拟通路（实测重复关模拟级会整机静默卡死），
   *   或者该关的 DAC/功放永远漏关（录音把标签翻回 false，播放侧收尾就跳过）。
   * 所以下面这三个位在真正开/关那一行如实置位/清位，收尾只看它们。 */

  bool                    dac_path_on;  /* codec DAC 模拟通路确实开着 */
  bool                    adc_path_on;  /* codec ADC 模拟通路确实开着 */
  bool                    pa_on;        /* 功放（AW8155）确实使能着 */
  int                     samplerate;   /* 当前采样率 */
  int                     nchannels;    /* 当前通道数 */
  int                     bpsamp;       /* 当前采样位深 */
  FAR struct ap_buffer_s *tx_apb;       /* 播放中的 buffer（队列模式） */
  FAR struct ap_buffer_s *rx_apb;       /* 录音中的 buffer */
  sem_t                   wr_sem;       /* write() 同步信号量 */
  bool                    wr_busy;      /* write() DMA 进行中 */
  sem_t                   rx_sem;       /* read() 同步信号量 */
  bool                    rx_busy;      /* read() DMA 进行中 */
  bool                    rx_aborted;   /* stop() 打断了正在等待的 read() */

  /* DMA 传输错误（TE）打断了正在等待的 read()。
   *
   * 为什么要单独立这一面旗：TE 发生时 HAL 在中断里就把通道
   * `DMA_FreeChannel()` + `State = READY` 做完了（bf0_hal_dma.c:1228-1235），
   * 于是"整段 0 完成中断 + 停机前句柄仍 BUSY"这条自愈判据两条都不成立
   * （句柄已经是 READY），这一类**永远不会自愈**，只能每次空转满 5 秒。
   * 错误回调在中断上下文里只做三件轻活：计数、立旗、post 信号量；
   * **不在中断里打日志** —— 串口那一侧会抢控制台锁把整机挂住，本文件在
   * 播放完成（TX 中断）回调里就为此删过一次 printf，那次实测是"整机卡死"。
   * 日志和恢复都在 read() 的收尾分支里补。 */

  bool                    rx_dma_err;

  /* 会话代号：每一次让"录音这次会话"作废的事件都会 +1 —— hw_start（新会话开始）、
   * hw_stop（上层 STOP）、hw_shutdown（最后一个 fd 被 close）。
   *
   * read() 进分片等待前先记下它，每一片醒来都比对一次。为什么要这个额外判据：
   * 那个循环的出口原来只有"被 post 唤醒 / rx_aborted / running 变假"三种，全都
   * 依赖**别的上下文确实把那两个标志写好或真的唤醒过它**。真机上出现过唤醒没生效
   * （见 ai_audio.c 里"连 nxsem_tickwait 的 5 秒超时都没回来"那条记录），也出现过
   * 新的会话/新的 STOP 把旧 read 的等待拖住的情形。有了会话代号，只要这次 read 的
   * 时代已经过去，它下一片醒来就自己收摊 —— 不需要任何人记得去唤醒它。 */

  uint32_t                session_gen;

  /* -110（read 等 DMA 超时）的现场证据。
   *
   * 光看"read 超时"一句话分不清是哪一种坏法，这 5 个计数器就是分诊用的：
   *   irq 一直不动 + read 次数在涨  → DMA 起了但完成中断压根没来
   *                                  （最可疑：ADC 模拟通路/AUDPRC 没真使能，
   *                                    也就是 hw_start 被 start 的早退跳过了）
   *   busy_fail 在涨                → Receive_DMA 起的不是新传输（HAL 侧 BUSY）
   *   not_armed 在涨                → Receive_DMA 回了 HAL_OK，可 DMA 句柄压根没
   *                                  进 BUSY：Start_IT 没干活（它自己的 HAL_BUSY
   *                                  被厂商丢掉了，见 read 里的第二次校验）
   *   irq 在涨而 timeout 也在涨     → 中断其实来了，是等待/唤醒这一侧的问题
   * 都是单调递增的 32bit 计数，不做回绕保护（真跑到回绕早该查完这件事了）。
   *
   * 补充（本次改动）：超时日志里 aprc.State[RX] / hdma[RX].State 这两个字段原来
   * 是在本函数收尾（DMAStop + 强制置 READY）之后才采样的，打出来的永远是刚写
   * 进去的 0x1/0x1，对定位没有价值。现在改成**停机之前**的快照，才真的能看出
   * "等待期间通道是不是一直武装着"。
   * 另外新增 recov 计数：read 判定"RX 真死了"（整段 5 秒 0 完成中断 + 停机前
   * 句柄仍 BUSY）之后做的完整恢复次数。它一涨就说明冻死那次又出现了、并且被
   * 自愈流程接住了。
   *
   * 本次再补两个心跳与一个错误计数器（对应三种故障的分诊）：
   *   half 在涨                     → 数据真的在流（RxHalfCplt 每 10ms 一次，
   *                                   20ms 一帧的半满）；irq 冻结而 half 还在涨
   *                                   说明"数据是通的、只有 TC 那一路丢了"，
   *                                   和"数据源头压根没动"是两种病；
   *   dma_err 在涨 + irq_delta==0   → DMA 传输错误(TE)：HAL 已经 FreeChannel +
   *                                   State=READY，所以 hdma[RX].State=0x1 而
   *                                   irq 不动，**这一组合就是 TE 的指纹**
   *                                   （自愈走 read 里新增的 TE 分支，不再要求
   *                                   句柄停在 BUSY）。
   */

  uint32_t                rx_irq_count;       /* RxCplt 中断次数（无条件自增） */
  uint32_t                rx_half_irq_count;  /* RxHalfCplt 中断次数（第二个心跳） */
  uint32_t                rx_read_count;      /* read() 进入次数 */
  uint32_t                rx_timeout_count;   /* read() 等待超时次数 */
  uint32_t                rx_dma_busy_fail;   /* 起 DMA 返回非 HAL_OK 的次数 */
  uint32_t                rx_dma_not_armed;   /* 声称起好了但句柄没进 BUSY 的次数 */
  uint32_t                rx_dma_err_count;   /* DMA 传输错误(TE) 中断次数 */
  uint32_t                rx_recover_count;   /* 确认 RX 死掉之后做完整恢复的次数 */

  /* 自愈日志的分诊信息。光有 rx_recover_count 分不出"一次干净自愈"和
   * "复位风暴"（后者每 5 秒自愈一次，只是把症状压回去）—— 下面这几个字段让
   * 上板日志自己就能回答这个问题：
   *   rx_recover_first == 0                 → 还从没自愈过；
   *   有 first、之后 count 一直不再涨        → 一次干净自愈；
   *   last 与 first 只差几十秒而 count 在涨  → 复位风暴。 */

  clock_t                 rx_recover_first;    /* 首次自愈的时刻（0 = 没有过） */
  clock_t                 rx_recover_last;     /* 上一次自愈的时刻 */
  clock_t                 rx_recover_log_next; /* 下一次允许打日志的时刻（限频） */
  uint32_t                rx_recover_silenced; /* 限频窗口内被吞掉的自愈次数 */

  /* TE 那条日志同样要限频，但理由和自愈日志不同：超时一次要等满 5 秒，
   * 天然每秒最多一条；而 TE 会让 read **立刻**返回，一个持续性的 TE 就是
   * "上层每次重试一条"，不加窗口能把串口刷满、把别的线索冲掉。
   * 所以复用同一个 30 秒窗口长度，用自己的一对字段。 */

  clock_t                 rx_dma_err_log_next; /* 下一次允许打 TE 日志的时刻 */
  uint32_t                rx_dma_err_silenced; /* 限频窗口内被吞掉的 TE 日志条数 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  sf32lb52_audio_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                                   FAR struct audio_caps_s *caps);
static int  sf32lb52_audio_configure(FAR struct audio_lowerhalf_s *dev,
                                     FAR const struct audio_caps_s *caps);
static int  sf32lb52_audio_shutdown(FAR struct audio_lowerhalf_s *dev);
static int  sf32lb52_audio_start(FAR struct audio_lowerhalf_s *dev);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int  sf32lb52_audio_stop(FAR struct audio_lowerhalf_s *dev);
#endif
static int  sf32lb52_audio_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                         FAR struct ap_buffer_s *apb);
static int  sf32lb52_audio_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                                        FAR struct ap_buffer_s *apb);
static int  sf32lb52_audio_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                                 unsigned long arg);
static ssize_t sf32lb52_audio_write(FAR struct audio_lowerhalf_s *dev,
                                    FAR const char *buffer, size_t buflen);
static ssize_t sf32lb52_audio_read(FAR struct audio_lowerhalf_s *dev,
                                   FAR char *buffer, size_t buflen);
static int  sf32lb52_audio_reserve(FAR struct audio_lowerhalf_s *dev);
static int  sf32lb52_audio_release(FAR struct audio_lowerhalf_s *dev);

static int  sf32lb52_audio_hw_init(FAR struct sf32lb52_audio_s *priv);
static int  sf32lb52_audio_hw_configure(FAR struct sf32lb52_audio_s *priv,
                                        int samplerate, int nchannels,
                                        int bpsamp);
static int  sf32lb52_audio_hw_start(FAR struct sf32lb52_audio_s *priv,
                                    bool playback);
static int  sf32lb52_audio_hw_stop(FAR struct sf32lb52_audio_s *priv);
static int  sf32lb52_audio_hw_shutdown(FAR struct sf32lb52_audio_s *priv);
static void sf32lb52_audio_pa_enable(bool enable);
static bool sf32lb52_audio_tid_alive(pid_t tid);
static bool sf32lb52_audio_holder_alive(FAR struct sf32lb52_audio_s *priv);
static void sf32lb52_audio_path_bit(FAR bool *bit, bool on,
                                    FAR const char *name,
                                    FAR const char *why);
static void sf32lb52_audio_tx_freeze(FAR struct sf32lb52_audio_s *priv);
static void sf32lb52_audio_tx_complete(FAR struct sf32lb52_audio_s *priv);
static void sf32lb52_audio_rx_complete(FAR struct sf32lb52_audio_s *priv);
static int  sf32lb52_audio_dma1_irq(int irq, FAR void *context,
                                    FAR void *arg);
static bool sf32lb52_audio_aprc_has_config(FAR struct sf32lb52_audio_s *priv);
static void sf32lb52_audio_aprc_soft_reset(FAR struct sf32lb52_audio_s *priv);
static void sf32lb52_audio_aprc_restore_adc(FAR struct sf32lb52_audio_s *priv);
static void sf32lb52_audio_recover_log(FAR struct sf32lb52_audio_s *priv);
static int  sf32lb52_audio_recover_rx(FAR struct sf32lb52_audio_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SF32LB52X codec 时钟配置表（采样率 → 内部 PLL/分频参数，参考 SDK） */

static const AUDCODE_DAC_CLK_CONFIG_TYPE codec_dac_clk_config[SF32LB52_AUDIO_CLK_TAB_NUM] =
{
  {48000, 0, 1, 0, 0x14D, 0,  5, 4, 2, 20, 20, 0},
  {32000, 0, 1, 1, 0x14D, 0,  5, 4, 2, 20, 20, 0},
  {24000, 0, 1, 5, 0x14D, 0, 10, 2, 2, 10, 10, 1},
  {16000, 0, 1, 4, 0x14D, 0,  5, 4, 2, 20, 20, 0},
  {12000, 0, 1, 7, 0x14D, 0, 20, 2, 1,  5,  5, 1},
  { 8000, 0, 1, 8, 0x14D, 0, 10, 2, 2, 10, 10, 1},
  {44100, 1, 1, 0, 0x14D, 1,  5, 4, 2, 20, 20, 0},
  {22050, 1, 1, 5, 0x14D, 1, 10, 2, 2, 10, 10, 1},
  {11025, 1, 1, 7, 0x14D, 1, 20, 2, 1,  5,  5, 1},
};

static const AUDCODE_ADC_CLK_CONFIG_TYPE codec_adc_clk_config[SF32LB52_AUDIO_CLK_TAB_NUM] =
{
  {48000, 0,  5, 0, 0, 1, 5, 0},
  {32000, 0,  5, 1, 0, 1, 5, 0},
  {24000, 0, 10, 0, 0, 0, 5, 2},
  {16000, 0, 10, 1, 0, 0, 5, 2},
  {12000, 0, 10, 2, 0, 0, 5, 2},
  { 8000, 0, 10, 3, 0, 0, 5, 2},
  {44100, 1,  5, 0, 1, 1, 5, 1},
  {22050, 1,  5, 2, 1, 1, 5, 1},
  {11025, 1, 10, 2, 1, 0, 5, 3},
};

static const struct audio_ops_s g_sf32lb52_audio_ops =
{
  sf32lb52_audio_getcaps,       /* getcaps        */
  sf32lb52_audio_configure,     /* configure      */
  sf32lb52_audio_shutdown,      /* shutdown       */
  sf32lb52_audio_start,         /* start          */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  sf32lb52_audio_stop,          /* stop           */
#endif
  NULL,                         /* pause          */
  NULL,                         /* resume         */
  NULL,                         /* allocbuffer    */
  NULL,                         /* freebuffer     */
  sf32lb52_audio_enqueuebuffer, /* enqueue_buffer */
  sf32lb52_audio_cancelbuffer,  /* cancel_buffer  */
  sf32lb52_audio_ioctl,         /* ioctl          */
  sf32lb52_audio_read,          /* read           */
  sf32lb52_audio_write,         /* write          */
  sf32lb52_audio_reserve,       /* reserve        */
  sf32lb52_audio_release        /* release        */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sf32lb52_audio_pa_enable
 *
 * Description: 控制板上音频功放（AW8155，PA10 = AU_PA_EN）
 *
 ****************************************************************************/

static void sf32lb52_audio_pa_enable(bool enable)
{
  HAL_GPIO_WritePin(SF32LB52_AUDIO_PA_GPIO, SF32LB52_AUDIO_PA_PIN,
                    enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/****************************************************************************
 * Name: sf32lb52_audio_path_bit
 *
 * Description:
 *   改一个真实通路状态位，值真的变了才打一行 LOG_INFO（位名 + 为什么）。
 *
 *   为什么不直接 syslog：hw_start/hw_stop 每次会话都会跑到这里，无条件打
 *   就是刷屏；而"哪条模拟通路在什么时候被开了/关了"正是收尾判断的依据，
 *   串口上必须能一条一条对着看（本次改动的验收判据）。
 *   只在值变化时打，正常一次会话最多 3 行（DAC/PA/ADC 各一行）。
 *
 ****************************************************************************/

static void sf32lb52_audio_path_bit(FAR bool *bit, bool on,
                                    FAR const char *name,
                                    FAR const char *why)
{
  if (*bit == on)
    {
      return;
    }

  *bit = on;
  syslog(LOG_INFO, "AUDIO: 通路位 %s -> %d（%s）\n",
         name, (int)on, why);
}

/****************************************************************************
 * Name: sf32lb52_audio_tx_freeze
 *
 * Description:
 *   在 DMA 完成中断里**立刻**掐断 codec 的 DAC 数据通路。
 *
 *   为什么必须在中断里做：HAL_AUDCODEC_Transmit_DMA() 把 TX DMA 设成
 *   DMA_CIRCULAR（见 bf0_hal_audcodec_m.c 的 HAL_AUDCODEC_DMA_Init /
 *   HAL_AUDCODEC_Transmit_DMA），这一块传到末尾的**同时**硬件就从头再传一遍。
 *   而"停下来"原来只有 write() 被信号量唤醒之后才会调 HAL_AUDCODEC_DMAStop()，
 *   中间隔着"中断返回 → 调度器唤醒线程 → 线程真的跑起来"这段软件延迟
 *   （几十 us 到 ms 量级，看有没有更高优先级任务在跑）。这段时间里缓冲区
 *   开头会被重播，块尾跳到块首那一下就是一声"啪" —— 100 ms 一块时一秒十声，
 *   上层听到的就是"每个字都有爆音"。
 *
 *   这里在中断里直接关掉两样东西，重播长度就压到"中断入口延迟"这一个量级
 *   （不到一个 32bit 字，16k 单声道下约 2 个采样），上层再做几毫秒的淡入淡出
 *   （见 app/hello_app/ai_audio.c 的 audio_apply_chunk_fade）就完全盖住了。
 *
 *   只写寄存器，不做 HAL_DMA_Abort 那套重活（禁中断 / 回收通道 / 复位 State）：
 *   那些收尾仍然留给 write() 里的 HAL_AUDCODEC_DMAStop()。
 ****************************************************************************/

static void sf32lb52_audio_tx_freeze(FAR struct sf32lb52_audio_s *priv)
{
  FAR DMA_HandleTypeDef *hdma = priv->codec.hdma[HAL_AUDCODEC_DAC_CH0];

  /* 先关 codec 这一侧的 DMA 使能：数据不再往 DAC 里送，重播当头断掉。
   * 这一位 write() 收尾时也会清（HAL_AUDCODEC_DMAStop），这里是抢在它前面。 */

  priv->codec.Instance->DAC_CH0_CFG &= ~AUDCODEC_DAC_CH0_CFG_DMA_EN;

  /* 再把 DMA 通道停掉。只清 CCR 的 EN 位；Instance 用的是 HAL 动态分配之后
   * 实际占住的那个通道（DMA_AllocChannel 可能改过 Instance），不能写死。 */

  if (hdma != NULL)
    {
      __HAL_DMA_DISABLE(hdma);
    }
}

/****************************************************************************
 * Name: sf32lb52_audio_tx_complete / sf32lb52_audio_rx_complete
 *
 * Description:
 *   DMA 完成处理（HAL 在 DMA 中断里调用，见文件底部 HAL_AUDPRC_*
 *   CpltCallback 重写），向 NuttX 上层归还 buffer。
 *
 ****************************************************************************/

static void sf32lb52_audio_tx_complete(FAR struct sf32lb52_audio_s *priv)
{
  FAR struct ap_buffer_s *apb = priv->tx_apb;
  bool final;

  priv->tx_apb = NULL;

  if (apb == NULL)
    {
      return;
    }

  final = (apb->flags & AUDIO_APB_FINAL) != 0;

  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
  if (final)
    {
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK);
    }
}

static void sf32lb52_audio_rx_complete(FAR struct sf32lb52_audio_s *priv)
{
  FAR struct ap_buffer_s *apb = priv->rx_apb;
  bool final;

  priv->rx_apb = NULL;

  if (apb == NULL)
    {
      return;
    }

  final = (apb->flags & AUDIO_APB_FINAL) != 0;

  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
  if (final)
    {
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK);
    }
}

/****************************************************************************
 * Name: sf32lb52_audio_dma1_irq
 *
 * Description:
 *   DMA1 通道中断统一入口（把 NuttX IRQ 转给 HAL 的通道池处理，
 *   HAL 会调用对应的 XferCpltCallback → 本驱动的 Tx/RxCpltCallback）
 *
 ****************************************************************************/

static int sf32lb52_audio_dma1_irq(int irq, FAR void *context,
                                   FAR void *arg)
{
  int idx = irq - NVIC_IRQ_FIRST - DMAC1_CH1_IRQn;

  switch (idx)
    {
      case 0:  HAL_DMAC1_CH1_IRQHandler();  break;
      case 1:  HAL_DMAC1_CH2_IRQHandler();  break;
      case 2:  HAL_DMAC1_CH3_IRQHandler();  break;
      case 3:  HAL_DMAC1_CH4_IRQHandler();  break;
      case 4:  HAL_DMAC1_CH5_IRQHandler();  break;
      case 5:  HAL_DMAC1_CH6_IRQHandler();  break;
      case 6:  HAL_DMAC1_CH7_IRQHandler();  break;
      case 7:  HAL_DMAC1_CH8_IRQHandler();  break;
      default: break;
    }

  return OK;
}

/****************************************************************************
 * Name: sf32lb52_audio_hw_init
 *
 * Description:
 *   codec + AUDPRC 硬件初始化：
 *   电源/时钟使能 → HAL Init → AUDPRC TX/RX DMA 句柄配置 → DMA 中断注册
 *
 ****************************************************************************/

static int sf32lb52_audio_hw_init(FAR struct sf32lb52_audio_s *priv)
{
  HAL_StatusTypeDef res;
  int i;

  memset(&priv->codec, 0, sizeof(priv->codec));
  memset(&priv->aprc, 0, sizeof(priv->aprc));

  /* 寄存器基址（HAL 初始化会直接访问 Instance） */

  priv->codec.Instance = (AUDCODEC_TypeDef *)AUDCODEC_BASE;
  priv->aprc.Instance  = (AUDPRC_TypeDef *)AUDPRC_BASE;

  /* 打开音频电源和 codec/AUDPRC 时钟 */

  syslog(LOG_ERR, "AUDIO: PMU enable\n");
  HAL_PMU_EnableAudio(1);
  HAL_RCC_EnableModule(RCC_MOD_AUDCODEC_HP);
  HAL_RCC_EnableModule(RCC_MOD_AUDCODEC_LP);
  HAL_RCC_EnableModule(RCC_MOD_AUDPRC);
  /* DMAC1 时钟：AUDPRC TX0/RX0 DMA 走 DMAC1，必须先开时钟，否则
   * HAL_AUDPRC_Transmit_DMA 返回 OK 但 DMA 永不传输/无完成中断 */
  HAL_RCC_EnableModule(RCC_MOD_DMAC1);
  syslog(LOG_ERR, "AUDIO: clocks ok\n");

  /* codec 基本配置（模拟前端） */

  priv->codec.Init.en_dly_sel      = 0;
  priv->codec.Init.dac_cfg.opmode  = 1;
  priv->codec.Init.adc_cfg.opmode  = 1;
  priv->codec.Init.samplerate_index = 3;   /* 默认 16k，configure 时更新 */

  /* codec 自带 DMA 播放通路的 DMA 句柄延迟到第一次播放时再挂（见 hw_start）——
   * 在板级 bringup 阶段做会让启动挂住（实测黑屏） */

  syslog(LOG_ERR, "AUDIO: codec init...\n");
  res = HAL_AUDCODEC_Init(&priv->codec);
  syslog(LOG_ERR, "AUDIO: codec init res=%d\n", res);
  if (res != HAL_OK)
    {
      return -EIO;
    }

  /* AUDPRC 初始化（数据通路） */

  priv->aprc.Init.clk_div = 1;   /* 音频主时钟分频（非 ASIC 芯片） */
  priv->aprc.Init.adc_div = 1;
  priv->aprc.Init.dac_div = 1;

  /* DMA 句柄必须在 HAL_AUDPRC_Init() 之前分配并填好 Instance/Request：
   * HAL 会用音频正确的参数初始化它们（WORD 对齐 + DMA_CIRCULAR + 高优先级）。
   * 若在 Init 之后才分配（或自己配成 HALFWORD/NORMAL），AUDPRC 的 32bit FIFO
   * 收不到正确数据字，表现为播放只有开关爆音、无实际音频。 */

  priv->aprc.hdma[HAL_AUDPRC_TX_CH0] = kmm_zalloc(sizeof(DMA_HandleTypeDef));
  priv->aprc.hdma[HAL_AUDPRC_RX_CH0] = kmm_zalloc(sizeof(DMA_HandleTypeDef));
  if (priv->aprc.hdma[HAL_AUDPRC_TX_CH0] == NULL ||
      priv->aprc.hdma[HAL_AUDPRC_RX_CH0] == NULL)
    {
      auderr("ERROR: Failed to alloc DMA handle\n");
      return -ENOMEM;
    }

  priv->aprc.hdma[HAL_AUDPRC_TX_CH0]->Instance     = AUDPRC_TX0_DMA_INSTANCE;
  priv->aprc.hdma[HAL_AUDPRC_TX_CH0]->Init.Request = AUDPRC_TX0_DMA_REQUEST;
  priv->aprc.hdma[HAL_AUDPRC_RX_CH0]->Instance     = AUDPRC_RX0_DMA_INSTANCE;
  priv->aprc.hdma[HAL_AUDPRC_RX_CH0]->Init.Request = AUDPRC_RX0_DMA_REQUEST;

  syslog(LOG_ERR, "AUDIO: aprc init...\n");
  res = HAL_AUDPRC_Init(&priv->aprc);
  syslog(LOG_ERR, "AUDIO: aprc init res=%d\n", res);
  if (res != HAL_OK)
    {
      return -EIO;
    }

  /* 功放使能引脚 PA10(AUDIO_PA_CTRL)：必须先 HAL_GPIO_Init 设为输出，
   * 否则 WritePin 无效、AW8155 功放始终关闭（参照板级 BSP_GPIO_Set） */

  {
    GPIO_InitTypeDef pa_init;

    pa_init.Mode = GPIO_MODE_OUTPUT;
    pa_init.Pin  = SF32LB52_AUDIO_PA_PIN;
    pa_init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SF32LB52_AUDIO_PA_GPIO, &pa_init);
    HAL_GPIO_WritePin(SF32LB52_AUDIO_PA_GPIO, SF32LB52_AUDIO_PA_PIN,
                      GPIO_PIN_RESET);
  }

  /* 注册 DMA1 通道中断（AUDPRC TX/RX 数据传输完成依赖 DMA 中断） */

  for (i = 0; i < 8; i++)
    {
      int irq = DMAC1_CH1_IRQn + i + NVIC_IRQ_FIRST;
      int r1;

      r1 = irq_attach(irq, sf32lb52_audio_dma1_irq, priv);
      up_enable_irq(irq);
    }

  audinfo("SF32LB52 audio hw initialized\n");
  return OK;
}

/****************************************************************************
 * Name: sf32lb52_audio_hw_configure
 *
 * Description: 按采样率/通道/位深配置 codec 通路（参考 SDK bf0_audio_configure）
 *
 ****************************************************************************/

static int sf32lb52_audio_hw_configure(FAR struct sf32lb52_audio_s *priv,
                                       int samplerate, int nchannels,
                                       int bpsamp)
{
  AUDCODEC_HandleTypeDef *codec = &priv->codec;
  int i;

  for (i = 0; i < SF32LB52_AUDIO_CLK_TAB_NUM; i++)
    {
      if (samplerate == codec_dac_clk_config[i].samplerate)
        {
          codec->Init.samplerate_index = i;
          codec->Init.dac_cfg.dac_clk = (FAR AUDCODE_DAC_CLK_CONFIG_TYPE *)
                                        &codec_dac_clk_config[i];
          codec->Init.adc_cfg.adc_clk = (FAR AUDCODE_ADC_CLK_CONFIG_TYPE *)
                                        &codec_adc_clk_config[i];
          break;
        }
    }

  if (i >= SF32LB52_AUDIO_CLK_TAB_NUM)
    {
      auderr("ERROR: Unsupported samplerate %d\n", samplerate);
      return -EINVAL;
    }

  /* 配置 codec 的 DAC（播放）和 ADC（录音）通道 */

  HAL_AUDCODEC_Config_TChanel(codec, SF32LB52_AUDIO_DAC_CH,
                              &codec->Init.dac_cfg);
  HAL_AUDCODEC_Config_RChanel(codec, SF32LB52_AUDIO_ADC_CH,
                              &codec->Init.adc_cfg);

  /* 配置 AUDPRC 的 TX（播放）和 RX（录音）数据通道
   * （参考 SDK drv_audprc configure：使能通道 + 数据格式）
   *
   * 第二个参数是 HAL 的**通道号 0/1**，不是 DMA 枚举值：
   * HAL_AUDPRC_Config_TChanel/Config_RChanel 只认 0/1，别的一律走 default
   * 分支返回 HAL_ERROR 什么都不写（bf0_hal_audprc.c:293 / :333）。
   * TX0 恰好等于 0 所以一直是对的；RX 原来传的是 DMA 枚举 HAL_AUDPRC_RX_CH0(=4)，
   * 于是这一行 RX 配置**从来没生效过**（返回值也没人看）—— RX_CH0_CFG 只在
   * Receive_DMA 里被置过"通道使能/清 DMA 掩码"，格式/单双声道位一直是 0。
   * 当前 16bit 单声道下两者的寄存器值正好一样（本次改动对现场行为无影响），
   * 但换成 24bit/立体声就会静默用错格式。
   */

  {
    AUDPRC_ChnlCfgTypeDef cfg;

    memset(&cfg, 0, sizeof(cfg));
    cfg.dma_mask = 0;
    cfg.en       = 1;
    cfg.format   = (bpsamp == 16) ? 0 : 1;
    cfg.mode     = (nchannels == 1) ? 0 : 1;

    HAL_AUDPRC_Config_TChanel(&priv->aprc,
                              SF32LB52_AUDIO_PRC_TX_CH - HAL_AUDPRC_TX_CH0,
                              &cfg);
    HAL_AUDPRC_Config_RChanel(&priv->aprc,
                              SF32LB52_AUDIO_PRC_RX_CH - HAL_AUDPRC_RX_CH0,
                              &cfg);
  }

  /* AUDPRC 采样时钟分频（参考 SDK bf0_audprc_src：按采样率查表，
   * 16k → clk_div 3000，xtal 时钟源）
   */

  priv->aprc.Init.adc_div = 3000;
  priv->aprc.Init.dac_div = 3000;
  priv->aprc.Init.clk_sel = 0;
  __HAL_AUDPRC_CLK_XTAL(&priv->aprc);
  __HAL_AUDPRC_STB_DIV_CLK(&priv->aprc, priv->aprc.Init.adc_div,
                           priv->aprc.Init.dac_div);

  /* AUDPRC 通路初始化（参考 SDK bf0_adc_dac_path_cfg_init：
   * Init.adc_cfg / Init.dac_cfg 字段 → Config_ADCPath/DACPath 写寄存器）
   */

  priv->aprc.Init.adc_cfg.src_hbf3_mode = 0;
  priv->aprc.Init.adc_cfg.src_hbf3_en   = 0;
  priv->aprc.Init.adc_cfg.src_hbf2_mode = 0;
  priv->aprc.Init.adc_cfg.src_hbf2_en   = 0;
  priv->aprc.Init.adc_cfg.src_hbf1_mode = 0;
  priv->aprc.Init.adc_cfg.src_hbf1_en   = 0;
  priv->aprc.Init.adc_cfg.src_ch_en     = 0;
  priv->aprc.Init.adc_cfg.rx2tx_loopback = 0;
  priv->aprc.Init.adc_cfg.data_swap     = 0;
  priv->aprc.Init.adc_cfg.src_sel       = 0;
  priv->aprc.Init.adc_cfg.vol_l         = SF32LB52_AUDIO_ADC_VOL;
  priv->aprc.Init.adc_cfg.vol_r         = SF32LB52_AUDIO_ADC_VOL;

  priv->playback_db                     = SF32LB52_AUDIO_DEFAULT_VOL;
  priv->aprc.Init.adc_cfg.src_sinc_en   = 0;
  priv->aprc.Init.adc_cfg.sinc_ratio    = 0;

  priv->aprc.Init.dac_cfg.dst_sel       = 0;
  priv->aprc.Init.dac_cfg.vol_l         = 0;
  priv->aprc.Init.dac_cfg.vol_r         = 0;
  priv->aprc.Init.dac_cfg.src_hbf3_mode = 0;
  priv->aprc.Init.dac_cfg.src_hbf3_en   = 0;
  priv->aprc.Init.dac_cfg.src_hbf2_mode = 0;
  priv->aprc.Init.dac_cfg.src_hbf2_en   = 0;
  priv->aprc.Init.dac_cfg.src_hbf1_mode = 0;
  priv->aprc.Init.dac_cfg.src_hbf1_en   = 0;
  priv->aprc.Init.dac_cfg.src_ch_en     = 0;
  priv->aprc.Init.dac_cfg.eq_clr        = 0;
  priv->aprc.Init.dac_cfg.eq_stage      = 1;
  priv->aprc.Init.dac_cfg.eq_ch_en      = 0;
  priv->aprc.Init.dac_cfg.src_sinc_en   = 0;
  priv->aprc.Init.dac_cfg.sinc_ratio    = 0;

  /* MUX/MIX 源选择（播放输出选择，参考例程 out_sel=0x5050） */

  priv->aprc.Init.dac_cfg.muxrsrc1 = (SF32LB52_AUDIO_OUT_SEL >> 12) & 0xF;
  priv->aprc.Init.dac_cfg.muxrsrc0 = (SF32LB52_AUDIO_OUT_SEL >> 8) & 0xF;
  priv->aprc.Init.dac_cfg.muxlsrc1 = (SF32LB52_AUDIO_OUT_SEL >> 4) & 0xF;
  priv->aprc.Init.dac_cfg.muxlsrc0 = SF32LB52_AUDIO_OUT_SEL & 0xF;
  priv->aprc.Init.dac_cfg.mixrsrc1 = (SF32LB52_AUDIO_OUT_SEL >> 12) & 0xF;
  priv->aprc.Init.dac_cfg.mixrsrc0 = (SF32LB52_AUDIO_OUT_SEL >> 8) & 0xF;
  priv->aprc.Init.dac_cfg.mixlsrc1 = (SF32LB52_AUDIO_OUT_SEL >> 4) & 0xF;
  priv->aprc.Init.dac_cfg.mixlsrc0 = SF32LB52_AUDIO_OUT_SEL & 0xF;

  HAL_AUDPRC_Config_ADCPath(&priv->aprc, &priv->aprc.Init.adc_cfg);
  HAL_AUDPRC_Config_DACPath(&priv->aprc, &priv->aprc.Init.dac_cfg);

  priv->samplerate = samplerate;
  priv->nchannels  = nchannels;
  priv->bpsamp     = bpsamp;

  audinfo("codec configured: %d Hz %d ch %d bit\n",
          samplerate, nchannels, bpsamp);
  return OK;
}

/****************************************************************************
 * Name: sf32lb52_audio_aprc_has_config
 *
 * Description:
 *   本驱动是否已经往 AUDPRC 数字侧写过一套会话配置（也就是 CONFIGURE 跑过）。
 *
 *   为什么要单独拎出这个判据：软复位（sf32lb52_audio_aprc_soft_reset）是
 *   **破坏性**的 —— 它清掉 CFG.AUDCLK_DIV / STB 里的 adc·dac 分频 /
 *   ADC_PATH_CFG0 / RX_CH0_CFG，而这些值只在 hw_configure() 写过一次、靠
 *   sf32lb52_audio_aprc_restore_adc() 重写回来。破坏和重建必须用**同一个**
 *   判据，否则会留下"复位做了、重建没做"的半截状态：块被清空、配置未知，而
 *   hw_start() 接下来照样无条件 __HAL_AUDPRC_ENABLE() 把它开起来 ——
 *   一个"开着但配置是空的"数字块比"没复位过的旧块"难查得多
 *   （DMA 完成中断照来、数据全是 0，看起来一切正常）。
 *
 *   反过来（判据不成立就不复位）是安全的：samplerate == 0 说明本驱动从没写过
 *   这套配置，没有属于我们的历史状态需要清；HAL_AUDPRC_Init() 写进去的那套
 *   默认值会被下一次 CONFIGURE 整块重写。
 *
 *   现在这条路径还不可达（CONFIGURE 总在 START 之前，而 samplerate 一旦非 0
 *   就再没被清回去过），所以本判据对现场行为零影响 —— 它挡的是将来某条
 *   "没 CONFIGURE 就跑到收尾 / 先 START 后 CONFIGURE"的路径，把那种情况下
 *   的破坏性动作变回无害。
 ****************************************************************************/

static bool sf32lb52_audio_aprc_has_config(FAR struct sf32lb52_audio_s *priv)
{
  return priv->samplerate != 0;
}

/****************************************************************************
 * Name: sf32lb52_audio_aprc_soft_reset
 *
 * Description:
 *   AUDPRC 数字级软复位 —— 厂商 bf0_audio_stop() 收尾那套原样搬过来：
 *   关模块 → 清 RX 通道配置 → CFG.SRESET 拉高再拉低（一个脉冲）。
 *   源码依据：SiFli-SDK rtos/rtthread/bsp/sifli/drivers/drv_audprc.c:1673-1677
 *   （__HAL_AUDPRC_DISABLE → HAL_AUDPRC_Clear_All_Channel →
 *     __HAL_AUDPRC_SRESET_START → __HAL_AUDPRC_SRESET_STOP）。
 *   两个宏在本 tree 的 bf0_hal_audprc.h:322 / :328 都有定义，SRESET 位是
 *   CFG 的 bit1（cmsis/sf32lb52x/audprc.h:102）。
 *
 *   为什么必须有这一步（本文件原来漏了）：每帧收尾只做 DMAStop(RX) +
 *   __HAL_AUDPRC_DISABLE，AUDPRC 内部（RX FIFO 读写指针、ADC 通路抽取滤波器
 *   的状态机）**没有任何软件复位手段**。厂商在每次 stream stop 都会复位它，
 *   我们是每 20ms 拉断一次数据通路、几千次都不复位 —— 现场那次 RX 永久停止
 *   （irq 冻结在 2339、之后每次 read 都超时）就是这一类"内部状态机卡住"的形态。
 *
 *   这里用的是 Clear_Adc_Channel 而不是厂商的 Clear_All_Channel：本板播放走
 *   codec 自带 DMA、不经过 AUDPRC，只清 ADC 侧就能覆盖录音通路，顺带把对
 *   AUDPRC TX 通路（队列模式 enqueuebuffer 才用）的影响降到零。
 *
 *   只写寄存器、不等待、不睡眠 —— 所以它也能用在 hw_shutdown 那种
 *   "持上层锁 + 关中断"的上下文里。
 ****************************************************************************/

static void sf32lb52_audio_aprc_soft_reset(FAR struct sf32lb52_audio_s *priv)
{
  AUDPRC_HandleTypeDef *aprc = &priv->aprc;

  __HAL_AUDPRC_DISABLE(aprc);           /* CFG.ENABLE = 0 */
  HAL_AUDPRC_Clear_Adc_Channel(aprc);   /* RX_CH0_CFG / RX_CH1_CFG = 0 */
  __HAL_AUDPRC_SRESET_START(aprc);      /* CFG.SRESET = 1 */
  __HAL_AUDPRC_SRESET_STOP(aprc);       /* CFG.SRESET = 0（脉冲结束） */
}

/****************************************************************************
 * Name: sf32lb52_audio_aprc_restore_adc
 *
 * Description:
 *   按当前会话参数重建 AUDPRC 数字侧（ADC/RX 通路）的寄存器。
 *
 *   为什么要重跑：软复位会把这个数字块的状态清掉，而下面这些寄存器都是
 *   hw_configure() 那一次写的 —— CFG 里的 AUDCLK_DIV（bit16-19）、STB 的
 *   adc/dac 分频（bit0-15 / bit16-31）、ADC_PATH_CFG0（含录音数字增益）、
 *   RX_CH0_CFG（通道使能 + 格式）。复位之后不重写，就会出现"DMA 起得来、
 *   但数据通路配置是空的"这种更难查的坏法。
 *   写的值和 hw_configure 用的是同一份缓存（priv->aprc.Init.* 与
 *   priv->samplerate/nchannels/bpsamp），所以是幂等的，重复调用无害。
 *
 *   调用时机：hw_start() 每次会话开始（覆盖"stop 复位过"和"上次会话没收干净"
 *   两种情况）、以及 recover_rx() 的恢复流程里。CONFIGURE 从没跑过
 *   （samplerate 还是 0）时不要调 —— 那时的缓存是空的。
 ****************************************************************************/

static void sf32lb52_audio_aprc_restore_adc(FAR struct sf32lb52_audio_s *priv)
{
  AUDPRC_HandleTypeDef *aprc = &priv->aprc;
  AUDPRC_ChnlCfgTypeDef cfg;

  /* ADC 输入源 = codec（复位后 SRC_SEL 位也要重写） */

  __HAL_AUDPRC_ADC_SRC_CODEC(aprc);

  /* 主时钟分频 + xtal 时钟源 + adc/dac 分频（和 HAL_AUDPRC_Init / hw_configure
   * 写的是同一批值） */

  MODIFY_REG(aprc->Instance->CFG, AUDPRC_CFG_AUDCLK_DIV_Msk,
             MAKE_REG_VAL(aprc->Init.clk_div, AUDPRC_CFG_AUDCLK_DIV_Msk,
                          AUDPRC_CFG_AUDCLK_DIV_Pos));
  __HAL_AUDPRC_CLK_XTAL(aprc);
  __HAL_AUDPRC_STB_DIV_CLK(aprc, aprc->Init.adc_div, aprc->Init.dac_div);

  /* ADC 通路（ADC_PATH_CFG0 含左右数字增益，复位后会丢） */

  HAL_AUDPRC_Config_ADCPath(aprc, &aprc->Init.adc_cfg);

  /* RX 通道配置。第二个参数必须是 0/1 通道号，不是 DMA 枚举值
   * （HAL_AUDPRC_RX_CH0 = 4，传进去会返回 HAL_ERROR 什么都不写）。 */

  memset(&cfg, 0, sizeof(cfg));
  cfg.dma_mask = 0;
  cfg.en       = 1;
  cfg.format   = (priv->bpsamp == 16) ? 0 : 1;
  cfg.mode     = (priv->nchannels == 1) ? 0 : 1;

  HAL_AUDPRC_Config_RChanel(aprc,
                            SF32LB52_AUDIO_PRC_RX_CH - HAL_AUDPRC_RX_CH0, &cfg);
}

/****************************************************************************
 * Name: sf32lb52_audio_recover_log
 *
 * Description:
 *   自愈日志 —— "一次干净自愈"和"复位风暴"必须能一眼分开，否则"做过恢复"
 *   这句话本身什么也说明不了（两种情况下它都成立）。
 *
 *   首次自愈无条件打一条、带醒目标记；之后再自愈限频到
 *   SF32LB52_AUDIO_RECOVER_LOG_MS 一条，出窗口时把间隔和窗口内被吞掉的次数
 *   一起打出来。判读方式（rx_recover_count 也会跟着超时日志里的 recov= 一起涨）：
 *     只有"首次自愈"这一条、之后 count 一直不变
 *       → 一次干净自愈：复位治住了；
 *     "首次自愈"之后又出现"第 N 次自愈"、且 距上次 只有几秒（< STORM_MS）
 *       → 复位风暴：每 5 秒又死一次，复位只是把症状压回去，根因还在，
 *         要照 599263b 那条"按帧停/起 DMA"的根治方向继续查，不是继续复位。
 *
 *   调用方（read() 的超时分支）已经先把 rx_recover_count 加过 1，所以这里读到
 *   的就是本次自愈的序号。本函数只在任务上下文（read 超时分支）被调用。
 ****************************************************************************/

static void sf32lb52_audio_recover_log(FAR struct sf32lb52_audio_s *priv)
{
  clock_t  now = clock_systime_ticks();
  uint32_t gap_ms;

  if (priv->rx_recover_first == 0)
    {
      /* 首次自愈：这一条必须留下、而且只留一次 —— 上板先看到它，之后再也
       * 没有第二条同类日志，就是"一次干净自愈"的凭据。 */

      priv->rx_recover_first    = now;
      priv->rx_recover_last     = now;
      priv->rx_recover_log_next = now + MSEC2TICK(SF32LB52_AUDIO_RECOVER_LOG_MS);

      syslog(LOG_WARNING,
             "AUDIO: RX 首次自愈（第 %u 次）：DMAStop + SRESET + 重建 ADC/RX 配置"
             " + ENABLE；此后不再出现第二条同类日志 = 一次干净自愈\n",
             (unsigned)priv->rx_recover_count);
      return;
    }

  gap_ms = (uint32_t)TICK2MSEC(now - priv->rx_recover_last);
  priv->rx_recover_last = now;

  /* 距上次打日志还不到窗口：只累计，不打日志（窗口内每秒一条会把串口刷满，
   * 反而把有用的上下文冲掉）。比较用有符号差值，tick 回绕时也成立。 */

  if ((int32_t)(now - priv->rx_recover_log_next) < 0)
    {
      priv->rx_recover_silenced++;
      return;
    }

  syslog(LOG_WARNING,
         "AUDIO: RX 第 %u 次自愈（距首次 %u ms、距上次 %u ms）—— %s；"
         "窗口内另有 %u 次未打日志\n",
         (unsigned)priv->rx_recover_count,
         (unsigned)TICK2MSEC(now - priv->rx_recover_first),
         (unsigned)gap_ms,
         (gap_ms < SF32LB52_AUDIO_RECOVER_STORM_MS)
           ? "复位风暴：复位只压住症状、根因还在" : "间隔够长，更像偶发",
         (unsigned)priv->rx_recover_silenced);

  priv->rx_recover_silenced = 0;
  priv->rx_recover_log_next = now + MSEC2TICK(SF32LB52_AUDIO_RECOVER_LOG_MS);
}

/****************************************************************************
 * Name: sf32lb52_audio_recover_rx
 *
 * Description:
 *   RX 通路被判定"真死了"之后的完整恢复（不是只做一次 hw_start）。
 *
 *   触发判据在 read() 的超时分支里（两条同时成立才算死）：
 *     1) 整段 5 秒等待里 rx_irq_count 一次都没涨 —— 完成中断压根没来；
 *     2) 停机之前 DMA 句柄仍停在 HAL_DMA_STATE_BUSY —— 通道一直武装着，
 *        没有任何人 abort 过它。
 *   这一对条件正是现场日志那种"Receive_DMA 回 HAL_OK、not_armed 不涨、
 *   却一个完成中断都不来"的形态：DMA 侧（通道/NVIC/TC 中断使能/CNDTR）都是好的，
 *   坏在它上游不再发数据请求 —— 也就是 AUDPRC 这一侧。
 *
 *   恢复顺序（先把 DMA 收干净，再复位数字块，最后按当前会话参数重建配置）：
 *     1) DMAStop(RX)：HAL_DMA_Abort → 关 TC/HT/TE、关通道、清该通道标志、
 *        DMA_FreeChannel（HAL_NVIC_DisableIRQ + 释放通道池）；
 *        顺手把两个 State 强制摆回 READY（厂商把 HAL 里那两行注释掉了，
 *        见 bf0_hal_audprc.c:850 与 bf0_hal_dma.c:923 的差异）；
 *     2) 软复位：DISABLE + 清 RX 通道配置 + CFG.SRESET 脉冲；
 *     3) 重建 ADC/RX 寄存器配置 + 重新使能模块；
 *     4) 本次 read 仍然按契约返回 -ETIMEDOUT（没等到数据就是没等到），
 *        下一次 read 会在恢复好的块上重新武装 DMA。
 *
 *   **不碰** codec 的模拟级（ADC 模拟通路的开关位一个都不动，关中断/重复关
 *   模拟级会整机卡死，见 hw_stop 的注释）、不碰播放侧、也不改 read() 的返回值语义。
 ****************************************************************************/

static int sf32lb52_audio_recover_rx(FAR struct sf32lb52_audio_s *priv)
{
  FAR DMA_HandleTypeDef *hdma = priv->aprc.hdma[SF32LB52_AUDIO_PRC_RX_CH];

  /* ⚠️ **只在它还武装着的时候才 abort**。
   *
   * 调用方 read() 在进恢复之前已经 `HAL_AUDPRC_DMAStop(RX)` 过一次，而厂商的
   * `HAL_DMA_Abort()`（DMAStop 内部）**没有状态判断**：无条件关中断、关通道、
   * 清该通道的全部标志、再 `DMA_FreeChannel()`。通道池那一侧有 owner 检查
   * （只释放自己的槽），但**寄存器写没有** —— 重复 abort 会再写一遍该物理通道的
   * CCR/IFCR，而这几毫秒里那个通道可能已经被另一个 DMA 用户重新分配走了，
   * 结果是把别人的通道**静默关掉**（不崩，但极难查）。
   *
   * 判据用 `hdma->State`：`HAL_DMA_Abort()` 自己会把它置成 READY，
   * 所以 READY 就说明已经收过尾，不必再来一次。 */

  if (hdma == NULL || hdma->State != HAL_DMA_STATE_READY)
    {
      HAL_AUDPRC_DMAStop(&priv->aprc, SF32LB52_AUDIO_PRC_RX_CH);
    }

  if (hdma != NULL)
    {
      hdma->State = HAL_DMA_STATE_READY;
    }

  priv->aprc.State[SF32LB52_AUDIO_PRC_RX_CH] = HAL_AUDPRC_STATE_READY;

  /* 复位与重建写成一对、共用同一个判据：要么都做，要么都不做
   * （理由见 sf32lb52_audio_aprc_has_config 的注释）。 */

  if (sf32lb52_audio_aprc_has_config(priv))
    {
      sf32lb52_audio_aprc_soft_reset(priv);
      sf32lb52_audio_aprc_restore_adc(priv);
    }

  __HAL_AUDPRC_ENABLE(&priv->aprc);

  sf32lb52_audio_recover_log(priv);
  return OK;
}

/****************************************************************************
 * Name: sf32lb52_audio_hw_start
 *
 * Description:
 *   启动播放或录音通路（参考 SDK audprc 例程 start_tx/start_rx）：
 *   播放：codec DAC 模拟 → AUDPRC TX 通路 → 功放
 *   录音：codec ADC 模拟 → AUDPRC RX 通路
 *
 ****************************************************************************/

static int sf32lb52_audio_hw_start(FAR struct sf32lb52_audio_s *priv,
                                   bool playback)
{
  AUDCODEC_HandleTypeDef *codec = &priv->codec;
  AUDPRC_HandleTypeDef   *aprc  = &priv->aprc;

  /* 音频 PLL 时钟配置（播放/录音前必调，参考 SDK bf0_audio_pll_config） */


  /* 参考源/带隙基准（含 BG_CFG0 VREF_SEL、REFGEN 使能）：SDK 的 bf0_enable_pll
   * 内部会调用 HAL_TURN_ON_PLL，openvela 的 weak 版本没有 → 必须显式补齐，
   * 否则 DAC 模拟级无参考源，播放静音（录音通路的 ADCPath 自带这一步） */

  HAL_TURN_ON_PLL();

  /* SDK 的 52X 参考序列（codec_hp_sin1k_test）在此板上用 VREF_SEL=0xc（AVDD 3.3V，
   * AVDD_V18_ENABLE 未定义走 #else 分支）；openvela 的 HAL_TURN_ON_PLL 写的是 4，
   * 带隙基准不正确 → DAC 模拟级无输出（只有通路开关爆音）。这里按 SDK 值纠正。 */

  codec->Instance->BG_CFG0 &= ~AUDCODEC_BG_CFG0_VREF_SEL_Msk;
  codec->Instance->BG_CFG0 |= (0xcUL << AUDCODEC_BG_CFG0_VREF_SEL_Pos);
  up_mdelay(2);


  /* 音频 PLL 是否真的锁定：UNLOCK=1 表示未锁定 → DAC 无模拟时钟 → 静音 */

  /* 播放与录音通路一起使能：播放走 codec 自带 DMA（AUDCODEC DAC_CH0，
   * 见 write()），录音走 AUDPRC RX0；两者可独立使用 */

  /* 注意：播放数据不再经 AUDPRC（改走 codec 自带 DMA，
   * HAL_AUDCODEC_Transmit_DMA），避免 AUDPRC 的 DAC 通路与 codec DMA
   * 争抢 codec 的 DAC 数据源；AUDPRC 现在只负责录音(RX)。 */

  /* AUDPRC：ADC 输入来自 codec */

  __HAL_AUDPRC_ADC_SRC_CODEC(aprc);

  /* 每次会话开始都把 AUDPRC 数字侧的录音配置重写一遍（时钟分频 / 主分频 /
   * ADC 通路 / RX 通道格式），值全部取自 hw_configure 缓存下来的参数。
   *
   * 为什么要在这里多做一遍：hw_stop()/hw_shutdown() 收尾时会做 AUDPRC 软复位
   * （厂商序列，见 sf32lb52_audio_aprc_soft_reset 的注释），复位可能连带清掉
   * CFG.AUDCLK_DIV / STB / ADC_PATH_CFG0 / RX_CH0_CFG —— 那些原来只有 CONFIGURE
   * 那一次写过。而 sf32lb52_audio_start() 在检测到残留 running 时是
   * "stop + start"、中间**没有** CONFIGURE 的（见 sf32lb52_audio_start），
   * 不重写就会用一套空配置录音（DMA 正常起、数据全 0）。
   *
   * CONFIGURE 从没跑过时（samplerate 还是 0）跳过 —— 和 hw_stop()/hw_shutdown()
   * 里那记软复位**共用同一个判据**，两边的开关永远是同向的（见
   * sf32lb52_audio_aprc_has_config 的注释）。
   * 这几行都是普通寄存器写 —— RX_CH0_CFG 那条没有 HAL_AUDPRC_Config_DACPath
   * 里那种"等 SRC_CH_CLR_DONE"的忙等，不会在这里卡住。 */

  if (sf32lb52_audio_aprc_has_config(priv))
    {
      sf32lb52_audio_aprc_restore_adc(priv);
    }

  __HAL_AUDPRC_ENABLE(aprc);

  /* 首次播放时挂上 codec 自带 DMA 句柄（SDK 52X 的播放路径，小智固件实测有声），
   * 并让 HAL 重新初始化（它会用音频参数配置句柄并动态分配 DMA 通道） */

  if (priv->codec.hdma[HAL_AUDCODEC_DAC_CH0] == NULL)
    {
      int cret;

      priv->codec.hdma[HAL_AUDCODEC_DAC_CH0] = kmm_zalloc(sizeof(DMA_HandleTypeDef));
      priv->codec.hdma[HAL_AUDCODEC_DAC_CH1] = kmm_zalloc(sizeof(DMA_HandleTypeDef));

      if (priv->codec.hdma[HAL_AUDCODEC_DAC_CH0] != NULL)
        {
          /* Instance 必须是某个合法 DMAC 通道：DMA_AllocChannel() 用它定位通道池，
           * 为 NULL 会进 HAL_ASSERT(0)（while(1)）死循环。若该通道已被占用，
           * 分配器会自动改选池里空闲的通道。 */
          priv->codec.hdma[HAL_AUDCODEC_DAC_CH0]->Instance     = DMA1_Channel2;
          priv->codec.hdma[HAL_AUDCODEC_DAC_CH0]->Init.Request = AUDCODEC_DAC0_DMA_REQUEST;
          priv->codec.hdma[HAL_AUDCODEC_DAC_CH0]->Parent       = &priv->codec;
        }

      if (priv->codec.hdma[HAL_AUDCODEC_DAC_CH1] != NULL)
        {
          priv->codec.hdma[HAL_AUDCODEC_DAC_CH1]->Instance     = DMA1_Channel3;
          priv->codec.hdma[HAL_AUDCODEC_DAC_CH1]->Init.Request = AUDCODEC_DAC1_DMA_REQUEST;
          priv->codec.hdma[HAL_AUDCODEC_DAC_CH1]->Parent       = &priv->codec;
        }

      cret = HAL_AUDCODEC_Init(&priv->codec);
    }

  /* codec：ADC 模拟通路（录音用） */

  HAL_AUDCODEC_Config_RChanel(codec, SF32LB52_AUDIO_ADC_CH,
                              &codec->Init.adc_cfg);
  HAL_AUDCODEC_Config_Analog_ADCPath(codec->Init.adc_cfg.adc_clk);
  __HAL_AUDCODEC_ADC_ENABLE(codec);

  /* ADC 模拟通路就在这一行真正开了：如实置位。
   * 注意它**不分方向**（原来的行为就是这样）：录音要用它，播放时也跟着使能、
   * 只是数据不经过它。删除它要另外验证，不在本次改动范围里。 */

  sf32lb52_audio_path_bit(&priv->adc_path_on, true, "adc_path_on",
                          "hw_start 开 ADC 模拟通路");

  /* codec：DAC 模拟通路 + 功放 —— 只有播放才碰。
   *
   * 纯录音时打开 DAC/功放是白开：麦克风数据不经过它，但功放使能那一下会在
   * 喇叭上打出一声可听见的爆音（实测 hw_test audio 这种纯录音也能听到）。
   *
   * 播放时的顺序也不能反（SF32LB52X 无独立 HP 块，主 Instance 使能 DAC）：
   *   1. 先把模拟通路以最小音量开起来，等耦合电容充到静态电平；
   *   2. 再开功放 —— 此时 DAC 输出没有跳变；
   *   3. 最后把音量拉到目标值。
   */

  if (playback)
    {
      __HAL_AUDCODEC_DAC_ENABLE(codec);
      HAL_AUDCODEC_Config_DACPath(codec, 1);
      HAL_AUDCODEC_Config_Analog_DACPath(codec->Init.dac_cfg.dac_clk);
      HAL_AUDCODEC_Config_DACPath(codec, 0);

      /* DAC 模拟通路开完就置位：收尾时只按这个位决定要不要关它 */

      sf32lb52_audio_path_bit(&priv->dac_path_on, true, "dac_path_on",
                              "hw_start 开 DAC 模拟通路");

      HAL_AUDCODEC_Config_DACPath_Volume(codec, 0, SF32LB52_AUDIO_VOL_MUTE);
      HAL_AUDCODEC_Config_DACPath_Volume(codec, 1, SF32LB52_AUDIO_VOL_MUTE);
      up_mdelay(SF32LB52_AUDIO_SETTLE_MS);

      sf32lb52_audio_pa_enable(true);

      /* 功放使能这一下也如实记：hw_stop/hw_shutdown 只有 pa_on 为真才去关它 */

      sf32lb52_audio_path_bit(&priv->pa_on, true, "pa_on",
                              "hw_start 开功放");

      up_mdelay(20);   /* 等功放上电稳定 */

      HAL_AUDCODEC_Config_DACPath_Volume(codec, 0, priv->playback_db);
      HAL_AUDCODEC_Config_DACPath_Volume(codec, 1, priv->playback_db);

      audinfo("DAC+PA path started\n");
    }
  else
    {
      audinfo("ADC path started (record only, DAC/PA untouched)\n");
    }

  /* 到这里硬件都真起来了，才把方向"提交"为当前会话的方向。
   * 在 START 成功这一刻提交（而不是 CONFIGURE 那一刻写）是本文件的关键约定：
   * 只 CONFIGURE、没 START 的调用（ai_audio.c 的 audio_hw_set_volume() 就是）
   * 必须**不改变**任何真实通路状态和当前会话方向，否则它会替正在录音的会话
   * 把方向翻成播放。pending_playback 只表示"下一次 START 想要的方向"。 */

  priv->playback = playback;
  priv->running  = true;

  /* 记下这次会话的持有者（谁把它起起来的），并清掉上一条会话留下的
   * "最近读写的线程" —— 新会话这会儿一个字节都还没读/写。
   * 这三个 id 就是 sf32lb52_audio_start() 判"方向冲突是别人真在用还是残留"
   * 的全部依据（见 struct 里那段说明与 sf32lb52_audio_holder_alive）。 */

  priv->holder_tid = nxsched_gettid();
  priv->holder_pid = getpid();
  priv->user_tid   = (pid_t)-1;

  /* 记下这条会话是"什么时候起的"：会话开头那一小段（真正读/写数据的线程还没进
   * 第一次调用）是唯一会"两条线程证据都查不到"的窗口，sf32lb52_audio_start()
   * 靠它判宽限期，绝不在这段时间里把一条刚起来的活会话当残局强停
   * （见 SF32LB52_AUDIO_SWITCH_GRACE_MS）。 */

  priv->holder_since = clock_systime_ticks();

  /* 新会话开始：上一次会话遗留的 read()（如果有）到此作废 —— 它的会话代号
   * 已经对不上，下一次从分片等待里醒来就会退出，不会把新会话的数据接走。 */

  priv->session_gen++;

  /* 这一行是"hw_start 到底有没有跑"的唯一凭据：
   *   有它 → ADC 模拟通路 / AUDPRC 真使能过，收不到数据要去查中断和数据通路；
   *   没有 → 上层 START 根本没执行到这里（多半被残留 running 的早退挡掉了），
   *          read() 等 DMA 超时（-110）就是从这个洞进来的。 */

  syslog(LOG_INFO, "AUDIO: hw_start 真跑了：playback=%d\n", (int)playback);
  return OK;
}

/****************************************************************************
 * Name: sf32lb52_audio_hw_stop
 *
 * Description: 停止播放/录音，关闭通路和功放
 *
 ****************************************************************************/

static int sf32lb52_audio_hw_stop(FAR struct sf32lb52_audio_s *priv)
{
  /* 幂等：已经停干净就不要再关一遍。
   *
   * 最后一个 fd 被 close() 时，上层 shutdown() 会紧跟在上层的
   * AUDIOIOC_STOP 之后进来，重复关一次已经关掉的模拟通路会卡死
   * （见 sf32lb52_audio_hw_shutdown 的注释）。
   * 判据里除了 running 还要看 rx_busy/rx_aborted/rx_apb/tx_apb：
   * 这些都是"还没收尾"的状态，有一样没清就走完整流程，别漏掉 buffer。
   *
   * 三个真实通路位也必须进判据 —— 原来那五项的判据在新位参与后**不再成立**：
   * hw_shutdown() 是故意不关模拟通路的（关中断上下文里关模拟级会卡死），
   * 它只清 running、把功放关掉，于是会出现"running=false 但 dac_path_on /
   * adc_path_on 仍为真"的合法状态。这种情况下再调 hw_stop()，旧判据会直接
   * 早退，那条还开着的模拟通路就**永远漏关**了（下一位持有者一直带着它跑，
   * 直到下一次 hw_start 又把它当新的开一遍）。加上这三个位之后，
   * 收尾的语义才闭合：只要还有任何一条模拟级开着或功放开着，就走完整流程。
   *   反过来，位都是假时早退是安全的：那些行的作用全是"把 DMA / buffer /
   *   ADC 通路收掉"，没有通路开着就没有对应的东西要收；而无条件 post 出来
   *   的多余计数会被 read() 开头的 nxsem_reset() 清掉（见下面的注释）。
   */

  if (!priv->running && !priv->rx_busy && !priv->rx_aborted &&
      priv->rx_apb == NULL && priv->tx_apb == NULL &&
      !priv->dac_path_on && !priv->adc_path_on && !priv->pa_on)
    {
      /* 早退也要放一次"还卡在 read 里的任务"：设备虽然早停了，但真机上出现过
       * "stop 那次唤醒没生效"，那种情况下录音线程会一直卡在分片等待里（界面
       * 跟着遭殃）。这里补一次和完整流程末尾同样的唤醒 + 会话作废，代价只是
       * 一个信号量计数（read() 开头会 reset 掉），换来的是"任何一次 stop 调用
       * 都不可能留着别人卡住"。 */

      priv->rx_aborted = priv->rx_busy;
      priv->session_gen++;
      nxsem_post(&priv->rx_sem);
      return OK;
    }

  /* 收尾的两条 DMAStop 都要**先看句柄状态**：厂商的 `HAL_DMA_Abort()`
   * （DMAStop 内部）不看 State，无条件关中断 / 关通道 / 清该通道全部标志
   * （bf0_hal_dma.c:938-963），而对一个已经收好尾的句柄再 abort 一次，写的是
   * 那个**物理通道**的寄存器 —— 它可能已经被别的 DMA 用户重新分配走了
   * （本构建开了动态通道分配，见 recover_rx 里那段说明），等于把别人的通道
   * 静默关掉。`HAL_DMA_Abort()` 自己会把 State 置成 READY，所以"READY 就是
   * 已经收过尾"这个判据和 recover_rx 里用的完全一样。
   *
   * TX0 这一条单独说清楚（只加守卫、不删不改）：本固件的播放走 codec 自带 DMA
   * （HAL_AUDCODEC_Transmit_DMA，见 hw_start 的说明），AUDPRC TX0 只在队列模式
   * （enqueuebuffer）里才会真的起来，而本板两个 app 走的都是 read()/write()
   * 直通 —— 所以它平时一直是空转。但 sf32lb52_audio_enqueuebuffer 里那条
   * HAL_AUDPRC_Transmit_DMA 还在，队列模式真被用起来时这条 abort 就是必要的
   * 收尾，因此按"不确定就别动"的规矩**保留**；加了守卫之后它平时也不会再写
   * 别人的通道，两边的目的都达到了。 */

  if (priv->aprc.hdma[HAL_AUDPRC_TX_CH0] == NULL ||
      priv->aprc.hdma[HAL_AUDPRC_TX_CH0]->State != HAL_DMA_STATE_READY)
    {
      HAL_AUDPRC_DMAStop(&priv->aprc, HAL_AUDPRC_TX_CH0);
    }

  if (priv->aprc.hdma[HAL_AUDPRC_RX_CH0] == NULL ||
      priv->aprc.hdma[HAL_AUDPRC_RX_CH0]->State != HAL_DMA_STATE_READY)
    {
      HAL_AUDPRC_DMAStop(&priv->aprc, HAL_AUDPRC_RX_CH0);
    }

  __HAL_AUDPRC_DISABLE(&priv->aprc);

  /* AUDPRC 数字级软复位（厂商 bf0_audio_stop 收尾那套，见
   * sf32lb52_audio_aprc_soft_reset 的注释）。位置和厂商一致：两条 DMAStop
   * 之后、模块 DISABLE 之后。
   *
   * 这是"RX 跑一段时间后永久停止"的**根治**那一步：原来只有 DMAStop +
   * DISABLE，AUDPRC 内部状态机没有任何复位手段，而本驱动是每帧（20ms）都把
   * 数据通路拉断一次、从头到尾不复位。现场那次 irq 冻结在 2339、此后每次 read
   * 都超时，就是这一类内部卡死的形态。复位可能清掉的时钟/通路配置，由
   * hw_start() 里的 sf32lb52_audio_aprc_restore_adc() 每次会话开头重写，
   * 两边配套、缺一不可。
   *
   * 判据和那次重建共用（见 sf32lb52_audio_aprc_has_config）：复位是破坏性的，
   * 只有"这套配置确实写过、将来一定会被重建"时才允许做。 */

  if (sf32lb52_audio_aprc_has_config(priv))
    {
      sf32lb52_audio_aprc_soft_reset(priv);
    }

  /* 播放用的 DAC DMA 也要停，而且**必须停**：
   * HAL 把音频 DMA 初始化成 DMA_CIRCULAR（见 hw_init 的注释：WORD 对齐 +
   * 循环 + 高优先级），也就是传输结束后硬件自己从头再来一遍。write() 正常
   * 返回时它自己会 DMAStop，但"写失败提前 break""上层直接 close""stop 时
   * 正阻塞在 write 里"这几条路上没人停它 —— 最后写进去的那 100 ms 就会
   * 无限重播，现场听感就是"昂昂昂昂"卡住不停。
   * 只动寄存器，和上面两行 AUDPRC DMAStop 一个量级，不碰模拟通路。 */

  HAL_AUDCODEC_DMAStop(&priv->codec, HAL_AUDCODEC_DAC_CH0);
  priv->codec.State[HAL_AUDCODEC_DAC_CH0] = HAL_AUDCODEC_STATE_READY;

  /* 清掉 HAL 的通道状态，否则下一次 Transmit/Receive DMA 会返回 HAL_BUSY */
  priv->aprc.State[HAL_AUDPRC_TX_CH0] = HAL_AUDPRC_STATE_READY;
  priv->aprc.State[HAL_AUDPRC_RX_CH0] = HAL_AUDPRC_STATE_READY;

  /* 唤醒可能正阻塞在 read() 里的任务。
   *
   * read() 等的是 priv->rx_sem，正常由 DMA 完成中断 post。stop 时 DMA 被停掉，
   * 中断不会再来 —— 必须在这里补一次，否则 read 只能干等它自己的超时。
   * **无条件 post**（不再只在 rx_busy 时才 post）：rx_busy 的读写和中断有竞态，
   * 实测出现过"stop 那一下刚好看到 rx_busy==false 就没唤醒"，录音线程卡在
   * read 里出不来，而 join 它的正是 LVGL 线程 —— 界面整块卡死（提交按钮一直
   * 不变色）。配对措施见 read()：起 DMA 前先 nxsem_reset()，多余的计数不会
   * 被下一次 read 当成"采集完成"。
   */

  syslog(LOG_INFO, "AUDIO: stop 唤醒 read（rx_busy=%d running=%d）\n",
         (int)priv->rx_busy, (int)priv->running);

  /* 有 read 在等：置 abort 标记（让它按 EOF 收场，不能把半截缓冲当数据交上去）。
   * 没有 read 在等：把标记清掉，别留下脏状态影响本函数开头的"幂等早退"判断。 */

  priv->rx_aborted = priv->rx_busy;

  /* **无条件** post：rx_busy 与中断之间有竞态，只在 busy 时 post 会漏唤醒
   * （实测那次漏了）。配对措施见 read()：起 DMA 前先 nxsem_reset()，多余计数
   * 不会被下次 read 当成"采集完成"。 */

  priv->session_gen++;      /* 会话作废：read 即使没被这次 post 唤醒，下一片醒来也能自己退出 */

  nxsem_post(&priv->rx_sem);

  /* 把还挂着的 buffer 归还给上层。
   *
   * 上层（NuttX audio 框架）的 read()/write() 是靠下层回调
   * AUDIO_CALLBACK_DEQUEUE 来唤醒的。如果 stop 时 buffer 还挂在 DMA 上
   * 却不归还，正在阻塞的 read() 永远等不到回调 —— 线程就此卡死。
   * 实测：录音 read 阻塞中发 AUDIOIOC_STOP，3 秒都不返回。
   * （DMA 已经停了，这时归还 buffer 是安全的）
   *
   * 只有**确实挂着本会话的 buffer** 时才回调：两个指针都只有队列模式
   * （上层 AUDIOIOC_ENQUEUEBUFFER）才会被赋值，本板两个 app 走的都是直通
   * read()/write()，所以直接通道下这里恒为 NULL、一个回调都不会发出去。
   * 这一点很重要：上层 audio_dequeuebuffer() 会写 upper->status，而
   * upper->status 在"最后一个 fd 被 close"时就被释放并置 NULL 了 —— 真要
   * 在那种时刻回调进去就是空指针。保留"有 buffer 才回调"这个惯判据，
   * 就同时挡住了"打向正在销毁的 task group"和"打向已释放的 upper 状态"。
   */

  if (priv->rx_apb != NULL)
    {
      FAR struct ap_buffer_s *apb = priv->rx_apb;

      priv->rx_apb = NULL;
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
    }

  if (priv->tx_apb != NULL)
    {
      FAR struct ap_buffer_s *apb = priv->tx_apb;

      priv->tx_apb = NULL;
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
    }

  /* 关断顺序必须和开机顺序反过来：先把 DAC 拉到最小音量，再关功放，
   * 最后才断模拟通路。原来的顺序（先 Close_Analog_DACPath 再关功放）是
   * 把还开着的 DAC 输出直接抽掉，那一下直流跳变经功放放大 = 结尾一声爆音。
   *
   * 每一步只看**真实通路位**，不看方向标签：标签是别的 app 一次 CONFIGURE
   * 就能改的谎话（见 struct 里的注释），按标签决定就会去关一条从没开过的
   * 模拟通路（重复关模拟级实测整机静默卡死），或者漏关真正开着的 DAC/功放
   * （录音把标签翻回 false 之后播放收尾就跳过了）。不变量：
   *   关之前一定开着、关完立刻清位，位关闭一次、开一次严格配对。
   */

  if (priv->dac_path_on)
    {
      HAL_AUDCODEC_Config_DACPath_Volume(&priv->codec, 0,
                                         SF32LB52_AUDIO_VOL_MUTE);
      HAL_AUDCODEC_Config_DACPath_Volume(&priv->codec, 1,
                                         SF32LB52_AUDIO_VOL_MUTE);
      up_mdelay(SF32LB52_AUDIO_SETTLE_MS);
    }

  if (priv->pa_on)
    {
      sf32lb52_audio_pa_enable(false);
      sf32lb52_audio_path_bit(&priv->pa_on, false, "pa_on",
                              "hw_stop 关功放");

      /* 留 5ms 让功放放电：紧接着（DAC 还开着时）就要断模拟通路，
       * 顺序反了那一下跳变会被还激活的功放放大成爆音。 */

      up_mdelay(5);
    }

  if (priv->dac_path_on)
    {
      HAL_AUDCODEC_Close_Analog_DACPath();
      sf32lb52_audio_path_bit(&priv->dac_path_on, false, "dac_path_on",
                              "hw_stop 关 DAC 模拟通路");
    }

  /* ADC 模拟通路同理：只有真开着才关。原来这里是无条件关的，加上新判据之后
   * 就危险了 —— 本函数现在会在"running=false、只剩某一两个通路位为真"的状态下
   * 也落进来（比如 hw_shutdown 之后又走到收尾），那种状态下 ADC 通路很可能
   * 早关掉了，无条件关一次就是重复关模拟级（实测整机静默卡死）。 */

  if (priv->adc_path_on)
    {
      HAL_AUDCODEC_Close_Analog_ADCPath();
      sf32lb52_audio_path_bit(&priv->adc_path_on, false, "adc_path_on",
                              "hw_stop 关 ADC 模拟通路");
    }

  priv->running = false;

  /* running 一清，这套持有者 id 就没有意义了。不清的话，下一个会话看到的是
   * 上一条会话的线程 id，判"持有者还在不在"就会拿错人。 */

  priv->holder_tid = (pid_t)-1;
  priv->holder_pid = (pid_t)-1;
  priv->user_tid   = (pid_t)-1;

  audinfo("audio stopped\n");
  return OK;
}

/****************************************************************************
 * Name: sf32lb52_audio_hw_shutdown
 *
 * Description:
 *   给 close() / shutdown() 用的最小化停止。
 *
 *   为什么不能照搬 hw_stop()：
 *   1. 上层 audio_close() 在关最后一个 fd 时是
 *      nxmutex_lock(upper->lock) → spin_lock_irqsave(upper->spinlock)
 *      （非 SMP 下等于关中断）之后，才在持锁关中断的状态里调
 *      lower->ops->shutdown()。所以这里既不能回调上层
 *      （upper 的锁被自己拿着），也不该做耗时的模拟级操作。
 *   2. 同一次会话里 AUDIOIOC_STOP 已经关过一次模拟通路，重复调
 *      HAL_AUDCODEC_Close_Analog_DACPath()/ADCPath() 关已经关掉的模拟级
 *      会整机卡死（实测：STOP 返回 0 后紧接着 close() 静默卡住、不复位、
 *      无 panic）。这正是 hw_test/audio_test 不犯的原因——它们的 fd 不是
 *      最后一个，shutdown() 压根没被调到。
 *
 ****************************************************************************/

static int sf32lb52_audio_hw_shutdown(FAR struct sf32lb52_audio_s *priv)
{
  /* DMAStop / DISABLE 只做寄存器操作，风险低于模拟通路关闭，先保留；
   * 这两个本身在关中断上下文里是否安全还没有证据，若仍卡死，
   * 下一步就把这两行也挪出 shutdown。
   *
   * 两条 DMAStop 同样加"先看句柄状态"的守卫，理由和 hw_stop 里那对一模一样
   * （见那段说明）：READY 就说明上一次 abort 已经收过尾，再 abort 一次写的是
   * 可能已被别人分配走的物理通道。TX0 同样是"平时空转、队列模式才用得上"，
   * 所以保留、只加守卫。 */

  if (priv->aprc.hdma[HAL_AUDPRC_TX_CH0] == NULL ||
      priv->aprc.hdma[HAL_AUDPRC_TX_CH0]->State != HAL_DMA_STATE_READY)
    {
      HAL_AUDPRC_DMAStop(&priv->aprc, HAL_AUDPRC_TX_CH0);
    }

  if (priv->aprc.hdma[HAL_AUDPRC_RX_CH0] == NULL ||
      priv->aprc.hdma[HAL_AUDPRC_RX_CH0]->State != HAL_DMA_STATE_READY)
    {
      HAL_AUDPRC_DMAStop(&priv->aprc, HAL_AUDPRC_RX_CH0);
    }

  __HAL_AUDPRC_DISABLE(&priv->aprc);

  /* 这里同样补软复位：close 掉最后一个 fd 是"把 AUDPRC 留在关着且没复位"
   * 的最后一次机会，下一个 app 打开时才能从一个干净的块上开始。
   * 只写 CFG/RX_CH0_CFG 两个寄存器、不等待不睡眠，所以在本函数所处的
   * "持上层锁 + 中断关闭"上下文里是安全的（和上面几条 DMAStop 一个量级）。
   * 复位清掉的配置由下一次 hw_start() 的 sf32lb52_audio_aprc_restore_adc()
   * 重写。判据与那次重建共用（见 sf32lb52_audio_aprc_has_config）：
   * 能重建才允许复位，避免留下"清空过、但没人重建"的块。 */

  if (sf32lb52_audio_aprc_has_config(priv))
    {
      sf32lb52_audio_aprc_soft_reset(priv);
    }

  /* DAC 的 DMA 同样要停：它是 DMA_CIRCULAR，不停就会无限重播最后一段
   * （"昂昂昂昂"卡住不停）。close() 是最后一个 fd 被关时的收尾路径，
   * 上一段播放如果没能自己停掉，这里就是唯一的机会。
   * 顺带把"停的时候还在传"这个异常打出来 —— 正常收尾时 DAC 状态早该是
   * READY，还带 BUSY_TX 就说明上一次传输没走完，下一次 write 会因此起不来。 */

  if ((priv->codec.State[HAL_AUDCODEC_DAC_CH0] & HAL_AUDCODEC_STATE_BUSY_TX) != 0)
    {
      syslog(LOG_WARNING,
             "AUDIO: shutdown 时 DAC 仍是 BUSY_TX(0x%x)，传输没收尾，已停\n",
             priv->codec.State[HAL_AUDCODEC_DAC_CH0]);
    }

  HAL_AUDCODEC_DMAStop(&priv->codec, HAL_AUDCODEC_DAC_CH0);
  priv->codec.State[HAL_AUDCODEC_DAC_CH0] = HAL_AUDCODEC_STATE_READY;

  priv->aprc.State[HAL_AUDPRC_TX_CH0] = HAL_AUDPRC_STATE_READY;
  priv->aprc.State[HAL_AUDPRC_RX_CH0] = HAL_AUDPRC_STATE_READY;

  /* 关功放是普通 GPIO 写，安全；但判据只能用真实位 pa_on，不能用方向标签
   * （标签会被别的 app 的一次 CONFIGURE 改掉：标签说"录音"时功放可能正开着，
   * 那就漏关；标签说"播放"时功放可能从没开过，那就是白关一下）。
   * 注意这条路径在**持上层锁 + 关中断**的上下文里（见本函数上面的注释），
   * 所以这里的 syslog 只在位真的变化时才出现 —— 正常收尾一次都不会打。 */

  if (priv->pa_on)
    {
      sf32lb52_audio_pa_enable(false);
      sf32lb52_audio_path_bit(&priv->pa_on, false, "pa_on",
                              "hw_shutdown 关功放");
    }

  /* 这里**不**关 DAC/ADC 模拟通路，也不清这两个位：关中断上下文里关模拟级
   * 会整机卡死（见上面的注释）。留下的"通路还开着"由下一次 hw_stop() 收尾 ——
   * 它的早退判据已经把这两个位算进去了（见 hw_stop 里的说明）。 */

  /* 最后一件必须做的事：把可能正阻塞在 read() 里的任务放出来。
   *
   * 这条路径只做 DMA/寄存器和 GPIO，原来一句唤醒都没有 —— 于是"最后一个 fd
   * 被 close、但没人发过 STOP"时（fd 被上层收掉、或 STOP 压根没送到驱动），
   * 录音线程只能靠 read() 自己那 5 秒分片超时出来，收尾慢得离谱（真机上表现
   * 就是 join 等 300ms 直接放弃）。这里的 post 与 hw_stop 里那句同一目的：
   * 给出一个不依赖任何超时的出口。
   *
   * 关于上下文：本函数在"持 upper->lock + 关中断"里被调（见函数头的说明），
   * nxsem_post() 在这种上下文里是合法的（它内部自己关中断，中断处理里也照样
   * 能调），不会去碰模拟级、也不会回调上层 —— 和上面刻意避开的那两类操作不同。
   * 顺序上 DMA 已经在前面停掉了，所以被唤醒的 read() 不会收到"采集完成"的假象。 */

  priv->rx_aborted = priv->rx_busy;
  priv->session_gen++;
  nxsem_post(&priv->rx_sem);

  priv->running = false;

  /* 和 hw_stop() 一样要清持有者 id：close() 之后 running 变假，留着上一条会话的
   * 线程 id 只会把下一次判定带偏（本函数所在的上下文不能加别的活，清几个
   * pid_t 只是普通赋值）。 */

  priv->holder_tid = (pid_t)-1;
  priv->holder_pid = (pid_t)-1;
  priv->user_tid   = (pid_t)-1;
  return OK;
}

/****************************************************************************
 * Name: sf32lb52_audio_getcaps
 *
 * Description: 上报设备能力（输入+输出、PCM、采样率）
 *
 ****************************************************************************/

static int sf32lb52_audio_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                                  FAR struct audio_caps_s *caps)
{
  DEBUGASSERT(caps && caps->ac_len >= sizeof(struct audio_caps_s));

  caps->ac_format.hw  = 0;
  caps->ac_controls.w = 0;

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_QUERY:
        if (caps->ac_subtype == AUDIO_TYPE_QUERY)
          {
            /* 支持输入（麦克风）和输出（喇叭），PCM 格式 */

            caps->ac_controls.b[0] = AUDIO_TYPE_INPUT |
                                     AUDIO_TYPE_OUTPUT;
            caps->ac_format.hw     = 1 << (AUDIO_FMT_PCM - 1);
          }
        else
          {
            caps->ac_controls.b[0] = AUDIO_SUBFMT_END;
          }
        break;

      case AUDIO_TYPE_INPUT:
      case AUDIO_TYPE_OUTPUT:
        if (caps->ac_subtype == AUDIO_TYPE_QUERY)
          {
            caps->ac_controls.hw[0] = AUDIO_SAMP_RATE_8K |
                                      AUDIO_SAMP_RATE_16K |
                                      AUDIO_SAMP_RATE_44K |
                                      AUDIO_SAMP_RATE_48K;
            caps->ac_channels = 1;
          }
        else
          {
            caps->ac_controls.b[0] = AUDIO_SUBFMT_END;
          }
        break;

      case AUDIO_TYPE_FEATURE:
        if (caps->ac_subtype == AUDIO_FU_UNDEF)
          {
            caps->ac_controls.b[0] = AUDIO_FU_VOLUME;
          }
        break;

      default:
        caps->ac_subtype = 0;
        caps->ac_channels = 0;
        break;
    }

  return caps->ac_len;
}

/****************************************************************************
 * Name: sf32lb52_audio_configure
 *
 * Description: 处理上层 AUDIOIOC_CONFIGURE
 *
 ****************************************************************************/

static int sf32lb52_audio_configure(FAR struct audio_lowerhalf_s *dev,
                                    FAR const struct audio_caps_s *caps)
{
  FAR struct sf32lb52_audio_s *priv =
    (FAR struct sf32lb52_audio_s *)dev;
  int ret = OK;

  /* 方向只在这里记成"下一次 START 的意图"，**不动**真实通路状态、也不动
   * 当前会话方向（真实状态要等 sf32lb52_audio_hw_start() 真的开起硬件才提交）。
   *
   * 原来这两行是直接写 priv->playback 的，而 ai_audio.c 的
   * audio_hw_set_volume() 是 open + CONFIGURE(OUTPUT) + close、**不 START**：
   * 它一进来就把正在录音的那个会话的方向标签翻成"播放"，之后录音侧收尾就
   * 变成"按播放收尾"——去关一条没开的 DAC 通路，同时漏关真正开着的 ADC 通路。
   * 见 struct 里真实通路位那一段注释。 */

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_INPUT:
        priv->pending_playback = false;
        ret = sf32lb52_audio_hw_configure(priv,
                                          caps->ac_controls.hw[0] |
                                          (caps->ac_controls.b[3] << 16),
                                          caps->ac_channels,
                                          caps->ac_controls.b[2]);
        break;

      case AUDIO_TYPE_OUTPUT:
        priv->pending_playback = true;
        ret = sf32lb52_audio_hw_configure(priv,
                                          caps->ac_controls.hw[0] |
                                          (caps->ac_controls.b[3] << 16),
                                          caps->ac_channels,
                                          caps->ac_controls.b[2]);
        break;

      case AUDIO_TYPE_FEATURE:
        if (caps->ac_format.hw == AUDIO_FU_VOLUME)
          {
            /* 标准音量接口：AUDIOIOC_SETPARAMTER + AUDIO_TYPE_FEATURE +
             * AUDIO_FU_VOLUME，值域 0..AUDIO_VOLUME_MAX(1000)。
             *
             * HAL 只接受 -36..+60 dB，**必须做映射** —— 原来把 0..1000
             * 直接当 dB 传进去，大于 60 的一律返回 HAL_ERROR，
             * 等于上层设音量根本没生效。
             *
             * 按方向生效：播放改 DAC 音量，录音改 ADC 数字增益
             * （这套 feature unit 里没有单独的麦克风增益控制，只能这样暴露）。
             *
             * 判据用"刚 CONFIGURE 的方向意图 + 那条通路真的开着"，
             * 不再用 running + playback 这个方向标签：
             *   - playback 只在 START 成功时才提交，别的 app 一次 CONFIGURE
             *     就能改掉它，拿它分方向会把音量下到错的通路上；
             *   - running 也可能是残留（上一个持有者已死），拿它当"真在录"
             *     会去改根本没在用的 ADC 增益。
             * 音量的数值先记进 playback_db（下一次播放启动时由 hw_start 应用），
             * 只有 DAC 模拟通路真开着才顺手把音量写进寄存器 —— 通路没开时写它
             * 等于去碰一条没打开的模拟级，没有必要。
             */

            uint16_t volume = caps->ac_controls.hw[0];

            if (volume > AUDIO_VOLUME_MAX)
              {
                ret = -EDOM;
              }
            else
              {
                int db = SF32LB52_AUDIO_VOL_MIN +
                         (SF32LB52_AUDIO_VOL_MAX - SF32LB52_AUDIO_VOL_MIN) *
                         (int)volume / AUDIO_VOLUME_MAX;

                if (!priv->pending_playback && priv->adc_path_on)
                  {
                    priv->aprc.Init.adc_cfg.vol_l = db;
                    priv->aprc.Init.adc_cfg.vol_r = db;
                    HAL_AUDPRC_Config_ADCPath(&priv->aprc,
                                              &priv->aprc.Init.adc_cfg);
                  }
                else
                  {
                    priv->playback_db = db;

                    if (priv->dac_path_on)
                      {
                        HAL_AUDCODEC_Config_DACPath_Volume(&priv->codec, 0, db);
                        HAL_AUDCODEC_Config_DACPath_Volume(&priv->codec, 1, db);
                      }
                  }

                audinfo("volume %d/1000 -> %d dB\n", volume, db);
              }
          }
        break;

      default:
        audwarn("WARNING: Unsupported configure type %d\n", caps->ac_type);
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: sf32lb52_audio_shutdown
 *
 * Description:
 *   最后一个 fd 被 close() 时，上层在**持 upper->lock + 关中断**的状态下
 *   调到这里。所以只能走最小化停止，不能走完整的 hw_stop()
 *   （回调上层 + 重复关模拟通路都会卡死，详见 hw_shutdown 的注释）。
 *
 ****************************************************************************/

static int sf32lb52_audio_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct sf32lb52_audio_s *priv =
    (FAR struct sf32lb52_audio_s *)dev;

  /* 这里**故意不**用 priv->running 早退（原来那句 `if (!priv->running)
   * return OK;` 已删）：close() 是应用退出后的最后一道收尾，而残留
   * running==true 恰恰说明上一个持有者没走干净。无条件跑一遍
   * hw_shutdown()，把 running 和各个 DMA/通道状态清到底，
   * 下一个 app 才有机会拿到一个干净的会话（否则就是"ps 里没有那个 app 了，
   * 但 hw_test 的 audio_in_start 一直 -16 EBUSY"）。 */

  return sf32lb52_audio_hw_shutdown(priv);
}

/****************************************************************************
 * Name: sf32lb52_audio_tid_alive
 *
 * Description:
 *   这个线程 id 对应的任务还在不在（还在返回 true）。
 *
 *   为什么用 nxsched_get_tcb() 而不是 kill(pid, 0) 看 ESRCH：NuttX 里线程退出时
 *   nxsched_release_pid() 会把它的 pid 从 pidhash 里摘掉，三条退出路径都调
 *   （task/exit.c、pthread/pthread_exit.c、task/task_terminate.c），所以"查不到
 *   TCB"就等于"这个线程真的没了"，语义比信号那套干净得多 —— kill() 是"给某个
 *   任务组发信号"，拿线程 id 去问会先过权限/组那一层，ESRCH 和 EPERM 分不干净，
 *   把"没权限"当成"不存在"就是替一个活着的会话拆台。
 *   查到了必须配对释放（引用计数，见 sched/sched/sched_gettcb.c）。
 *
 *   还要把"取值失败"和"没记录（-1）"分开：记进来的值有可能是
 *   nxsched_gettid() 的失败返回值 -ESRCH，那不是"没记录"，见下面的实现。
 *
 ****************************************************************************/

static bool sf32lb52_audio_tid_alive(pid_t tid)
{
  FAR struct tcb_s *tcb;

  /* -ESRCH 不是"没记录"，是取值失败：nxsched_gettid() 的 inline 实现只在
   * 调用者真的处于 RUNNING 态时才回 tid，否则回 -ESRCH
   * （openvela/nuttx/include/nuttx/sched.h 里那段实现），而 hw_start/read/write
   * 是把这个返回值**原样**记进 holder_tid/user_tid 的。
   * 取值失败时我们并没有"这个线程没了"的证据，所以按**活着**处理（保守）：
   * 和 -1 一起归零就等于拿一个查不到的 id 去判"持有者已死"，替活会话拆台。 */

  if (tid == (pid_t)-ESRCH)
    {
      return true;
    }

  if (tid < 0)
    {
      return false;
    }

  tcb = nxsched_get_tcb(tid);
  if (tcb == NULL)
    {
      return false;
    }

  nxsched_put_tcb(tcb);
  return true;
}

/****************************************************************************
 * Name: sf32lb52_audio_holder_alive
 *
 * Description:
 *   当前这次会话（running 为真）还有没有有效持有者。
 *
 *   三条独立证据，任一条成立就算"还活着"：
 *     1) 把这次会话起起来的线程还在；
 *     2) 最近一次 read()/write() 的线程还在（会话真的有人在用）；
 *     3) 起会话的那个任务组还在（app 没退）。
 *   三条都查不到 TCB 才判"没有有效持有者" —— 这时候反方向那条通路已经没人要了，
 *   调用方可以强制收干净再切过去（调用方还有一条宽限期，见 sf32lb52_audio_start）。
 *
 *   第 3 条为什么是"判残留"的**必要条件**（上一轮把它只当日志，代价是一整条误伤
 *   路径）：本板真有"起会话的线程是短命线程"的用法 —— robot_ui 用
 *   PTHREAD_CREATE_DETACHED 起的 voice_open_thread 在 audio_in_start() 之后立刻
 *   return NULL；ai_audio.c 在播放线程里恢复录音、随后播放线程自己退出。
 *   于是在"新会话刚 START 成功 → 起会话的线程已退出 → 真正读数据的线程还没进
 *   第一次 read()（user_tid 还是 -1）"这个窗口里，前两条证据**必然**都是空的，
 *   可会话是**活的**：光看线程就会把这条活会话判成残局强停，后果比原来的 -110
 *   严重得多（现场把正在录音/正在播报的会话拆掉）。
 *   NuttX 里 tg_pid == 组长 pid（holder_pid 取的就是 getpid()，nxsched_get_tcb
 *   按 pidhash 查），所以"holder_pid 也查不到 TCB"**等价于"整个 app 没了"**：
 *   这时候才真的是"app 死了留下残局、没人会再来收"。这条判据一刀砍掉上面那个
 *   误伤窗口（那两个场景里 app 都活着），而真正要治的场景仍然覆盖得到。
 *
 *   代价是**有意的取舍**：app 还活着、但它那条播放线程真的死了时，这种残留不再
 *   被认出来，录音端退化成 -EBUSY —— 交给上层重试，或由那个 app 自己手里的 fd
 *   走 close()/AUDIOIOC_STOP 收尾。相比"误停一条活会话"，这个代价可以接受。
 *
 *   判"活着"时调用方的行为**一个字都不能变**（仍然 -EBUSY）：半双工下录放必须
 *   串行，不能替正在用的人把通路拆了。
 *
 ****************************************************************************/

static bool sf32lb52_audio_holder_alive(FAR struct sf32lb52_audio_s *priv)
{
  /* 一个都没记过（理论上到不了：running 只由 hw_start 置真，那里一定会记），
   * 或者记下的值是 nxsched_gettid() 的失败值 -ESRCH（同样说明不了任何事）：
   * 判不了就按"有人"处理，保持原来的 -EBUSY 行为。 */

  if (priv->holder_tid < 0 && priv->user_tid < 0)
    {
      return true;
    }

  if (sf32lb52_audio_tid_alive(priv->holder_tid) ||
      sf32lb52_audio_tid_alive(priv->user_tid))
    {
      return true;
    }

  /* 两条线程证据都没了，还差最后一条：**整个任务组也没了**才叫残留。
   * 组还在 = app 还活着（上面那两个短命线程的场景都落在这里）→ 按"有人"处理。 */

  return sf32lb52_audio_tid_alive(priv->holder_pid);
}

/****************************************************************************
 * Name: sf32lb52_audio_start
 *
 * Description: 处理上层 AUDIOIOC_START
 *
 ****************************************************************************/

static int sf32lb52_audio_start(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct sf32lb52_audio_s *priv =
    (FAR struct sf32lb52_audio_s *)dev;
  bool want;
  bool live;
  bool dir;
  uint32_t age_ms;

  /* 本次要起的方向：取最近一次 CONFIGURE 记下的意图（START 成功才提交成
   * priv->playback）。不加这一步而直接读 priv->playback 的话，这里的"要起的
   * 方向"其实是上一次会话的方向 —— 而这个函数在方向不同的时候必须拒绝，
   * 判错方向就等于替别人把正在用的通路拆了。 */

  want = priv->pending_playback;

  /* running 已经是 true 时**不能**直接 return OK（原来就是这么写的，把整段
   * hw_start 跳过了）：
   *
   * 这套硬件是半双工，设计上同一时刻只有一个持有者；NuttX 上层只在
   * PREPARED→RUNNING 时才调 start，所以 running==true 只可能是
   *   1) 上一个持有者（app）录音中途挂掉/退出，没走 close → 残留；
   *   2) 同一方向的重复 START（同一个 app 自己重来一次），旧通路还没收尾。
   * 两种情况都需要先把旧通路收干净。直接 OK 走人的后果是：ADC 模拟通路、
   * AUDPRC 使能（都在 hw_start 里）全没做，read() 的 DMA 起得来却永远等不到
   * 完成中断，5 秒后以 -110 收场，应用从此变聋。
   *
   * 但"先完整 stop 再 start"这件事**只在两种情况下**才允许做：
   *   ① 方向一致（上一个持有者和本次要起的是同一条通路，收尾不会动到别人）；
   *   ② 方向不一致，但那条通路的持有者线程已经不存在了（死掉的那个 app/线程
   *      留下的残局，见下面的异方向分岔）。
   * 除这两种以外一律拒绝。两个 app 同时在线、另一个方向的通路是别人正在用的，
   * 拆掉它的后果是录音被中止（或播放只剩第一块），比拒绝这次 START 严重得多。
   * 所以这里按"真实在跑的方向 + 持有者还在不在"分岔。
   */

  if (priv->running)
    {
      /* "真实在跑的方向"= 已提交的方向 + 至少一条通路位确实开着。
       * 三个位全假说明 running 是残留（没有对应的硬件状态），按"没有方向"
       * 处理：那种情况下任何方向都只是残局，可以收尾后重起。
       * 这里**不**看 priv->pending_playback 之外的东西：pending 是本次要起的
       * 方向，把两者比一比就知道是不是在抢别人的通路。 */

      live = priv->dac_path_on || priv->pa_on || priv->adc_path_on;
      dir  = priv->playback;

      if (live && dir != want)
        {
          /* 异方向：先看对面是不是真有活人在用。
           *
           * 持有者还活着 → **拒绝**，不替对方拆台。返回 -EBUSY 是安全的：上层
           * 两个 app 都按 -16 处理 —— audio_in_start() 把它透出来（它自己有一拍
           * 重试），robot_ui 的播报路径报 "audio open failed" 后当次静默走完，
           * 都不会因为这次拒绝崩掉。
           *
           * 持有者线程已经不存在、并且整个任务组也没了（= 那个 app 真死了）→ 这是
           * 死掉的那个 app 留下的残局：没人会再来收，这里不收的话反方向的 START 就
           * 永远卡在 -EBUSY，录音端从此永久聋到重启板子。
           * 判定依据见 sf32lb52_audio_holder_alive（线程 + 任务组三条证据）以及下面
           * 的宽限期（会话刚起来的那一小段绝不强停）。 */

          if (sf32lb52_audio_holder_alive(priv))
            {
              syslog(LOG_WARNING,
                     "AUDIO: 拒绝方向切换：要起 %s，但 %s 通路正被占用"
                     "（dac=%d pa=%d adc=%d running=%d 持有线程=%d 最近读写=%d "
                     "组=%d）→ -EBUSY\n",
                     want ? "播放" : "录音",
                     dir ? "播放" : "录音",
                     (int)priv->dac_path_on, (int)priv->pa_on,
                     (int)priv->adc_path_on, (int)priv->running,
                     (int)priv->holder_tid, (int)priv->user_tid,
                     (int)priv->holder_pid);
              return -EBUSY;
            }

          /* 会话还没过宽限期 → **绝不** force-stop，只照原样返回 -EBUSY。
           *
           * 误伤窗口永远在会话开头：刚 START 成功的会话，起会话的线程可能已经退出
           * （短命线程，见 struct 里 holder_tid 的说明），而真正读数据的线程还没进
           * 第一次 read()（user_tid 还是 -1）—— 这段时间里线程证据看起来是空的，
           * 会话却是活的。宽限期把这一段单独掐掉：宁可按'有人在用'回 -EBUSY，也不去
           * 拆一条刚开始的会话（-EBUSY 对上层是安全值：audio_in_start() 会重试一次，
           * robot_ui 的播报路径报错后当次静默走完）。 */

          age_ms = (uint32_t)TICK2MSEC(clock_systime_ticks() - priv->holder_since);
          if (age_ms < SF32LB52_AUDIO_SWITCH_GRACE_MS)
            {
              syslog(LOG_WARNING,
                     "AUDIO: 方向冲突：%s 通路是 %u ms 前刚起的（还不到宽限期 %d ms），"
                     "按'占用者可能只是还没进第一次读写'处理（持有线程=%d 最近读写=%d "
                     "组=%d 都查不到，但会话还太新）→ -EBUSY\n",
                     dir ? "播放" : "录音", (unsigned)age_ms,
                     (int)SF32LB52_AUDIO_SWITCH_GRACE_MS,
                     (int)priv->holder_tid, (int)priv->user_tid,
                     (int)priv->holder_pid);
              return -EBUSY;
            }

          syslog(LOG_WARNING,
                 "AUDIO: 方向冲突且占用者已无人生还（要起 %s，%s 通路残留："
                 "起会话的线程 %d / 最近读写的线程 %d 都查不到 TCB，"
                 "起会话的任务组 %d 也查不到 TCB（= 整个 app 没了），"
                 "并且这条会话已经起了 %u ms（>= 宽限期 %d ms）），"
                 "判定为残留会话，强制做一次干净 stop 再切到 %s\n",
                 want ? "播放" : "录音",
                 dir ? "播放" : "录音",
                 (int)priv->holder_tid, (int)priv->user_tid,
                 (int)priv->holder_pid,
                 (unsigned)age_ms,
                 (int)SF32LB52_AUDIO_SWITCH_GRACE_MS,
                 want ? "播放" : "录音");
        }
      else
        {
          /* 同方向：残留 / 重复 START，收干净再起 */

          syslog(LOG_WARNING,
                 "AUDIO: 检测到残留运行状态（running=1 dir=%s 与本次要起的 %s 一致，"
                 "dac=%d pa=%d adc=%d），先做一次干净 stop 再启动\n",
                 dir ? "播放" : "录音",
                 want ? "播放" : "录音",
                 (int)priv->dac_path_on, (int)priv->pa_on,
                 (int)priv->adc_path_on);
        }

      /* 两条路都走到这里：把旧会话收干净（异方向残留是刚判定完的，同方向是
       * 重复 START），下面 hw_start 才从零开始把想要的那条通路起起来。 */

      sf32lb52_audio_hw_stop(priv);
    }

  return sf32lb52_audio_hw_start(priv, want);
}

/****************************************************************************
 * Name: sf32lb52_audio_stop
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int sf32lb52_audio_stop(FAR struct audio_lowerhalf_s *dev)
{
  return sf32lb52_audio_hw_stop((FAR struct sf32lb52_audio_s *)dev);
}
#endif

/****************************************************************************
 * Name: sf32lb52_audio_enqueuebuffer
 *
 * Description:
 *   上层投递一个 buffer：
 *   - 播放：数据经 AUDPRC TX0 DMA 送向 DAC
 *   - 录音：AUDPRC RX0 DMA 把 ADC 数据收进 buffer
 *   完成由 DMA 中断回调（HAL_AUDPRC_Tx/RxCpltCallback）通知。
 *
 ****************************************************************************/

static int sf32lb52_audio_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                        FAR struct ap_buffer_s *apb)
{
  FAR struct sf32lb52_audio_s *priv =
    (FAR struct sf32lb52_audio_s *)dev;
  HAL_StatusTypeDef res;

  DEBUGASSERT(priv && apb);

  /* 队列模式的方向：正在跑就按**已提交的**会话方向（别人一次 CONFIGURE 不能
   * 把在跑的会话的 buffer 改道），没在跑就按刚 CONFIGURE 的意图。
   * 这是原来的等价语义 —— 原来 priv->playback 会被 CONFIGURE 直接改掉，
   * 也就是"没 START 也按 CONFIGURE 的方向"，现在只有 START 过才算数。
   * 本板两个 app（hello_app / robot_ui）走的都是 write/read 直通，没有队列
   * 模式的使用者，这里只是把语义摆正、不留一个会被方向标签污染的口子。 */

  if (priv->running ? priv->playback : priv->pending_playback)
    {
      priv->tx_apb = apb;
      res = HAL_AUDPRC_Transmit_DMA(&priv->aprc, apb->samp, apb->nbytes,
                                    SF32LB52_AUDIO_PRC_TX_CH);
    }
  else
    {
      priv->rx_apb = apb;
      res = HAL_AUDPRC_Receive_DMA(&priv->aprc, apb->samp, apb->nbytes,
                                   SF32LB52_AUDIO_PRC_RX_CH);
    }

  if (res != HAL_OK)
    {
      auderr("ERROR: DMA enqueue failed: %d\n", res);
      priv->tx_apb = NULL;
      priv->rx_apb = NULL;
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: sf32lb52_audio_cancelbuffer
 *
 ****************************************************************************/

static int sf32lb52_audio_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                                       FAR struct ap_buffer_s *apb)
{
  FAR struct sf32lb52_audio_s *priv =
    (FAR struct sf32lb52_audio_s *)dev;

  if (priv->tx_apb == apb)
    {
      /* 守卫的规矩和 hw_stop / hw_shutdown 里那两对一样：READY 说明这条通道
       * 已经收过尾，再 abort 一次写的是可能已被别人分配走的物理通道。
       * 这条路只有在**队列模式**（enqueuebuffer 真的起过这条通道）时才走得
       * 进来，正常情况下 tx_apb/rx_apb 都是 NULL、一个 abort 都不会发。 */

      if (priv->aprc.hdma[SF32LB52_AUDIO_PRC_TX_CH] == NULL ||
          priv->aprc.hdma[SF32LB52_AUDIO_PRC_TX_CH]->State != HAL_DMA_STATE_READY)
        {
          HAL_AUDPRC_DMAStop(&priv->aprc, SF32LB52_AUDIO_PRC_TX_CH);
        }

      priv->tx_apb = NULL;
    }

  if (priv->rx_apb == apb)
    {
      if (priv->aprc.hdma[SF32LB52_AUDIO_PRC_RX_CH] == NULL ||
          priv->aprc.hdma[SF32LB52_AUDIO_PRC_RX_CH]->State != HAL_DMA_STATE_READY)
        {
          HAL_AUDPRC_DMAStop(&priv->aprc, SF32LB52_AUDIO_PRC_RX_CH);
        }

      priv->rx_apb = NULL;
    }

  return OK;
}

/****************************************************************************
 * Name: sf32lb52_audio_ioctl
 *
 ****************************************************************************/

static int sf32lb52_audio_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                                unsigned long arg)
{
  int ret = OK;

  switch (cmd)
    {
      case AUDIOIOC_HWRESET:
        audinfo("AUDIOIOC_HWRESET\n");
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: sf32lb52_audio_write
 *
 * Description:
 *   直通写入：把应用数据经 codec 自带的 DAC DMA（HAL_AUDCODEC_Transmit_DMA）
 *   送出并等待播放完成。
 *   （NuttX 标准 buffer 队列流程之外的简单同步播放接口）
 *
 ****************************************************************************/

static ssize_t sf32lb52_audio_write(FAR struct audio_lowerhalf_s *dev,
                                    FAR const char *buffer, size_t buflen)
{
  FAR struct sf32lb52_audio_s *priv =
    (FAR struct sf32lb52_audio_s *)dev;
  HAL_StatusTypeDef res;
  int ret;

  if (buffer == NULL || buflen == 0 || !priv->running)
    {
      return 0;
    }

  /* 记下"这次会话最近一次真的有人在写"（判残留时的一条证据，见
   * sf32lb52_audio_holder_alive）。放在 running 早退之后：设备没在跑就不算
   * 有人用它。 */

  priv->user_tid = nxsched_gettid();

  priv->wr_busy = true;

  /* 数据体检：样本是否为空/异常（全 0 会让 DAC 只输出直流，只有开关爆音） */
  {
    FAR const int16_t *p = (FAR const int16_t *)buffer;
    int i;
    int mx = 0;

    for (i = 0; i < 256; i++)
      {
        int v = p[i];

        if (v < 0)
          {
            v = -v;
          }

        if (v > mx)
          {
            mx = v;
          }
      }

  }


  clock_t t0 = clock_systime_ticks();

  (void)t0;

  /* ★ 起传输前必须把状态摆正。
   *
   * vendor HAL 里 HAL_AUDCODEC_DMAStop() 那句 "State = READY" 是**注释掉的**，
   * 而 HAL_AUDCODEC_Transmit_DMA() 只要看到 State 还带 HAL_AUDCODEC_STATE_BUSY_TX
   * 就直接 return HAL_BUSY。于是只要上一次传输没走到本函数末尾那两行收尾
   * （上一次会话被 STOP/close 打断、上一次 write 起不来、掉电前的残留……），
   * State 就永远卡在 BUSY_TX：之后**每一次** write 都在这里失败、返回 0，
   * 上层看到的就是"界面显示正在播放，但一个字节都没播出去"。
   *
   * 兜法：残留就先把 DMA 停掉，再把状态强制摆成 READY。这条警告正常播放时
   * 一次都不该出现 —— 出现了就说明上一条路径没收好尾，串口日志能直接看到。 */

  if ((priv->codec.State[HAL_AUDCODEC_DAC_CH0] & HAL_AUDCODEC_STATE_BUSY_TX) != 0)
    {
      syslog(LOG_WARNING,
             "AUDIO: DAC 状态残留 BUSY_TX(0x%x)，先停 DMA 再重起\n",
             priv->codec.State[HAL_AUDCODEC_DAC_CH0]);
      HAL_AUDCODEC_DMAStop(&priv->codec, HAL_AUDCODEC_DAC_CH0);
    }

  priv->codec.State[HAL_AUDCODEC_DAC_CH0] = HAL_AUDCODEC_STATE_READY;

  /* DMA 句柄自己也有状态机：HAL_AUDCODEC_Transmit_DMA 里
   * HAL_DMA_Start_IT() 的返回值是被丢掉的，句柄如果不是 READY 它直接返回
   * HAL_BUSY、DMA 根本没起来，而 codec 那边的 BUSY_TX 已经置上了 —— 现象
   * 就是 write() 干等 600ms 超时、一个字节都没播。这里一并兜住并留证。 */

  if (priv->codec.hdma[HAL_AUDCODEC_DAC_CH0] != NULL &&
      priv->codec.hdma[HAL_AUDCODEC_DAC_CH0]->State != HAL_DMA_STATE_READY)
    {
      syslog(LOG_WARNING,
             "AUDIO: DAC 的 DMA 句柄状态异常(0x%x)，强制复位成 READY\n",
             priv->codec.hdma[HAL_AUDCODEC_DAC_CH0]->State);
      priv->codec.hdma[HAL_AUDCODEC_DAC_CH0]->State = HAL_DMA_STATE_READY;
    }

  res = HAL_AUDCODEC_Transmit_DMA(&priv->codec, (FAR uint8_t *)buffer, buflen,
                                  HAL_AUDCODEC_DAC_CH0);
  if (res != HAL_OK)
    {
      /* 起不来也别把上一次的 DMA 留在循环模式里：DAC 的 DMA 是
       * DMA_CIRCULAR，残留的传输会无限重播最后一段（"昂昂昂昂"）。
       * 同时把 res / Lock / State 打出来 —— 这次现场就是靠它定位的。 */

      syslog(LOG_ERR,
             "AUDIO: Transmit_DMA 失败 res=%d Lock=%d State=0x%x dmastate=0x%x "
             "buflen=%u running=%d\n",
             (int)res, (int)priv->codec.Lock,
             priv->codec.State[HAL_AUDCODEC_DAC_CH0],
             priv->codec.hdma[HAL_AUDCODEC_DAC_CH0] != NULL ?
               (int)priv->codec.hdma[HAL_AUDCODEC_DAC_CH0]->State : -1,
             (unsigned)buflen, (int)priv->running);

      HAL_AUDCODEC_DMAStop(&priv->codec, HAL_AUDCODEC_DAC_CH0);
      priv->codec.State[HAL_AUDCODEC_DAC_CH0] = HAL_AUDCODEC_STATE_READY;
      priv->wr_busy = false;
      return 0;
    }

  {
    /* 按实际采样率算等待上限（ms），再加 500ms 余量 */
    uint32_t wait_ms = (uint32_t)(((uint32_t)(buflen / 2) * 1000U) /
                       (uint32_t)(priv->samplerate ? priv->samplerate : 16000)) + 500U;

    ret = nxsem_tickwait_uninterruptible(&priv->wr_sem, MSEC2TICK(wait_ms));
  }

  priv->wr_busy = false;

  /* 停止 DMA 传输（单次播放完成）。
   *
   * 注意这里的两下收尾大多已经做过了：DMA 完成中断里的
   * HAL_AUDCODEC_TxCpltCallback 已经把 DAC 的 DMA 使能和 DMA 通道关掉了
   * （sf32lb52_audio_tx_freeze）—— 就是为了不让循环 DMA 在"中断返回 → 本线程
   * 被唤醒"这段窗口里从块首重播。这里再走一遍是给下面几条路兜底：
   * 中断没来（等超时）、Transmit 起不来、上层提前把会话停掉。
   * HAL_AUDCODEC_DMAStop() 内部是 HAL_DMA_Abort + 清 DAC_CHx_CFG 的 DMA_EN，
   * 幂等，重复调没问题。 */

  HAL_AUDCODEC_DMAStop(&priv->codec, HAL_AUDCODEC_DAC_CH0);
  priv->codec.State[HAL_AUDCODEC_DAC_CH0] = HAL_AUDCODEC_STATE_READY;

  if (ret < 0)
    {
      /* 等超时：DMA 没在预期时间内报完成，本块算丢。这条**正常播放时不该
       * 出现**（每块 100ms，等的是 100ms+500ms 余量），出现就说明播放通路
       * 出问题了，串口日志要留着看。 */

      syslog(LOG_ERR, "AUDIO: 等播放完成超时（buflen=%u），本块丢弃\n",
             (unsigned)buflen);
      return 0;
    }

  return buflen;
}

/****************************************************************************
 * Name: sf32lb52_audio_read
 *
 * Description:
 *   直通读取：启动 AUDPRC RX0 DMA 采集麦克风数据并等待完成。
 *
 ****************************************************************************/

static ssize_t sf32lb52_audio_read(FAR struct audio_lowerhalf_s *dev,
                                   FAR char *buffer, size_t buflen)
{
  FAR struct sf32lb52_audio_s *priv =
    (FAR struct sf32lb52_audio_s *)dev;
  FAR DMA_HandleTypeDef *hdma_rx;
  HAL_StatusTypeDef res;
  uint32_t gen;
  uint32_t irq_before;          /* 本次 read 武装 DMA 之前的中断计数 */
  uint32_t irq_at_stop;         /* 收尾停 DMA **之前**的中断计数 */
  uint32_t aprc_state_at_stop;  /* 停 DMA 之前的 aprc.State[RX]（停完就是 0x1 了） */
  uint32_t dma_state_at_stop;   /* 停 DMA 之前的 hdma[RX].State（同上） */
  uint32_t cndtr_at_stop;       /* 停 DMA 之前的剩余传输数（判决实验的核心） */
  uint32_t ccr_at_stop;         /* 停 DMA 之前的 CCR：EN/TCIE/TEIE/CIRC 还在不在 */
  uint32_t isr_at_stop;         /* 停 DMA 之前该 DMA 的 ISR（挂着哪些标志） */
  uint32_t dma_err_code;        /* 停 DMA 之前的 hdma[RX].ErrorCode */
  bool     rx_err;              /* 本次等待是被 DMA 传输错误(TE)结束的 */
  bool     quiet;               /* 本次 TE 日志被限频窗口吞掉（只计数不打日志） */
  clock_t  now;                 /* 本次收尾的时刻（日志时间戳 + 限频窗口比较） */
  int ret;

  priv->rx_read_count++;

  if (buffer == NULL || buflen == 0 || !priv->running)
    {
      syslog(LOG_WARNING, "AUDIO: read 时设备没在跑（len=%zu running=%d）\n",
             buflen, (int)priv->running);
      return 0;
    }

  /* 记下"这次会话最近一次真的有人在读"（判残留时的一条证据，见
   * sf32lb52_audio_holder_alive）。起会话的线程可能早就退了、读数据的是另一个
   * 线程 —— 这一行就是为了让那种会话不被当成残留拆掉。 */

  priv->user_tid = nxsched_gettid();

  /* 先把上一次 stop 可能留下的计数清掉，再起 DMA。
   * 配合 hw_stop() 里"无条件 post"的唤醒：不清的话，上一次 stop 的 post
   * 会让本次 read 一进等待就立刻返回，把没采满的缓冲当数据交上去。 */

  nxsem_reset(&priv->rx_sem, 0);

  priv->rx_busy    = true;
  priv->rx_aborted = false;

  /* TE 那面旗也一起清：它只表示"**本帧**等待期间 DMA 报了传输错误"。
   * 清在这里而不是进了等待才清，是因为错误回调只在 rx_busy 为真时立旗
   * （见 HAL_AUDPRC_ErrorCallback），而 rx_busy 就是刚刚置真的这一行 ——
   * 两边对"本帧"的定义由此对齐。 */

  priv->rx_dma_err = false;

  /* 本次 read 属于哪个会话。选在 nxsem_reset() 之后取：那次 reset 会把上一次
   * stop 留下的计数清掉，而接下来任何一个 hw_stop()/hw_shutdown()/新会话的
   * hw_start() 都会让 gen 变号 —— 只要变了，就说明这次等待已经没有意义了。 */

  gen = priv->session_gen;

  hdma_rx = priv->aprc.hdma[SF32LB52_AUDIO_PRC_RX_CH];

  /* 起传输前先把 DMA 句柄的状态摆正 —— 和 write() 里那层兜底对称（write 侧
   * 对应的是 codec 的 DAC 句柄，这边是 AUDPRC 的 RX 句柄）。
   *
   * 为什么必须兜：厂商的 HAL_DMA_Start_IT() 只在 hdma->State ==
   * HAL_DMA_STATE_READY 时才真正发传输，否则直接返回 HAL_BUSY 什么都不做
   * （bf0_hal_dma.c:829 判 READY，否则 883 行 status = HAL_BUSY 后 goto __EXIT）；
   * 而 HAL_AUDPRC_Receive_DMA() 把它的返回值**丢掉**，自己照样 return HAL_OK
   * （bf0_hal_audprc.c:818 调用处没接返回值，828 行无条件 return HAL_OK）。
   * 于是句柄只要残留非 READY，read() 就会"以为起来了、其实没起"，一路干等 5 秒
   * 以 -110 收场 —— 正是这层兜底要堵的洞。
   *
   * 兜法分两步，缺一不可：
   *   1) HAL_AUDPRC_DMAStop() 内部是 HAL_DMA_Abort()，它把 hdma->State 置 READY
   *      （bf0_hal_dma.c:923）；真正把通道收干净的是这一步；
   *   2) 再手动把两个 State 都置 READY。因为同一个 DMAStop 里厂商把
   *      `haprc->State = HAL_AUDPRC_STATE_READY` 那行**注释掉了**
   *      （bf0_hal_audprc.c:850），只 abort 不看 haprc->State 的话，
   *      Receive_DMA 开头那句 `State & HAL_AUDPRC_STATE_BUSY_RX` 判断
   *      （bf0_hal_audprc.c:774）照样会回 HAL_BUSY —— 这是本文件里已经踩过的
   *      同族坑（见下面 DMAStop 之后的注释）。
   *
   * 这条警告正常录音时一次都不该出现；出现了就说明上一条路径没收好尾。 */

  if (hdma_rx != NULL && hdma_rx->State != HAL_DMA_STATE_READY)
    {
      syslog(LOG_WARNING,
             "AUDIO: RX 的 DMA 句柄状态残留(0x%x)，先停 DMA 再强制复位成 READY\n",
             (unsigned)hdma_rx->State);

      HAL_AUDPRC_DMAStop(&priv->aprc, SF32LB52_AUDIO_PRC_RX_CH);

      hdma_rx->State = HAL_DMA_STATE_READY;
      priv->aprc.State[SF32LB52_AUDIO_PRC_RX_CH] = HAL_AUDPRC_STATE_READY;
    }

  /* 重新武装这一步是"每次 read 都停一次、再起一次"，跟厂商参考驱动的做法不同，
   * 也是那个约 10% 超时最可能的来源。事实与推断分开列（本次只改返回值语义，
   * 这里的时序一行不动 —— 动它得有真机验证）：
   *
   * 事实：
   *   1) RX 通道是 DMA_CIRCULAR（bf0_hal_audprc.c:685 与 :804），而完成中断里
   *      只 post 信号量、不停任何东西。从"中断来了"到本线程真去 DMAStop 这段
   *      （唤醒 + 调度延迟）里，DMA 还在同一块 record_buf 上继续跑，能再写满
   *      一整圈（20ms 一帧正好一圈）→ 交上去那一帧的开头会被新采样覆盖。
   *   2) HAL_AUDPRC_DMAStop() 里头是 HAL_DMA_Abort()（bf0_hal_dma.c:898）：
   *      关 TC/HT/TE 中断、关通道、清该通道全部标志；本构建开了
   *      DMA_SUPPORT_DYN_CHANNEL_ALLOC（bf0_hal_dma.h:27），所以还走
   *      DMA_FreeChannel() —— 那里会 HAL_NVIC_DisableIRQ() 并释放通道池；
   *      随后 ADCPATH_DISABLE（bf0_hal_audprc.c:846）关掉 ADC 数据通路。
   *   3) 下一次 read 的 Receive_DMA 把这些逐条反过来：ADCPATH_ENABLE →
   *      DMA_AllocChannel（先 NVIC 使能、必要时换通道）→ DMA_Start（清标志、
   *      写 CNDTR/CPAR/CM0AR、开 IT、开通道）→ 设 RX_CH0_CFG 的 DMA 使能位
   *      （这一位从来只置不清）→ __HAL_AUDPRC_ENABLE。
   *   4) HAL_DMA_IRQHandler() 只在"TC 标志还在、且 CCR 的 TC 中断使能还在"时
   *      回调（bf0_hal_dma.c:1133）→ 停/起窗口里落下的完成事件是静默丢掉的，
   *      连计数都没有（本文件那 5 个计数器覆盖不到它）。
   *
   * 推断（未验证，所以没动）：
   *   - "armed 了（not_armed=0）却 5 秒没有中断"，最可能就出在第 2、3 步之间：
   *     标志刚被清、中断刚被开，而通道与请求位正在"上一次刚结束、下一次刚起步"
   *     这一刻被反复改。落在窗口里的那一次完成通知，要么被丢（不回调），要么在
   *     新通道还没配置完时被提前服务 —— 后者会让这次 read 拿着**没被写过的**数据
   *     当成功返回（更隐蔽的一种坏法）。
   *   - 厂商参考驱动（SiFli-SDK 的 drv_audprc.c）是**整场会话只武装一次**循环
   *     DMA（bf0_audio_start 里 Receive_DMA，之后只靠 RxCplt/RxHalfCplt 回调交
   *     数据，到 stream stop 才 DMAStop），从不按帧停/起。本驱动每 20ms 就
   *     stop + re-arm 一次（50 次/秒），把上面那两个窗口的频率放大了几个数量级，
   *     和"约 10% 的失败"是吻合的形态。
   *
   * 根治方向（要真机验证，本次不做）：照参考驱动改成本文件内的双缓冲、一次武装、
   * 只用 RxCplt 交数据；或至少把 ADCPATH 与 NVIC 的关/开从每帧路径里拿掉。 */

  /* 武装之前先把中断计数记下来。超时分支要靠它回答一个只能在这一刻回答的问题：
   * "接下来这 5 秒里，完成中断到底来过没有"（差值 = 0 就是这个通道已经死了）。 */

  irq_before = priv->rx_irq_count;

  res = HAL_AUDPRC_Receive_DMA(&priv->aprc, (FAR uint8_t *)buffer, buflen,
                               SF32LB52_AUDIO_PRC_RX_CH);

  /* 整体使能在 DMA 启动后（参考 SDK audprc start 顺序） */

  __HAL_AUDPRC_ENABLE(&priv->aprc);

  if (res != HAL_OK)
    {
      priv->rx_busy = false;
      priv->rx_dma_busy_fail++;

      /* 起不来 DMA 的现场：HAL 手上有 hdma_rx 的通道状态和 aprc 的通道状态，
       * 两个都打出来才能分清是"AUDPRC 这一侧还认为 RX 在忙"还是
       * "DMA 通道自己没释放"。hdma_rx 可能还没分配（hw_init 失败过），
       * 所以打 State 前先判空，不能为了日志把驱动打崩。 */

      syslog(LOG_ERR,
             "AUDIO: read 起 DMA 失败 res=%d aprc.State[RX]=0x%x "
             "hdma[RX].State=0x%x running=%d\n",
             (int)res,
             (unsigned)priv->aprc.State[SF32LB52_AUDIO_PRC_RX_CH],
             hdma_rx != NULL ? (unsigned)hdma_rx->State : 0u,
             (int)priv->running);
      return 0;
    }

  /* 返回 HAL_OK 也不能就信 —— 上面说过，Receive_DMA 把 HAL_DMA_Start_IT() 的
   * 返回值丢了，自己无条件 return HAL_OK（bf0_hal_audprc.c:818 / 828）。
   * 所以这里再验一次"到底起没起"：Start_IT 成功时会把句柄置成
   * HAL_DMA_STATE_BUSY（bf0_hal_dma.c:841），没进 BUSY 就说明它走了 else 分支
   * 直接 return HAL_BUSY，DMA 一个字节都不会传。
   *
   * 这种情况**不能**接着往下等：等下去必然是 5 秒后那个 -110，而且把时间白白
   * 花掉。直接返回 0（上层拿到 0 就走正常收尾、把麦克风还回去），并把句柄没
   * 到位这件事记进 rx_dma_not_armed —— 它是"OK 但没起来"这一类坏法的唯一凭据
   * （busy_fail 只统计 res 非 HAL_OK，压根覆盖不到这里）。
   *
   * 顺手把 aprc 的 RX 通道状态掰回 READY：Receive_DMA 在 Start_IT **之前**就
   * 先把 State 置成了 BUSY_RX（bf0_hal_audprc.c:785），不回退的话本次会话后面
   * 每一次 read 都会被开头那句 BUSY_RX 判断直接挡回去（774 行）。这里只做一个
   * 字段写，不碰寄存器，也就没有"句柄可能还没分配过通道"的风险。 */

  if (hdma_rx != NULL && hdma_rx->State != HAL_DMA_STATE_BUSY)
    {
      priv->rx_busy = false;
      priv->rx_dma_not_armed++;
      priv->aprc.State[SF32LB52_AUDIO_PRC_RX_CH] = HAL_AUDPRC_STATE_READY;

      syslog(LOG_ERR,
             "AUDIO: read 的 DMA 其实没起来（hdma[RX].State=0x%x "
             "aprc.State[RX]=0x%x running=%d not_armed=%u），不等了直接返回 0\n",
             (unsigned)hdma_rx->State,
             (unsigned)priv->aprc.State[SF32LB52_AUDIO_PRC_RX_CH],
             (int)priv->running, (unsigned)priv->rx_dma_not_armed);
      return 0;
    }

  /* 分片等待（每片 100ms，最多 5 秒），**不能一次等满 5 秒**：
   * 上层的 audio_record_stop() 是"先停设备再 join 录音线程"，而 join 是在
   * LVGL 线程里调的。实测出现过"停设备那一下没能唤醒这个等待、5 秒超时也没
   * 回来"（提交按钮一直绿着、界面整块卡死），录音线程于是永远出不来。
   * 分片之后：每 100ms 主动看一眼 rx_aborted / running / 会话代号，只要
   * stop（或 close）走过就把这一片收掉返回 0，最坏 100ms 就能把录音线程
   * 放出来，不再靠那一次 post 或者那一次超时。
   *
   * 第三个判据（会话代号）是给"post 没生效 / 状态没来得及更新"兜底的：
   * 它比对的是**本次 read 的时代还在不在**，不依赖任何人把标志写好或者记得
   * 唤醒这个任务 —— 真机上正是"唤醒没生效"导致录音线程收不了尾（见那次的
   * AUDIOIOC_STOP failed: 25 与 300ms join 超时）。
   *
   * 第五个判据 rx_dma_err 是本次新增的：DMA 传输错误(TE)时错误回调会 post
   * 一次信号量把这一片立刻叫醒（不等这 100ms），这里再判一次旗子是给
   * "post 那一下又被别的 post 混掉"兜底 —— 两个机制合起来，TE 之后本次 read
   * 最坏 100ms 就收尾，而不是空转满 5 秒。 */

  {
    int slice;

    for (slice = 0; slice < 50; slice++)
      {
        ret = nxsem_tickwait_uninterruptible(&priv->rx_sem, MSEC2TICK(100));

        if (ret == OK || priv->rx_aborted || !priv->running ||
            gen != priv->session_gen || priv->rx_dma_err)
          {
            break;
          }
      }
  }

  priv->rx_busy = false;

  /* 被 stop() / close / 新会话打断（或设备已经不在跑了）：按 EOF 返回（不能报
   * buflen，数据并没采满；上层拿到 0 就会跳出录音循环、把麦克风还回去）。
   *
   * 这三个判据合起来就是"**这一代会话真的结束了**"。它必须和下面那个**分片等待
   * 超时**（返回 -ETIMEDOUT）用不同的返回值分开：
   *   0           → 会话结束：有人停了它 / 设备没在跑 / 换了新会话；
   *   -ETIMEDOUT  → 会话还活着，只是这一次读没等到数据（上层跳过这一帧接着读）。
   * 真机上这两种被混在一起过：大约 10% 的读会超时（现场 irq=448 read=479
   * timeout=47），而上层把 0 一律当"会话死了"，于是每 5 秒拆一次设备重录，
   * 一断一续，永远攒不齐一句话 —— VAD / ASR 全都不动。 */

  if (priv->rx_aborted || !priv->running || gen != priv->session_gen)
    {
      priv->rx_aborted = false;
      HAL_AUDPRC_DMAStop(&priv->aprc, SF32LB52_AUDIO_PRC_RX_CH);
      priv->aprc.State[SF32LB52_AUDIO_PRC_RX_CH] = HAL_AUDPRC_STATE_READY;
      return 0;
    }

  /* 超时现场必须在**停 DMA 之前**抓。
   *
   * 下面这几行收尾（DMAStop 内部 HAL_DMA_Abort 会把 hdma->State 置 READY、
   * 紧接着又强制 aprc.State=READY）写完，两个状态就恒等于 0x1/0x1 了；
   * 原来的日志正是在那之后才采样，所以"aprc.State[RX]=0x1 hdma[RX].State=0x1"
   * 这两个字段从来只是把本函数自己刚写进去的值读回来，对定位毫无价值 ——
   * 这次冻结现场就被它误导过。改在这里抓，才能看出"等待期间通道是不是一直
   * 武装着（BUSY）、有没有人把它 abort 掉"。
   *
   * 注意顺序：本条 return 0 的路径（会话结束）说明不了任何事，所以快照放在
   * 它后面；而它自己也有一处 DMAStop，那边不需要现场。
   *
   * 本次再补三个寄存器（同样必须在 abort 之前读，abort 会把它们清掉）：
   *   CNDTR —— **判决实验的核心**。20ms@16k 单声道 16bit 一帧 = 640 字节
   *            = 160 个 32bit 字，所以
   *              CNDTR == 160（一字节没搬）→ DMA 的请求线从头到尾没被拉高，
   *                                          坏在数据源头（ADC 模拟通路 /
   *                                          AUDPRC 这一侧），和 DMA 通道无关；
   *              CNDTR 停在 0..159        → 搬了一部分之后断流：数据真的流过，
   *                                          是中途停的（请求线半路没了）。
   *   CCR   —— 停机这一刻通道还"武装着"没有：EN（通道使能）、TCIE/TEIE（完成/
   *            错误中断使能）、CIRC（循环位）。本驱动停之前它应当是
   *            EN|TCIE|HTIE|TEIE|CIRC —— 少了哪一位就说明有人动过它。
   *   ISR   —— 该 DMA 当时挂着哪些通道标志（TC/HT/TE）。和 irq/half/dma_err
   *            三个计数配对：ISR 上还有 TC 而 irq 没涨 = "中断产生了但没人
   *            处理"（NVIC 被关/优先级被压），ISR 干干净净 = "中断压根没产生"。
   *   ErrorCode —— DMA 通道自己的错误码（HAL_DMA_ERROR_TE = 0x1）。 */

  irq_at_stop        = priv->rx_irq_count;
  aprc_state_at_stop = (uint32_t)priv->aprc.State[SF32LB52_AUDIO_PRC_RX_CH];
  dma_state_at_stop  = (hdma_rx != NULL) ? (uint32_t)hdma_rx->State : 0u;
  cndtr_at_stop      = (hdma_rx != NULL && hdma_rx->Instance != NULL) ?
                       (uint32_t)hdma_rx->Instance->CNDTR : 0u;
  ccr_at_stop        = (hdma_rx != NULL && hdma_rx->Instance != NULL) ?
                       (uint32_t)hdma_rx->Instance->CCR : 0u;
  isr_at_stop        = (hdma_rx != NULL && hdma_rx->DmaBaseAddress != NULL) ?
                       (uint32_t)hdma_rx->DmaBaseAddress->ISR : 0u;
  dma_err_code       = (hdma_rx != NULL) ? (uint32_t)hdma_rx->ErrorCode : 0u;

  /* 本次等待是不是被 DMA 传输错误结束的。旗子在这里读走并清掉：TE 的错误
   * 回调只是在中断里立旗 + post 信号量（不在中断里打日志，见错误回调的注释），
   * 收尾和日志都留在这一处任务上下文里做，两边对"本帧"的定义就唯一了。 */

  rx_err = priv->rx_dma_err;
  priv->rx_dma_err = false;

  /* 停止循环 DMA 传输（单次采集完成） */

  HAL_AUDPRC_DMAStop(&priv->aprc, SF32LB52_AUDIO_PRC_RX_CH);

  /* HAL 的 HAL_AUDPRC_DMAStop() 不复位 State（那行被厂商注释掉了），
   * 而 HAL_AUDPRC_Receive_DMA() 开头会判 State & HAL_AUDPRC_STATE_BUSY_RX
   * 并直接返回 HAL_BUSY。不复位的话**同一次会话里第二次 read() 一定失败**
   * （表现为"读了 1 秒就返回"，实测 hw_test audio 2 只拿到 32000/64000）。
   */

  priv->aprc.State[SF32LB52_AUDIO_PRC_RX_CH] = HAL_AUDPRC_STATE_READY;

  /* 两个入口：分片等待超时（ret < 0），或者等待被 DMA 传输错误结束（rx_err）。
   * **rx_err 必须和超时走同一条出口**：TE 时错误回调 post 过一次信号量把
   * 循环叫醒，于是 ret 是 OK —— 只看 ret 的话会掉进下面的 `return buflen`，
   * 把一块**没被写过的**缓冲当"采满了"交给上层（比报错隐蔽得多的一种坏法）。 */

  if (ret < 0 || rx_err)
    {
      /* 50 片 × 100ms 都等完、一片完成通知都没来：**这一片** DMA 就是没给数据。
       *
       * 超时走到这里一定是"整整 5 秒没等到"：被 stop / 设备停了 / 会话换代这三种
       * 都会让上面的循环提前 break，并且命中再上面那条 return 0（它们的返回值
       * 必须是 0，语义是 EOF）。所以超时这一路只剩"没等到"这一种。
       * （rx_err 那一路不必等满 5 秒，它是被 TE 中断提前结束的。）
       *
       * 返回 -ETIMEDOUT（nuttx/include/errno.h:170，值 110 —— 现场日志里
       * ret=-110 的同一个错误号）：它表示"**这次读没拿到数据**"，不是"会话结束"。
       * 上层（app/hello_app/ai_audio.c 的录音线程）拿到它必须跳过这一帧接着读，
       * 只有连续超时到上限才当会话死了处理。
       *
       * 为什么非要让上层分得出来：真机上大约 10% 的读会走到这里，而上层原来把
       * 0 一律当"会话死了"，于是每 5 秒拆一次设备重录 —— 一次抖动就毁掉整场录音。
       *
       * 判读方式（现场那次 irq=448 read=479 timeout=47，约 10% 超时）：
       *   irq_delta=0 且 CNDTR=160 → 这 5 秒里中断一次都没来、一字节都没搬：
       *                              请求线从未被拉高，坏在 ADC/AUDPRC 通路侧；
       *   irq_delta=0 且 CNDTR<160 → 搬了一半之后断流（曾经在流、中途停了）；
       *   half 在涨而 irq 不涨     → 数据其实是通的，只有 TC 那一路丢了；
       *   err_te=1                 → 这一次是 DMA 传输错误(TE)结束的
       *                              （HAL 已把通道 Free + State=READY）；
       *   irq 在涨                 → 中断是来的，问题在等待/唤醒这一段。 */

      now = clock_systime_ticks();

      if (rx_err)
        {
          /* TE 这条日志要限频：它让 read **立刻**返回（不像超时要等 5 秒），
           * 持续性的 TE 就是"上层每次重试一条"。规矩和自愈日志一样
           * （见 sf32lb52_audio_recover_log）：窗口内只累计、不打日志，
           * 出窗口那一条把被吞掉的条数一起报出来。 */
          quiet = (int32_t)(now - priv->rx_dma_err_log_next) < 0;

          if (quiet)
            {
              priv->rx_dma_err_silenced++;
            }
          else
            {
              priv->rx_dma_err_log_next =
                now + MSEC2TICK(SF32LB52_AUDIO_RECOVER_LOG_MS);
            }
        }
      else
        {
          quiet = false;
          priv->rx_timeout_count++;
        }

      if (!quiet)
        {
          syslog(LOG_WARNING,
                 "AUDIO: read 等 DMA 失败（ret=%d err_te=%d）"
                 " irq=%u half=%u read=%u timeout=%u busy_fail=%u not_armed=%u"
                 " dma_err=%u dma_err_silenced=%u recov=%u irq_delta=%u"
                 " CNDTR=%u CCR=0x%x ISR=0x%x dma_err_code=0x%x"
                 " aprc.State[RX]=0x%x hdma[RX].State=0x%x"
                 " t=%u running=%d playback=%d\n",
                 ret, (int)rx_err,
                 (unsigned)priv->rx_irq_count,
                 (unsigned)priv->rx_half_irq_count,
                 (unsigned)priv->rx_read_count,
                 (unsigned)priv->rx_timeout_count,
                 (unsigned)priv->rx_dma_busy_fail,
                 (unsigned)priv->rx_dma_not_armed,
                 (unsigned)priv->rx_dma_err_count,
                 (unsigned)priv->rx_dma_err_silenced,
                 (unsigned)priv->rx_recover_count,
                 (unsigned)(irq_at_stop - irq_before),
                 (unsigned)cndtr_at_stop,
                 (unsigned)ccr_at_stop,
                 (unsigned)isr_at_stop,
                 (unsigned)dma_err_code,
                 (unsigned)aprc_state_at_stop,
                 (unsigned)dma_state_at_stop,
                 (unsigned)TICK2MSEC(now),
                 (int)priv->running, (int)priv->playback);

          /* 报完就清：下一条窗口日志报的是**它这个窗口**里被吞掉的条数。
           * 只在 TE 这一路清，别把 timeout 那条日志变成 TE 计数的清零点。 */

          if (rx_err)
            {
              priv->rx_dma_err_silenced = 0;
            }
        }

      /* "RX 真的死了"的判据（两种情况各两条）：
       *
       * A) 冻死（原有判据，两条必须同时成立）：
       *   1) irq_delta == 0：整整 5 秒，完成中断一次都没来；
       *   2) dma_state_at_stop == BUSY：停机之前句柄仍停在 BUSY，也就是这一路
       *      DMA 从头到尾都武装着（通道使能 + TC/HT/TE 中断使能 + CNDTR 都写好了），
       *      没有任何人 abort 过它。
       * 两条合起来 = 冻结现场那种"Receive_DMA 回 HAL_OK、句柄也确实进过 BUSY，
       * 但完成中断一个都不来"的形态。这时 DMA 侧是干净的（NVIC 在每次成功分配
       * 时都会重新使能，见 bf0_hal_dma.c:299-309），坏的是它上游不再发数据请求，
       * 也就是 AUDPRC —— 所以这一次要做的是**完整的 AUDPRC 恢复**，不是再 arm 一遍。
       *
       * B) DMA 传输错误（本次新增）：rx_err 为真就是 TE。这一类在原来那对判据下
       *    **永远不会自愈**：HAL 自己在中断里就做完了 DMA_FreeChannel + State=READY
       *    （bf0_hal_dma.c:1228-1235），于是 2) 必然不成立（句柄是 READY 而不是
       *    BUSY），每次固定空转满 5 秒。TE 说明这一路 DMA 已经用不了了（通道被
       *    回收、TC/HT/TE 中断全被关），要接着录只能把它重新武装到干净状态上 ——
       *    和 A) 要做的是同一件事，所以并进同一个恢复调用，只是判据多这一条。
       *
       * 反过来说：只有 1) 成立（句柄已经不在 BUSY、又不是 TE）说明有人主动
       * abort 过它；只有 2) 成立（说明中断来过，是等待/唤醒那一侧的问题）——
       * 这两种都不做恢复，贸然复位 AUDPRC 反而把正常通路拆了。
       *
       * 恢复之后仍然按契约返回 -ETIMEDOUT（这一次确实没等到数据），
       * 下一次 read 会在恢复好的块上重新武装 DMA。 */

      if (rx_err ||
          (irq_at_stop == irq_before && dma_state_at_stop == HAL_DMA_STATE_BUSY))
        {
          priv->rx_recover_count++;
          sf32lb52_audio_recover_rx(priv);
        }

      return -ETIMEDOUT;
    }

  return buflen;
}

/****************************************************************************
 * Name: sf32lb52_audio_reserve / sf32lb52_audio_release
 *
 ****************************************************************************/

static int sf32lb52_audio_reserve(FAR struct audio_lowerhalf_s *dev)
{
  return OK;
}

static int sf32lb52_audio_release(FAR struct audio_lowerhalf_s *dev)
{
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sf32lb52_audio_initialize
 *
 ****************************************************************************/

int sf32lb52_audio_initialize(void)
{
  FAR struct sf32lb52_audio_s *priv;
  int ret;

  syslog(LOG_ERR, "AUDIO: initialize start\n");
  priv = kmm_zalloc(sizeof(struct sf32lb52_audio_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  ret = sf32lb52_audio_hw_init(priv);
  syslog(LOG_ERR, "AUDIO: hw_init ret=%d\n", ret);
  if (ret < 0)
    {
      kmm_free(priv);
      return ret;
    }

  nxsem_init(&priv->wr_sem, 0, 0);
  nxsem_init(&priv->rx_sem, 0, 0);

  /* 持有者身份显式给 -1：kmm_zalloc 出来是全 0，而 pid 0 是 idle 任务 ——
   * "没记过"和"真的记了一个 pid"必须分得开（见 struct 里那段说明）。 */

  priv->holder_tid = (pid_t)-1;
  priv->holder_pid = (pid_t)-1;
  priv->user_tid   = (pid_t)-1;

  priv->dev.ops = &g_sf32lb52_audio_ops;

  ret = audio_register("audio0", &priv->dev);
  syslog(LOG_ERR, "AUDIO: register ret=%d\n", ret);
  if (ret < 0)
    {
      auderr("ERROR: audio_register failed: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  audinfo("/dev/audio0 registered\n");
  return OK;
}

/****************************************************************************
 * HAL 弱符号回调重写（DMA 中断 → 本驱动）
 *
 ****************************************************************************/

void HAL_AUDPRC_TxCpltCallback(AUDPRC_HandleTypeDef *haprc, int cid)
{
  FAR struct sf32lb52_audio_s *priv;

  if (haprc == NULL)
    {
      return;
    }

  priv = (FAR struct sf32lb52_audio_s *)
         ((FAR char *)haprc - offsetof(struct sf32lb52_audio_s, aprc));

  if (priv->wr_busy)
    {
      priv->wr_busy = false;
      nxsem_post(&priv->wr_sem);
    }
  else
    {
      sf32lb52_audio_tx_complete(priv);
    }
}

void HAL_AUDCODEC_TxCpltCallback(AUDCODEC_HandleTypeDef *hacodec, int cid)
{
  FAR struct sf32lb52_audio_s *priv;

  if (hacodec == NULL)
    {
      return;
    }

  priv = (FAR struct sf32lb52_audio_s *)
         ((FAR char *)hacodec - offsetof(struct sf32lb52_audio_s, codec));

  if (priv->wr_busy)
    {
      priv->wr_busy = false;

      /* 传输完成：**先冻结再唤醒 write()**。
       * 顺序反了（或者干脆不冻结）就会在"唤醒 → 线程真的跑到 DMAStop()"
       * 这段窗口里从块首重播，每块一声"啪" —— 详见 tx_freeze 的注释。
       * write() 只用 DAC_CH0，别的通道进来不动。 */

      if (cid == HAL_AUDCODEC_DAC_CH0)
        {
          sf32lb52_audio_tx_freeze(priv);
        }

      nxsem_post(&priv->wr_sem);
    }
}

void HAL_AUDPRC_RxCpltCallback(AUDPRC_HandleTypeDef *haprc, int cid)
{
  FAR struct sf32lb52_audio_s *priv;

  if (haprc == NULL)
    {
      return;
    }

  priv = (FAR struct sf32lb52_audio_s *)
         ((FAR char *)haprc - offsetof(struct sf32lb52_audio_s, aprc));

  /* 无条件计数，**放在 rx_busy 判断之前**：这条只回答一个事实 ——
   * "RX 完成中断到底来过没有"。如果它在 read 超时时还是 0，就说明
   * DMA 根本没收到数据/中断没使能，而不是等待逻辑的问题。 */

  priv->rx_irq_count++;

  if (priv->rx_busy)
    {
      priv->rx_busy = false;
      nxsem_post(&priv->rx_sem);
    }
  else
    {
      sf32lb52_audio_rx_complete(priv);
    }
}

/****************************************************************************
 * Name: HAL_AUDPRC_RxHalfCpltCallback
 *
 * Description:
 *   RX 半满中断 —— "数据真的在流"的第二个心跳。
 *
 *   Receive_DMA 本来就把 RX 装成了 DMACIRCULAR 并把半满回调挂在句柄上
 *   （bf0_hal_audprc.c 里 AUDPRC_DMAHalfRxCplt → 这个 __weak 空实现），
 *   只是一直没人接。本驱动每帧只等整满那一次完成中断，一旦 TC 那一路丢了
 *   （或落在每帧 stop/re-arm 的窗口里被静默吞掉），日志上就只剩"irq=0"，
 *   分不清"数据源头压根没动"和"数据在流、只是完成通知丢了"。
 *   20ms 一帧 → 半满每 10ms 一次，这个计数就是那个分辨器。
 *
 *   这里**只累加一个计数器**：中断上下文里不能打串口日志（会抢控制台锁、
 *   把整机挂住，本文件踩过一次），也不能做任何会改变通路状态的事 ——
 *   它每 10ms 就来一次，做重活等于把录音通路拖垮。
 ****************************************************************************/

void HAL_AUDPRC_RxHalfCpltCallback(AUDPRC_HandleTypeDef *haprc, int cid)
{
  FAR struct sf32lb52_audio_s *priv;

  if (haprc == NULL)
    {
      return;
    }

  priv = (FAR struct sf32lb52_audio_s *)
         ((FAR char *)haprc - offsetof(struct sf32lb52_audio_s, aprc));

  /* 无条件自增（和 rx_irq_count 同一个规矩：这条只回答"来过没有"）。 */

  priv->rx_half_irq_count++;
}

/****************************************************************************
 * Name: HAL_AUDPRC_ErrorCallback
 *
 * Description:
 *   DMA 传输错误（TE）—— 这一类原来**永远不会自愈**，这里补上。
 *
 *   故障形态：TE 中断进来后 HAL 自己就做完了收尾
 *   （bf0_hal_dma.c:1228-1235：关 TC/HT/TE 中断 → DMA_FreeChannel →
 *   State=READY），随后 AUDPRC 侧的 AUDPRC_DMAError（bf0_hal_audprc.c:1062）
 *   把 aprc->State[cid] 也置 READY 再回调到这里。于是现场表现是：
 *       irq_delta == 0（完成中断确实没来）
 *       hdma[RX].State == 0x1（READY，而不是 BUSY）
 *   而 read() 里那条自愈判据要求"句柄仍 BUSY"，两条对不上 → 一次都不恢复，
 *   每次固定空转满 5 秒再返回 -110。
 *
 *   本回调只做三件**中断安全**的轻活（**不打日志**：串口那一侧会抢控制台锁、
 *   把整机挂住，本文件在播放完成回调里就为此删过一次 printf）：
 *     1) rx_dma_err_count++：TE 次数的唯一凭据；
 *     2) 立 rx_dma_err 旗：read() 的分片等待每 100ms 会看一眼，最坏 100ms
 *        就把这一帧收尾；
 *     3) 唤醒正在等帧的 read（post rx_sem），让它不必等那一片的 100ms。
 *   日志与恢复都在 read() 的收尾分支里做（那里能拿到 CNDTR/CCR/ISR 的
 *   停机前快照，信息比中断里全）。
 *
 *   只对 RX 通道立旗：TX0 只在队列模式（enqueuebuffer）里才会起来，那条路
 *   没有阻塞在信号量上的读者，立旗没人消费反而会污染下一次 read。
 ****************************************************************************/

void HAL_AUDPRC_ErrorCallback(AUDPRC_HandleTypeDef *haprc, int cid)
{
  FAR struct sf32lb52_audio_s *priv;

  if (haprc == NULL)
    {
      return;
    }

  priv = (FAR struct sf32lb52_audio_s *)
         ((FAR char *)haprc - offsetof(struct sf32lb52_audio_s, aprc));

  /* 计数无条件做：日志是限频的，计数器不是 —— "到底出过几次"只能靠它。
   * 错误码不用在这里记：HAL 在调本回调**之前**已经把它写进
   * hdma[cid]->ErrorCode（bf0_hal_dma.c:1228，值 HAL_DMA_ERROR_TE = 0x1），
   * read() 的收尾日志直接读那个字段；直到下一次武装那一刻才被 HAL 清掉
   * （HAL_DMA_Start_IT 开头置 NONE），所以这一帧读到的就是本帧的值。 */

  priv->rx_dma_err_count++;

  if (cid != SF32LB52_AUDIO_PRC_RX_CH || !priv->rx_busy)
    {
      /* 非 RX 通道，或者当前这一次等待已经收过尾（rx_busy 已清）：只计数。
       * 不立旗是必须的 —— 旗子只有一个消费者（read()），在没人等的时候立下去，
       * 下一次 read 会把它当成"自己这一帧出错了"。 */

      return;
    }

  priv->rx_dma_err = true;
  nxsem_post(&priv->rx_sem);
}
