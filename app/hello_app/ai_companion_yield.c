/**
 * ai_companion_yield.c - 跨 app 请求的薄壳：只登记请求，一个设备都不碰
 *
 * 装的是两个异步请求（都从这里受理、真动作都归 hello_app 自己的线程）：
 *   - 让路 / 收回（g_mic_req，**电平**语义：方向持续有效，让路线程每 100ms 读一次）；
 *   - 「提交」（g_voice_submit_req，**一次性**语义：镜像面板点一下 = "我说完了，
 *     立刻把这一段送去识别"，录音线程每帧认领一次）。收尾动作本身在
 *     ai_companion_main.c 的 audio_data_callback() 里，走的是和静音超时同一段代码。
 *
 * ★ 这个文件里**不许**出现任何 audio_* / audio_in_* / ioctl / close /
 *   pthread_join —— 一条都没有。理由见 ai_companion_yield.h 头上那次真机事故：
 *   跨 app（另一个 task group）替 hello_app 停设备，一旦不按预期收敛
 *   （AUDIOIOC_STOP 报 ENOTTY、录音线程 300ms 没退出、join 被放弃），留下的就是
 *   "录音线程还阻塞在 read() 里、fd 状态不明"的残局，紧接着在同一份驱动状态上
 *   起播放 hw_start，整个 hello_app 组就死了。
 *
 * 所以这里只做两件事，真动作全在 ai_companion_main.c
 * （ai_companion_listen_hold_impl() + 让路线程，那边的 static 是这个模块
 * 的实情，为什么用门面的理由写在 ai_companion_yield.h 末尾）：
 *
 *   1) 收下调用方的请求方向（g_mic_req）；
 *   2) 让 hello_app 的让路线程看见它 —— 线程每 100ms 读一次这个变量
 *      （ai_companion_main.c 的 yield_worker_task），所以"写变量"就是最直接的
 *      唤醒，本文件**依旧一个内核对象都不建**。这一点没变、也不许变：
 *      ai_companion 完全可能比 robot_ui 后起来，为它建对象本身就是一次跨 app 的
 *      资源申请，正好是这次要避免的那类操作。代价是请求最多晚一拍（100ms）被处理。
 *      （hello_app 那边自己起了信号量/线程来受理，那是它**自己组内**的事，
 *      和这里只登记请求不冲突：让路线程等不到 post 也有 100ms 超时兜着。）
 *
 * 为什么不需要锁：g_mic_req 承载的是"最后一次请求的方向"，**电平语义**，不是事件
 * 队列。让路线程每拍读一次，读到的永远是"最新的那个方向"：
 *   - robot_ui 在一个 tick 内先让路又收回 → 让路线程读到"收回"，而它上一拍也没
 *     让过路，于是什么都不做，结果正确；
 *   - 反过来让路线程读到稍旧的"让路" → 最多多让 100ms，下一拍读到"收回"就收回来。
 * 32 位对齐的 int 读写在 Cortex-M 上是原子的，一个变量足够；为它引入一把（可能
 * 还是跨 app 的）锁，只会把"设备动作权"又散到调用方那边去。
 * （2026-09-15 起旁边多了一个 g_mic_req_seq 序号，同样是"一个 32 位对齐变量自己
 *   管自己"，只在读它的时候一起读，不需要额外的锁或原子操作。）
 *
 * 旧版的 g_yielded 幂等标志和那把 g_yield_lock 都删掉了：判重留在 hello_app 那边
 * （ai_companion_main.c 的 g_mic_hold_active）—— 重复送进来的让路请求只会被当成
 * "还在让着"，不会再碰一次设备；而"谁有权动设备"这件事从此只有
 * hello_app 自己的线程。
 */

#include <nuttx/config.h>

#include "ai_companion_yield.h"

/* 最后一次收到的请求方向（电平语义，理由见文件头）。
 * 静态初始化成 NONE：ai_companion 还没跑的时候它也是个合法的可读变量 ——
 * 正好对上头文件那条"hello_app 没在跑时安全空转"的约束。 */

static volatile int g_mic_req = AI_COMPANION_MIC_REQ_NONE;

/* 请求序号：每登记一次（不管方向变没变）自增一次。
 *
 * 电平语义分不出"同一个让路被重复登记"和"早就没人管这次让路了"，而
 * ai_companion_main.c 的收回看门狗必须分得出来 —— 连续两次 yield(true)
 * 是调用方在说"我还在用"，不该被判成"漏了回收"。判据只看"变没变"，
 * 不看大小，所以 uint32 回绕无所谓（见头文件里的说明）。
 * 32 位对齐读写在 Cortex-M 上是原子的，和 g_mic_req 一样不需要锁。 */

static volatile uint32_t g_mic_req_seq;

/* 「提交」请求标志（一次性：置一次、被认领一次就清）。
 *
 * 和 g_mic_req 的语义差别写在头文件里：让路是**电平**（方向持续有效），提交是
 * **按钮动作**（点一下一次）。所以这里没有"方向"这个概念，就一个布尔位：
 *   - 写者：任何线程（现在只有 robot_ui 的 LVGL 线程：镜像面板点「提交」，
 *     经 main.c 转发 → ai_companion_voice_submit() → 本文件的写函数）；
 *   - 认领者：hello_app 那条录音线程（每帧一次，见 main.c 的数据回调）。
 * 32 位对齐的读写在 Cortex-M 上是原子的，一个位不需要锁（和 g_mic_req 同理）。*/
static volatile bool g_voice_submit_req;

void ai_companion_audio_yield(bool yield)
{
  /* 只写方向，立刻返回：不判重、不等设备、不碰 fd。 */

  g_mic_req = yield ? AI_COMPANION_MIC_REQ_YIELD
                    : AI_COMPANION_MIC_REQ_RECLAIM;
  g_mic_req_seq++;
}

void ai_companion_mic_reclaim(void)
{
  /* 和 ai_companion_audio_yield(false) 是同一件事。单开这个入口是给调用方的
   * 收尾处用的：那里只关心"还回去"，不该去读一个 bool 参数的语义。 */

  g_mic_req = AI_COMPANION_MIC_REQ_RECLAIM;
  g_mic_req_seq++;
}

int ai_companion_mic_request(void)
{
  /* 只给 ai_companion_main.c 的让路线程读（主循环那一拍也读它，但只用来判断
   * "要不要唤醒让路线程"）。不做"读走就清空"：清空会引入
   * "读到一半被覆盖"的丢请求窗口，而电平语义本来就不需要清（读的人自己记着
   * 当前是让着还是收着，见 g_mic_hold_active）。 */

  return g_mic_req;
}

uint32_t ai_companion_mic_request_seq(void)
{
  return g_mic_req_seq;
}

void ai_companion_mic_force_reclaim(void)
{
  /* 等价于调用方自己调了一次 ai_companion_mic_reclaim()：方向和序号一起动，
   * 之后模块内部读到的就是一个自洽的"收回"。只有 ai_companion_main.c 的
   * 收回看门狗会调它（理由见头文件）。 */

  g_mic_req = AI_COMPANION_MIC_REQ_RECLAIM;
  g_mic_req_seq++;
}

void ai_companion_voice_submit_request(void)
{
  /* 只置位，立刻返回：不判有没有在录音、不等任何人、不碰设备。
   * 该不该登记由调用方（ai_companion_voice_submit）先判过状态快照 —— 那里
   * 才看得到 hello_app 的 g_speech_capturing / g_speech_frames。 */

  g_voice_submit_req = true;
}

bool ai_companion_voice_submit_take(void)
{
  /* 认领即清：一次按钮动作最多被处理一次（理由见头文件里那两个入口的说明）。
   *
   * 为什么"读走就清"是安全的：唯一认领者是录音线程那一帧里的一段顺序代码，
   * 而登记请求的前置条件（ai_companion_voice_submit() 里判）是"此刻真的在常听、
   * 而且正在累积够长的一段语音"。所以这个位最多活到**下一帧**（≤ 20ms）就被
   * 认领掉；万一这一小段里录音被停了（robot_ui 恰好开始播提醒，hello_app 让路
   * 交出麦克风），请求会留到录音恢复后的第一帧 —— 那一刻上一次的累积已经被清场
   * 清掉了（让路收回 / 监听守护重开都会清 g_speech_capturing / g_speech_frames），
   * 而一句新的话要先攒够 300ms 语音才会被 VAD 认成"开始说话"，所以那一帧认领到的
   * 请求只会被丢掉（见 audio_data_callback 里的判据），不会拦腰截断老人下一句话。 */

  bool req = g_voice_submit_req;

  g_voice_submit_req = false;
  return req;
}
