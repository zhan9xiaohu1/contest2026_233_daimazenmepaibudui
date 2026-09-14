/**
 * ai_companion_yield.c - "让路"接口的薄壳：只登记请求，一个设备都不碰
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

void ai_companion_audio_yield(bool yield)
{
  /* 只写方向，立刻返回：不判重、不等设备、不碰 fd。 */

  g_mic_req = yield ? AI_COMPANION_MIC_REQ_YIELD
                    : AI_COMPANION_MIC_REQ_RECLAIM;
}

void ai_companion_mic_reclaim(void)
{
  /* 和 ai_companion_audio_yield(false) 是同一件事。单开这个入口是给调用方的
   * 收尾处用的：那里只关心"还回去"，不该去读一个 bool 参数的语义。 */

  g_mic_req = AI_COMPANION_MIC_REQ_RECLAIM;
}

int ai_companion_mic_request(void)
{
  /* 只给 ai_companion_main.c 的让路线程读（主循环那一拍也读它，但只用来判断
   * "要不要唤醒让路线程"）。不做"读走就清空"：清空会引入
   * "读到一半被覆盖"的丢请求窗口，而电平语义本来就不需要清（读的人自己记着
   * 当前是让着还是收着，见 g_mic_hold_active）。 */

  return g_mic_req;
}
