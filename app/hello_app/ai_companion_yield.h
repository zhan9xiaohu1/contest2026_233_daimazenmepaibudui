/**
 * ai_companion_yield.h - 请 hello_app 的常开麦"让路"的跨 app 接口（非阻塞）
 *
 * 背景（提醒"只响第一块"的根因）：
 *   ai_companion 是**常开麦**的 —— 开机起一个录音线程一直占着
 *   /dev/audio/audio0 的录音通路；而 robot_ui 播提醒/提示音时是拿自己的
 *   g_audio_ctx 直接开播放设备。本板音频是**半双工**、驱动里只有一个方向标志，
 *   两边各看自己的 ctx，谁都不知道对方在用设备。于是提醒一出声就把设备抢过来：
 *   hello_app 的录音被无声地打断（它那边只会看到 read 返回 0，见 ai_audio.c），
 *   而提醒本身也只响一小段。
 *
 * ★ 为什么这个接口必须是非阻塞的（2026-09-14 真机事故）：
 *   原来它是**同步**的：robot_ui 的播报线程调 ai_companion_audio_yield(true)
 *   之后，停设备这件事（AUDIOIOC_STOP / close / pthread_join 录音线程）就在
 *   **调用方自己的线程里**替 hello_app 干了。可两边是各自独立的 task group，
 *   fd 和线程都归各自的组，跨组去动别人的 fd 和线程，一旦不按预期收敛就是灾难。
 *   那次串口原文：
 *     [Reminder] 提示音：让路 —— 先请 hello_app 交出麦克风
 *     AUDIO_IN: AUDIOIOC_STOP failed: 25          （ENOTTY，STOP 根本没生效）
 *     AUDIO_IN: stopped
 *     [AUDIO] stop: 录音线程 300ms 内没退出，放弃 join
 *     [让路] 麦克风已交出：常开监听已停（设备已还回去，等调用方出声）
 *     AUDIO: 通路位 adc_path_on -> 1（hw_start 开 ADC 模拟通路）
 *     （此后没有任何一行；事后 ps 里 hello_app(PID 12) 连它 3 个 pthread 全部消失）
 *   留下的残局是"录音线程还阻塞在 read() 里、fd 状态不明"，而播放在同一份驱动
 *   状态上起 hw_start(playback=1)，最终整个 hello_app 组死掉。
 *   结论：**跨 app 同步停设备这件事本身不该做**。设备的 stop / close / join 只能由
 *   持有它的那个 app 自己的线程去做。所以本接口改成"只登记请求、立刻返回"，
 *   真正的设备动作交给 hello_app 自己的线程；调用方改为**轮询**下面那两个查询函数。
 *
 * ★ 2026-09-14 任务 B：受理让路的又只能是"hello_app 的哪条线程"？
 *   v1 是 hello_app 的**主循环**（main_loop_task 每 100ms 一拍）。可那条线程会被
 *   **几十秒**的阻塞调用占住：process_ai_dialogue() 里 voice_asr_recognize() 是
 *   阻塞 HTTPS，追问流程 ask_speak() 里的 voice_tts_speak() 同理。于是"老人附近
 *   刚好有异响触发追问 / 正在对话"的那几十秒里，让路请求根本没人受理 → robot_ui
 *   等满 1.5 秒 → 提示音被驱动拒掉 → 铃不响。
 *   v2（现在）：hello_app 起了一条**让路线程**专门受理让路，它和主循环同属
 *   hello_app 这个 task group，不受主循环阻塞影响；主循环那一拍退化成"看到新请求
 *   就唤醒它"。对调用方来说**接口、签名、用法一个字都没变**，只是
 *   ai_companion_mic_released() 变快、变可靠了。
 *   （细节在 ai_companion_main.c 的让路线程那一节。）
 *
 * 用法（robot_ui 的提醒链路就是一个例子，见 app/robot_ui/main.c 的
 * reminder_play_exclusive()）：
 *
 *     ai_companion_audio_yield(true);      // 登记"请让路"，立刻返回（不碰设备）
 *     ... 轮询 ai_companion_mic_released()，自己带超时（推荐 1500ms）...
 *     ... 自己 audio_play_start() / 等整段真的放完 ...
 *     ai_companion_mic_reclaim();          // 撤回请求，立刻返回（无条件调）
 *
 *   ⚠️ 配对纪律不变：漏一次 reclaim，hello_app 就一直聋着（它"要常听"的意图
 *   被收掉了，监听守护也救不回来），比原来的毛病更糟。所以配对的写法应该还是
 *   "yield(true) 之后到 reclaim 之间没有任何 return" —— 起播失败、等播放
 *   超时、正常放完，全部落到同一处收尾。
 *
 * 为什么单独开一个文件：robot_ui 的 main.c 要 include 它，而 hello_app 这边的
 * 音频状态（g_audio_ctx / g_listen_wanted / g_sm_ctx）全是 ai_companion_main.c
 * 的 static。那个 main.c 连同它的依赖（mimo_voice / ai_tools_provider …）都带
 * 不进 robot_ui 的编译单元，所以这个头文件必须**自给自足**：只依赖 <stdbool.h>，
 * 不引 LVGL、不引 robot_ui、不引 hello_app 的任何别的头文件
 * （和 app/robot_ui/robot_ui_bridge.h 是同一套约束）。
 * 实现在两个新文件里：本目录的 ai_companion_yield.c（薄壳：只登记请求方向，
 * 一个设备都不碰）加上 ai_companion_main.c（真动作 + 状态回报，见下面"内部实现"）。
 *
 * ⚠️ 三条约束（调用方必须守）：
 *   1) **非阻塞，但也不等于立刻让开**：接口只登记请求，真正把设备腾出来要等
 *      hello_app 的让路线程下一拍（最多 100ms；主正在出声时会往后推，那时候设备
 *      本来就在 hello_app 自己手里，见 ai_companion_main.c 的 mic_audio_busy()）。
 *      那一次停的上限是 audio_record_stop 的 300ms 线程回收。所以调用方**必须轮询**
 *      ai_companion_mic_released() 并自己带超时 —— 接口这边不替它等，也不会有
 *      任何回调；超时之后就照常出声（打一行日志说明没等到）；
 *   2) **轮询只能在普通线程里，绝不能是 LVGL 线程**：轮询本身会让当前线程最多等
 *      1.5 秒，在 LVGL 线程里等会把界面冻住（这个项目已经因为同类问题冻过）；
 *   3) **hello_app 没在跑时安全空转**：整机是单一大镜像（CONFIG_BUILD_FLAT），
 *      符号在最终链接时解析，robot_ui 完全可能比 ai_companion 先起来。所有入口
 *      都只登记/读状态，一个设备都不碰；ai_companion_mic_released() 在音频根本
 *      没初始化时直接报"不在它手里"，调用方不用白等那 1.5 秒。
 *
 * 本接口只管"麦克风"这一半：它不会去打断 ai_companion 正在进行的 TTS / 追问
 * 出声（半双工设备上，调用方紧接着那次播放会把它们挤掉，这是设备层面的取舍，
 * 见 ai_audio.c 里 audio_prepare_output 那一段的说明）。
 */

#ifndef __AI_COMPANION_YIELD_H
#define __AI_COMPANION_YIELD_H

#include <stdbool.h>

/**
 * @brief  登记一个"请 ai_companion 交出麦克风（yield=true）/ 收回（yield=false）"
 *         的请求。**非阻塞，立刻返回，一个设备都不碰。**
 *
 * 它只做两件事：把请求方向写进一个跨 app 可见的变量 + 让 hello_app 的让路线程
 * （和主循环同属一个 task group，每 100ms 一拍）看见它。真正的"停设备 / 重开麦"
 * 由 hello_app 自己的线程完成（ai_companion_main.c 的 yield_worker_task()），
 * 调用方靠轮询 ai_companion_mic_released() 知道让路成没成。
 *
 * 幂等（重复让路、重复收回都只是把同一个方向再写一次），线程安全（单变量电平
 * 语义，不需要锁），任何线程都能调。
 *
 * @param  yield true = 请求让出；false = 请求收回（等价于 ai_companion_mic_reclaim()）
 */
void ai_companion_audio_yield(bool yield);

/**
 * @brief  麦克风现在是否真的**不在 hello_app 手里**了（非阻塞，任何线程可调）
 *
 * @return 1 = 已经让出去了，调用方可以放心出声；0 = 还没让出来
 *
 * 判据（定义在 ai_companion_main.c —— 只有那里看得到 hello_app 的音频状态）：
 *   1) hello_app 的音频还没初始化（它没起来 / 已经收摊）→ 直接返回 1：
 *      它不可能占着设备，调用方不必白等；
 *   2) 让路请求还没被 hello_app 的让路线程认领（100ms 一拍，最多晚 100ms；主
 *      正在出声时会往后推，见文件头上那条）→ 返回 0；
 *   3) 认领者正拿着设备开关（停/开麦那一手还没做完）→ 返回 0：这一刻的状态是
 *      "半程"，只有那一手做完才算数；
 *   4) 认领之后，还要**设备真的空了**才算，三条都要满足：
 *      - g_audio_started == false："我还想常听"的意图已经收掉；
 *      - !audio_record_is_active()：设备没在 START 状态、或者已经有人喊停；
 *      - 录音线程**真的收尾了**（record_thread_valid == false 或者
 *        record_exited == true）。
 *      最后一条是专门为那次事故加的：线程"还没退出"时（AUDIOIOC_STOP 报错、
 *      线程仍阻塞在 read 里）光看前两条会误报"已经让出去了"，而那恰恰是最危险的
 *      时刻 —— 宁可让调用方等到它自己的超时，也不能让它以为设备空了。
 *   （2）/（3）这两条都是"报 0 的理由"，判据只比 v1 更严，没有一处放松。
 */
int ai_companion_mic_released(void);

/**
 * @brief  撤回让路请求（= 旧的 yield(false)）。**非阻塞、无条件可调。**
 *
 * 只登记"收回"这个方向；真正重开麦同样由 hello_app 的让路线程下一拍在做：先立回
 * "要常听"的意图，再清场（半截语音累积 / KWS 流式状态），再
 * start_audio_listening()；重开失败由监听守护按退避接着重试。
 * 调用方**不需要**等，也没有可等的东西 —— 所以它适合放在单出口收尾处无条件调，
 * 保证"让出去了一定还回来"。
 *
 * （漏一次收回的后果不变：让路线程一直以为"还在让着"，hello_app 就一直聋着 ——
 *   它"要常听"的意图被收掉了，监听守护也救不回来。）
 */
void ai_companion_mic_reclaim(void);

/****************************************************************************
 * 内部实现（薄壳在 ai_companion_yield.c，真动作在 ai_companion_main.c）
 *
 * 为什么在 main.c 里留一个薄门面，而不是把这些 static 放开可见性：
 *   g_listen_wanted 的语义（"想常听"而不是"已经听上"，开机第一次开麦前就置位）、
 *   g_speech_capturing / g_speech_frames / KWS 流式状态**什么时候清才安全**
 *   （必须等录音线程死透了，否则是在和录音线程抢同一份状态）、以及让路/收回时
 *   该不该动状态机 —— 这些约束的解释都写在 main.c 那几段注释里。放开可见性等于
 *   把"谁有权改、什么时候能改"散到两个文件，外面乱调就没人拦得住；加一层门面
 *   代价最小，也保证让路走的是**和监听守护同一段**逻辑，不会各自漂移。
 * （app/robot_ui/robot_ui_bridge.h 的"内部桥接原语"是同一套做法。）
 ****************************************************************************/

/* 让路请求的方向。薄壳（ai_companion_yield.c）写、hello_app 的让路线程读；
 * 电平语义：读到的永远是"调用方最新一次请求的方向"，不是事件队列。 */

enum
{
  AI_COMPANION_MIC_REQ_NONE    = 0,   /* 还没有人请求过 */
  AI_COMPANION_MIC_REQ_YIELD   = 1,   /* 请求让出麦克风 */
  AI_COMPANION_MIC_REQ_RECLAIM = 2    /* 请求收回（重开常开监听） */
};

/**
 * @brief  读当前登记着的让路请求方向（返回上面那三个值之一）
 *
 * 只给 ai_companion_main.c 用：让路线程每 100ms 读一次，按电平决定要不要动作
 * （主循环那一拍也读它，但只用来判断"要不要唤醒让路线程"）。
 */
int ai_companion_mic_request(void);

/**
 * @brief  真正的"让路 / 收回"设备动作（用 ai_companion_main.c 的 static 干活）
 * @param  hold true = 交出麦克风并停掉常开监听；false = 收回并试着重开
 *
 * ⚠️ 只允许 hello_app 自己的线程调（现在是 ai_companion_main.c 的让路线程，以及
 * 让路线程起不来时退回的主循环）：里面会 audio_record_stop() → pthread_join
 * 录音线程、并动音频设备。绝不能从别的 app / LVGL 线程调进来 —— 理由见本文件头上
 * 那次事故。
 */
void ai_companion_listen_hold_impl(bool hold);

#endif /* __AI_COMPANION_YIELD_H */
