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
 *   ⚠️ 配对纪律不变：**漏一次 reclaim 仍然要命**，所以配对的写法应该还是
 *   "yield(true) 之后到 reclaim 之间没有任何 return" —— 起播失败、等播放
 *   超时、正常放完，全部落到同一处收尾。
 *   2026-09-15 补了一道**看门狗兜底**（在 ai_companion_main.c 里，不在本文件）：
 *   让路被认领之后 MIC_HOLD_WATCHDOG_MS（10 秒，见那边的取值理由）没收到
 *   RECLAIM —— 不管是漏了、还是调用方自己卡住了 —— hello_app 会**强制收回**
 *   麦克风（恢复"要常听"、按正常路径重开常开麦），并打一行
 *   `[让路] 收回请求超时，强制恢复常开麦…`。
 *   所以"一直聋着"不再是永久状态；但代价是被误判之后那次出声会被打断，
 *   配对纪律和以前一样必须守。
 *   由此多了一条**可选的续租**：调用方在让路期间（比如一次很长的播报里）
 *   再调一次 ai_companion_audio_yield(true)，看门狗会重新计时，不会被误判。
 *   重复登记同一个方向本来就被允许（幂等），只是现在它多了一层"我还在用"的含义。
 *   强制收回之后，调用方如果还在轮询 ai_companion_mic_released()，会看到它一直
 *   报"没让出"（请求已经不算数了）—— 和"让路请求还没被认领"是同一种回答，
 *   调用方照旧按自己的超时处理即可，不需要为它加任何判断。
 *
 * ★ 2026-09-14 任务 C：本文件还承载第二个非阻塞请求 —— 「提交」
 *   （ai_companion_voice_submit()）：语音聊天镜像面板底部那个按钮，点一下
 *   = "我说完了，立刻把当前这段录音送去识别"，不等 VAD 的静音超时（3 秒）
 *   自己收尾。**和让路是两件事**，只是共用"只登记请求、真动作在 hello_app
 *   自己线程里"这一套写法：
 *     - 让路是**电平**（方向持续有效，hello_app 每拍读一次）；
 *     - 提交是**一次性动作**（置一次、认领一次就清，见文件末尾那两行说明）。
 *   收尾走的是和"VAD 判静音超时"**同一段**代码（ai_companion_main.c 的
 *   speech_capture_complete()），所以"语音结束"的语义只有一份；触发它的动作
 *   落在 hello_app 那条**录音线程**上（VAD 判定本身就在那条线程里），不是主
 *   循环、也不是让路线程 —— 理由见 ai_companion_main.c 的 audio_data_callback()。
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
#include <stdint.h>

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
 * 「提交」：请 ai_companion 立刻收尾当前这一段录音（非阻塞）
 *
 * 界面上的出处：语音聊天的**镜像面板**底部那个大按钮（用户原话：「下面是关闭
 * 按钮，我希望换成提交按钮」）。语义是"我说完了，立刻把这段录音送去识别"，
 * 不等 VAD 的静音超时（AUDIO_VAD_SILENCE_TIMEOUT_MS = 3 秒）自己收尾 ——
 * 老人说完话不用再干等 3 秒。
 *
 * 关闭仍然在面板右上角的「×」那一条路上，和这个请求无关。
 ****************************************************************************/

/* 提交请求的受理结果（ai_companion_voice_submit() 的返回值） */
typedef enum
{
  AI_COMPANION_SUBMIT_NONE = 0,   /* 没在累积语音：什么都没登记，调用方可以提示"没听到" */
  AI_COMPANION_SUBMIT_ACCEPTED,   /* 请求已登记：hello_app 下一帧就会把这一段送去识别 */
  AI_COMPANION_SUBMIT_BUSY        /* 它正忙（送识别 / 等大模型 / 出声 / 追问流程）：
                                   * 这一下不该插一脚，调用方也**不要**提示"没听到" ——
                                   * 界面上本来就有"正在想…/正在说话…" */
} ai_companion_submit_result_t;

/**
 * @brief  请 ai_companion 立刻收尾当前这一段录音。**非阻塞、任何线程可调。**
 *
 * @return 见 ai_companion_submit_result_t
 *
 * 它只做两件事：**读一次** hello_app 的语音状态（快照，只读，一个设备都不碰），
 * 有可提交的语音时**登记一个请求标志**（一次性：置一次、被认领一次就清）。
 * 真正"把这一段结束掉"的动作在 hello_app 自己那条**录音线程**上做 ——
 * 见 ai_companion_main.c 的 audio_data_callback() 里那段（为什么必须是那条线程：
 * VAD 判"说完了"本来就在那条线程里，用同一条线程走同一段收尾代码，才不会出现
 * "两条路同时收尾"；别的线程去触发会把整段 ASR 拽到那条线程上阻塞几十秒，
 * 让路协议就是为这件事改过一轮）。
 *
 * 三条纪律（和让路那两个入口一致）：
 *   1) 非阻塞，登记完立刻返回；下一次录音帧（≤ 20ms）就会被处理掉；
 *   2) hello_app 没在跑时安全空转：音频没初始化 / 没在常听 / 正在让路（麦克风
 *      在别人手里）一律报 NONE，**并且不登记请求** —— 一次性动作不能留在那里
 *      等"以后有人认领"，那会变成"老人几秒前那一按把后面的句子拦腰截断"；
 *   3) 调用方拿到 ACCEPTED 只是"请求登记上了"，别把它当成"识别结果马上就来"：
 *      结果照旧由 voice_state / user_said 那条链路推（robot_ui_bridge）。
 */
int ai_companion_voice_submit(void);

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
 *
 * 「提交」也一样：ai_companion_voice_submit()（读 g_speech_capturing /
 * g_speech_frames / g_sm_ctx 的那个入口）和下面的认领函数都实现在两个文件里，
 * 请求标志本身留在本文件的薄壳里（和一个设备都不碰的 g_mic_req 做邻居）。
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
 * @brief  让路请求的**序号**：每登记一次就自增一次（方向变没变都算）
 *
 * 电平语义分不出"同一个方向被重复登记了一次"和"早就没人管这次让路了"，而
 * ai_companion_main.c 里那个**收回看门狗**必须分得出来（连续两次 yield(true)
 * 说明调用方还在用着麦克风，不该被判成"漏了回收"）。所以这里额外记一个单调
 * 递增的序号：只要它变了，就说明调用方**又登记了一次**（见上面的"续租"）。
 *
 * 回绕是允许的：唯一的判据是"和上一次读到的值不一样"，不做大小比较。
 */
uint32_t ai_companion_mic_request_seq(void);

/**
 * @brief  把登记的请求方向改写成"收回"（只给 ai_companion_main.c 的看门狗用）
 *
 * 用在一个很具体的场合：让路被认领之后太久没等到 RECLAIM，hello_app 认定
 * 调用方已经不管这次让路了，于是**替它**把方向收回来（否则"请求还是让路、
 * 我们却已经收回"会让模块内部那几条按电平比较的判据来回横跳）。
 * 序号同样自增一次，等于"模块自己登记了一次收回"。
 *
 * 调用方之后再调用任何入口都会照常覆盖它，语义不变。
 */
void ai_companion_mic_force_reclaim(void);

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

/**
 * @brief  登记一次「提交」请求（一次性：置一次、被认领一次就清）
 *
 * 只给 ai_companion_main.c 的 ai_companion_voice_submit() 用（它先读状态快照，
 * 确认"真的在累积一段语音"才登记）。
 *
 * 为什么是"一次性"而让路那两个是"电平"：提交是**按钮动作**（点一下发生一次），
 * 不是持续有效的方向。电平语义在这里反而危险 —— 请求一直挂着的话，下一次
 * 录音帧（可能是几十秒后麦克风回来、老人重新说的另一句话）会被当成"这一下要
 * 立刻收尾"，把新句子拦腰截断。
 */
void ai_companion_voice_submit_request(void);

/**
 * @brief  认领一次「提交」请求（读走就清，返回"有没有人请求过"）
 *
 * 只给 ai_companion_main.c 的 audio_data_callback() 用（录音线程，每帧一次）。
 * 认领即清，所以一次按钮动作最多被处理一次。
 */
bool ai_companion_voice_submit_take(void);

/****************************************************************************
 * 这条语音追问立刻收摊：报警一旦真的走起来，另一条确认路就该停下（非阻塞）
 *
 * 登记方：robot_ui（robot_ui_show_alarm() 的入口处，任何报警来源都覆盖）。
 * 认领方：ai_companion_main.c 的 ask_flow_tick()（主循环，每 100ms 一拍）。
 *
 * 和上面三个请求一样的纪律：只置标志，一个设备都不碰、一页界面都不碰；
 * 真正的收摊由 hello_app 自己的线程做（相位只有一个写者）。
 * 一次性语义：认领即清，一次报警只会让这一次追问收摊，不会波及下一次。
 */
void ai_companion_ask_abort(void);

/**
 * @brief  认领一次追问收摊请求（读走就清）。只给 ask_flow_tick() 用。
 */
bool ai_companion_ask_abort_take(void);

#endif /* __AI_COMPANION_YIELD_H */
