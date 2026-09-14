/****************************************************************************
 * AI Companion Main Entry
 * 智爱陪伴 - AI老人陪伴守护终端
 * 主程序入口 - 初始化并运行状态机
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <semaphore.h>

#include "ai_state_machine.h"
#include "ai_audio.h"
#include "ai_llm.h"
#include "ai_sound_detect.h"
#include "ai_care.h"
#include "ai_network.h"
#include "ai_network_config.h"
#include "kws_dtw.h"

/* 把"要显示的东西"直接推给界面（同一个地址空间里的 robot_ui）。
 *
 * 整机是单一大镜像（CONFIG_BUILD_FLAT）：界面和这里在同一个地址空间、符号在
 * 最终链接时解析，本来一次函数调用就够。原来界面反馈要绕公网 MQTT（publish 到
 * zhi_ai/<client_id>/command 再由 robot_ui 的 network_task 收回来），
 * 那条路上任何一环慢/丢/断，用户看到的就是"说话完全没反应"。
 * 这个头文件是自给自足的（不引 LVGL，hello_app 编不了 LVGL）。 */
#include "robot_ui_bridge.h"

/* 让路接口（robot_ui 播报提醒前请我们交出麦克风）：声明在那个自给自足的头文件
 * 里，真动作在本文件的让路线程（yield_worker_task）/ 
 * ai_companion_listen_hold_impl()。实现在这里而不是 ai_companion_yield.c，
 * 是因为它要直接读写本文件的一堆 static（g_listen_wanted / g_audio_ctx /
 * g_speech_capturing / g_sm_ctx）—— 理由见头文件末尾那段。
 * ⚠️ 设备动作**只能**从 hello_app 自己的线程发起，就两条：让路线程，和监听守护
 * （listen_supervise_tick，跑在主循环线程里）—— 理由见 mic_hold_apply 的说明。 */
#include "ai_companion_yield.h"

#include "voice/voice_asr.h"
#include "voice/voice_tts.h"
#include "volc_asr.h"
#include "volc_tts.h"
#include "mimo_voice.h"
#include "ai_tools_provider.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 主循环间隔(毫秒) */
#define MAIN_LOOP_INTERVAL_MS    100

/* ---- 语音状态回传 (S2) ----
 *
 * 语音入口搬到这里之后，界面对语音链路的进展一无所知（用户原话：
 * "我怎么知道他在听他什么时候回复"）。四个状态有**两条**出路：
 *
 *   1) 主路 —— 同进程直调界面（robot_ui_bridge.h 的
 *      robot_ui_bridge_voice_state()）。整机单一大镜像，界面就在同一个地址空间，
 *      本机自己的显示没有任何理由依赖网络；
 *   2) 旁路 —— MQTT publish 到 zhi_ai/<client_id>/command，给手机/PC 端
 *      （它们不在这个地址空间里）。动作名和状态串要和 robot_ui 的
 *      on_ai_command_received() 对上：
 *
 *   {"action":"voice_state","param":"listening"}  正在听用户说话
 *   {"action":"voice_state","param":"thinking"}   送 ASR / 大模型，等回复
 *   {"action":"voice_state","param":"speaking"}   TTS 正在出声
 *   {"action":"voice_state","param":"idle"}       常听 / 空闲，没有进行中的事
 *
 * 两条都发的原因：直调解决"本机界面没反应"，MQTT 解决"远端看不见"。
 * 原来只有 MQTT 这一条，开机时它常常连不上（RNDIS/DNS 还没就绪），
 * 于是本机界面从头到尾一动不动 —— 用户口中的"语音聊天卡死、说话没反应"。
 */

#define VOICE_STATE_ACTION       "voice_state"
#define VOICE_STATE_LISTENING    "listening"
#define VOICE_STATE_THINKING     "thinking"
#define VOICE_STATE_SPEAKING     "speaking"
#define VOICE_STATE_IDLE         "idle"

/* ---- 用户原话回传 (S2) ----
 *
 * 老人回头看不到"我刚说了什么"，对话看着像单向的（用户原话："又看不到回复又
 * 看不到自己说了什么"）。所以 ASR 出文本的那一刻把原文也发一份出去，
 * 同样是两条路：主路直调 robot_ui_bridge_voice_user_said()，旁路 MQTT
 * 发 {"action":"user_said","param":"<ASR 原文>"}（动作名和 robot_ui 的
 * on_ai_command_received() 对上，手机端也订着它）。
 *
 * 界面拿它和 ai_reply 摆进同一条对话时间线（写完这句的是用户，回的是它）。 */

#define AI_CMD_ACTION_USER_SAID  "user_said"

/* 回传给界面的用户原话上限（字节，不含结尾 '\0'）。
 * MQTT 那一路：ai_network_publish_command() 的参数缓冲是 AI_CMD_PARAM_MAX(512)，
 * ASR 的 text_buf 也是 512，理论上塞得下；但 JSON 转义会把 " 和 \ 涨成 2 字节，
 * 撑爆后 ai_network_json_escape() 会把参数**整段丢空**（界面收到一条空消息）。
 * 直调那一路：界面自己的显示缓冲也只有 256（robot_ui_bridge.h 的
 * ROBOT_UI_BRIDGE_TEXT_MAX），两边在这里取同一个上限，省得两路显示不一样。 */
#define USER_SAID_MAX            256

/* ---- 异常声追问流程的参数 (W1) ---- */

/* 同一阵响声的防抖时间：这段时间内再来一次检测直接丢掉 */
#define ASK_DEBOUNCE_MS          10000

/* 每轮追问最多听多久（毫秒） */
#define ASK_LISTEN_TIMEOUT_MS    6000

/* 迟迟收不到"播放完成"回调时的兜底（毫秒）。
 * audio_play_start() 的播放被 stop 打断时回调不会触发，不兜底就一直卡在 SPEAKING。 */
#define ASK_SPEAK_TIMEOUT_MS     8000

/* 追问总轮数上限（含第一次）：问满这么多轮还听不清就上报"未确认" */
#define ASK_MAX_ROUNDS           2

/* 追问用的 TTS 缓冲（"请问需要帮助吗？"这种短句约 2 秒 ≈ 64 KB） */
#define ASK_TTS_BUF_BYTES        (128 * 1024)

/* ASR 结果缓冲 */
#define ASK_ANSWER_MAX           256

/* 报警/推送里的详情串长度 */
#define ASK_DETAIL_MAX           192

/* 追问回答的判定结果 */
#define ASK_VERDICT_REASSURED    0    /* 不用帮忙 */
#define ASK_VERDICT_EMERGENCY    1    /* 需要帮忙 */
#define ASK_VERDICT_UNCLEAR      2    /* 听不清 / 不含关键词 */

/* ---- 本机灯控（"说一句话就把灯打开"）----
 *
 * 队友的灯模拟器订阅的是 zhi_ai/<client_id>/device_cmd，command 收的是
 * **裸字符串** "on"/"off"（见 robot_ui/network_comm.c 的 send_device_command()：
 * cJSON_AddStringToObject(root, "command", command)，它不会帮你拼 JSON，
 * 所以这里千万别把 command 写成 {"state":"on"} 之类的整串）。 */

#define LIGHT_DEVICE_ID          "living_room_light"
#define LIGHT_CMD_ON             "on"
#define LIGHT_CMD_OFF            "off"

/* 命中后本地回话（走和 AI 回复同一条 TTS 播报链路） */
#define LIGHT_REPLY_ON           "好，灯打开了"
#define LIGHT_REPLY_OFF          "好，灯关掉了"

/* 命令没发出去时的兜底：宁可明说"没搞定"，也不能不出声或者让界面一直转圈 */
#define LIGHT_FAIL_ON            "网络没连上，灯没打开"
#define LIGHT_FAIL_OFF           "网络没连上，灯没关掉"

/* 灯控回话的 TTS 缓冲。这句话 6 个字 ≈ 1 秒 ≈ 32 KB，给 4 秒余量，
 * 免得 TTS 返回的 PCM 被底层按 buffer full 静默截尾（volc_tts.c 的行为）。
 * 必须走堆：静态 128 KB 会把内核 SRAM 顶掉，堆里的大块会落到 PSRAM。 */
#define LIGHT_TTS_BUF_BYTES      (128 * 1024)

/* 灯控识别的文本上限：ASR 这次给的就是 512 的缓冲，一句灯控命令远不到这么长，
 * 超过就按"拿不准"处理，直接退回大模型。 */
#define LIGHT_TEXT_MAX           512

/* 程序退出标志 */
static volatile sig_atomic_t g_running = 1;
static bool g_audio_started;
static bool g_sound_started;
static bool g_care_started;
static bool g_net_started;

/* 上层"要常听"的意图（不是"已经听上了"）。
 * 开机第一次 start_audio_listening() 之前就置 true：那一次要是就失败
 * （比如设备忙），监听守护才会去重试。stop_audio_listening() 里清掉，
 * 免得退出流程刚关了麦又被守护拉起来。 */

static volatile bool g_listen_wanted;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* 全局状态机上下文 */
static sm_context_t g_sm_ctx;

/* 全局音频模块上下文 */
static audio_context_t g_audio_ctx;

/* 全局LLM模块上下文 */
static llm_context_t g_llm_ctx;

/* 全局声音检测上下文 */
static sound_detect_context_t g_sound_ctx;

/* 全局主动关怀上下文 */
static care_context_t g_care_ctx;

/* 全局网络上下文（AI 回复/表情/报警要回传给界面，见 ai_network.h） */
static ai_network_context_t g_net_ctx;

/* 语音段累积缓冲区 (用于 ASR) */

#define SPEECH_BUF_MAX_FRAMES  (16000 * 10)  /* 最长10秒 @16kHz */
static int16_t *g_speech_buf = NULL;
static size_t   g_speech_frames = 0;
static bool     g_speech_capturing = false;

/* 上一次成功发给界面的语音状态串（空串 = 还没发过），去重就靠它 */
static char g_voice_state_sent[16];

/* 上一次判去重时"界面就绪了没有"（robot_ui_bridge_is_ready() 的结果）。
 * 状态串没变、但界面从"没就绪"变成"就绪"时要补推一次 —— 那一轮之前的推送
 * 都被桥接那层丢掉了（界面还在 lv_init），不补的话界面上就永远停在开机默认值。
 * 界面一直没起来（robot_ui 不在跑）时它恒为 false，去重照常生效，不会刷屏。 */
static bool g_voice_state_ui_ready;

/* 本轮对话是否已经出过声：两个播放完成回调置位，VAD 又听到人开始时清掉。
 *
 * 播放线程的收尾顺序是"先关播放/恢复录音，再回调上层"，中间有一小段
 * 已经不在放音、状态机却还没从 AI_TALKING 切走的时间。没有这个标志的话，
 * 界面会从"正在说话…"闪一下"正在想…"再回到空闲。 */
static volatile bool g_voice_spoke;

/****************************************************************************
 * 唤醒词「你好，openvela」/「Hello，openvela」的命令词识别 (KWS)
 *
 * 识别算法全在 kws_dtw.c 里（纯本地纯计算，不开麦、不起线程）。这里只做
 * 接线，两件事：
 *   - 录音线程的 audio_data_callback() 里把每块 PCM 喂给 kws_feed()，
 *     命中时**只置一个标志**（回调每 20ms 跑一次，DTW 那一下是毫秒级突发，
 *     不能让录音线程去做流程：不在这里发网络、不碰 LVGL、不刷串口）；
 *   - 主循环 kws_wake_tick() 取这个标志，走和"VAD 检测到有人说话"同一条路。
 *
 * 两个标志都是"一个线程写、另一个线程读"的单个 bool，在 ARM32 上对齐读写
 * 不会读到半截值，所以不加锁（kws_dtw 自己也要求它的接口只被一个线程调 ——
 * 那个线程就是录音线程，kws_init() 在它起来之前调一次）。
 *
 * g_kws_ready 是开机时定下来的模板条数，只用来打日志。
 * g_kws_enabled = 有模板才算"功能开了"：没有模板时 kws_feed 永远返回 0，
 *   喂不喂结果一样，跳过只是为了省掉 MFCC 那 2~3% 的 CPU。也正因为跳过，
 *   现场没录模板时，整条链路的行为和接这个模块之前**完全一样**（只有 VAD 触发）。
 ****************************************************************************/

static int           g_kws_ready;      /* 开机时从 /data/kws 载入的模板条数 */
static bool          g_kws_enabled;    /* 有模板才喂帧、才处理命中 */
static volatile bool g_kws_hit;        /* 录音线程置：刚命中唤醒词 */

/****************************************************************************
 * 异常声 -> 追问流程的共享状态 (W1)
 *
 * 相位（g_ask_phase）只有一个写者：main_loop_task 里的 ask_flow_tick()。
 * 别的线程（播放线程 / 录音线程 / 检测线程）只置事件标志，不改相位，
 * 免得几个人同时推流程。
 ****************************************************************************/

typedef enum
{
  ASK_PHASE_IDLE = 0,     /* 空闲，等着检测到异常声 */
  ASK_PHASE_PENDING,      /* 检测到了，等 tick 开始问 */
  ASK_PHASE_SPEAKING,     /* 正在说「请问需要帮助吗？」 */
  ASK_PHASE_LISTENING     /* 限时听回答：等 ASR 结果或超时 */
} ask_phase_t;

static volatile int      g_ask_phase = ASK_PHASE_IDLE;
static volatile int      g_ask_pending_type = SOUND_TYPE_NONE;
static volatile float    g_ask_pending_conf;
static volatile int      g_ask_round;          /* 已经问过几轮 */
static volatile bool     g_ask_play_done;      /* 播放线程置：这轮的问话说完了 */
static volatile bool     g_ask_answer_ready;   /* 录音线程置：ASR 有结果了（空串=没听清） */
static volatile uint32_t g_ask_phase_ms;       /* 进入当前相位的时间 */
static volatile uint32_t g_ask_deadline_ms;    /* ASK_PHASE_LISTENING 的截止时间 */
static volatile uint32_t g_last_abnormal_ms;   /* 上次触发时间（防抖用） */
static char              g_ask_answer[ASK_ANSWER_MAX];

static void print_usage(const char *program)
{
  printf("用法: %s [选项]\n", program);
  printf("  --ask <文本>          启动后向 MiMo/ai_agent 发送文本\n");
  printf("  --sound-self-test     注入测试冲击声，验证检测链路\n");
  printf("  --ask-self-test       注入一条假的异常声，跑一遍追问流程\n");
  printf("  --client-id <ID>      MQTT 客户端 ID（默认 %s）\n",
         AI_DEFAULT_MQTT_CLIENT_ID);
  printf("  --help                显示帮助\n");
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**
 * @brief  取单调时钟的毫秒数（追问流程算超时用）
 */

static uint32_t main_now_ms(void)
{
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

/****************************************************************************
 * 语音状态回传 (S2)
 *
 * 界面（robot_ui）只能从 MQTT 知道语音链路走到哪一步了，这里把
 * 「在听 / 在想 / 在说 / 空闲」推过去。没有新的状态机，四个态全部从
 * 现成的量算出来（唯一多记的是"这轮出过声没有"，见 g_voice_spoke）：
 *   - audio_is_playing()   板级播放标志（TTS 出声期间为真）
 *   - g_ask_phase          异常声追问流程的相位
 *   - g_speech_capturing   VAD 报过"开始说话"、还没报"说完了"
 *   - sm_get_state()       IDLE / LISTENING / AI_TALKING / ...
 ****************************************************************************/

/**
 * @brief  算出当前该显示哪个语音状态
 */

static const char *voice_state_probe(sm_context_t *ctx)
{
  /* 扬声器在响 = 我在说话。这条要排最前面：TTS 出声时状态机还停在
   * AI_TALKING，不先判播放，界面就会一直显示"正在想…"。 */

  if (audio_is_playing(&g_audio_ctx) || g_ask_phase == ASK_PHASE_SPEAKING)
    {
      return VOICE_STATE_SPEAKING;
    }

  /* 正听到人说话，或追问流程在限时等回答 = 我在听。
   * 追问那条链路在等回答期间状态机可能还停在上一轮的 AI_TALKING
   * （要等 ask_flow_finish() 才补 AI_RESPONSE），所以这里要排在状态机前面。 */

  if (g_speech_capturing || g_ask_phase == ASK_PHASE_LISTENING)
    {
      return VOICE_STATE_LISTENING;
    }

  /* 追问流程刚被异常声触发、问句还在 TTS 合成里（这个相位从"排入"一直
   * 持续到 ask_flow_begin() 里开播）= 跟等大模型回复一样，算"正在想" */

  if (g_ask_phase == ASK_PHASE_PENDING)
    {
      return VOICE_STATE_THINKING;
    }

  /* AI_TALKING 覆盖 ASR + 等大模型 + TTS 合成这一整段。
   * 已经出过声的那一小段（见 g_voice_spoke）不算"正在想"。 */

  if (sm_get_state(ctx) == SM_STATE_AI_TALKING && !g_voice_spoke)
    {
      return VOICE_STATE_THINKING;
    }

  /* 剩下的都当空闲：IDLE / LISTENING（VAD 常开但没人说话）/ 报警态。 */

  return VOICE_STATE_IDLE;
}

/**
 * @brief  把上面的状态串翻成"直调界面"那个接口的档位
 *
 * 状态串是 MQTT 那一路的线协议（要和 robot_ui 的 on_ai_command_received()
 * 对上，也要和手机端对上），档位枚举是直调那一路的接口（在 robot_ui_bridge.h
 * 里自己定了一套，因为那个头文件不能引 LVGL）。两套值就在这里对上，
 * 别的地方一律用串，免得出现第三种表示。
 *
 * 认不出来的串一律按"空闲"处理：界面宁可少刷一次，也不要停在一个错的字上。
 */

static int voice_state_to_bridge(const char *state)
{
  if (state == NULL)
    {
      return ROBOT_UI_BRIDGE_VOICE_IDLE;
    }

  if (strcmp(state, VOICE_STATE_LISTENING) == 0)
    {
      return ROBOT_UI_BRIDGE_VOICE_LISTENING;
    }

  if (strcmp(state, VOICE_STATE_THINKING) == 0)
    {
      return ROBOT_UI_BRIDGE_VOICE_THINKING;
    }

  if (strcmp(state, VOICE_STATE_SPEAKING) == 0)
    {
      return ROBOT_UI_BRIDGE_VOICE_SPEAKING;
    }

  return ROBOT_UI_BRIDGE_VOICE_IDLE;
}

/**
 * @brief  语音状态有变化就发给界面（主循环每 100ms 调一次）
 *
 * 只在真的变化时发：状态没变就一个字节都不发（否则一秒十条，MQTT 和界面
 * 日志都别看了）。
 *
 * ★ 两条路一起走，不是二选一：
 *   1) **同进程直调**（主路）：robot_ui_bridge_voice_state()，本地立刻刷屏。
 *      整机是单一大镜像，界面就在同一个地址空间里，本机自己的界面反馈完全
 *      不该依赖公网一个来回 —— 真机上就是因为这条 MQTT 路断着（开机时
 *      DNS 还没就绪 -> MQTT connect 失败 -> g_net_started 恒为 false），
 *      老人说话时界面从头到尾一动不动，看着就是"卡死/没反应"。
 *   2) **MQTT 照发**（保留）：手机/PC 端还订阅着 zhi_ai/<client_id>/command，
 *      它们不在这个地址空间里，只能靠 MQTT。断了它们就瞎了。
 *      发失败只打一行日志：这只是个提示，不能反过来影响语音链路。
 */

static void voice_state_tick(sm_context_t *ctx)
{
  const char *want;
  bool ui_ready;

  want = voice_state_probe(ctx);
  ui_ready = robot_ui_bridge_is_ready();

  /* 去重：状态没变、而且界面的就绪状态也没变，就一个字节都不发。
   *
   * 为什么要带上 ui_ready 一起判：界面比我们先跑起来的情况是存在的
   * （robot_ui 还在 lv_init / 建控件时我们这一轮就已经在推状态了，那种推送会被
   * 桥接那层丢掉）。只看状态串的话，那一次"被丢掉的 idle"会被记成"已发"，
   * 之后再也不会补 —— 界面上就一直停在开机默认显示。带上这一位之后，
   * 界面就绪那一刻会**补推一次当前状态**，之后照常去重。
   * 也不会因此刷屏：界面一直没起来时这一位恒为 false，去重照旧生效。 */

  if (strcmp(want, g_voice_state_sent) == 0 && ui_ready == g_voice_state_ui_ready)
    {
      return;
    }

  /* 先记下来再发：发不出去也不重试（网络不好时这里会一直刷失败日志），
   * 界面看到的就还是上一次成功收到的状态。 */

  snprintf(g_voice_state_sent, sizeof(g_voice_state_sent), "%s", want);
  g_voice_state_ui_ready = ui_ready;

  /* 主路：同进程直调界面（无网络依赖，任何时刻都能刷） */

  robot_ui_bridge_voice_state(voice_state_to_bridge(want));

  /* 旁路：公网 MQTT，给手机/PC 端。没连上就只记一条日志 —— 界面那边已经由
   * 上面那条直调刷过了，这里失败不再等于"用户看不到"。 */

  if (!g_net_started)
    {
      /* 回传通道没起来（开机时 RNDIS/DNS 还没就绪的常见情况）：本地语音链路
       * 和界面直调都照常，只是手机端收不到状态了。 */
      return;
    }

  if (ai_network_publish_command(&g_net_ctx, VOICE_STATE_ACTION, want) < 0)
    {
      printf("[语音] 状态 MQTT 回传失败: %s（界面已由直调刷过，手机端看不到）\n",
             want);
    }
}

/**
 * @brief  信号处理函数
 */

static void signal_handler(int signo)
{
  (void)signo;
  g_running = 0;
}

/**
 * @brief  开始累积一句话（VAD 报"人开始说话" 和 唤醒词命中 共用这一段）
 *
 * 抽出来的目的就是让"唤醒词命中"和"检测到有人说话"走**同一段代码**：
 * 两条路要的后续完全一样（清累积长度 → 置 capturing → 状态机收 WAKEUP →
 * 等这一次说话结束时报 VOICE_COMPLETE，由 AI_TALKING 进去送 ASR）。
 * 合成一处以后改一边不会漏另一边。
 */

static void speech_capture_begin(sm_context_t *ctx)
{
  g_speech_frames = 0;              /* 从这一块开始重新累积 */
  g_speech_capturing = true;
  g_voice_spoke = false;            /* 新一轮说话开始：这轮还没出过声 */
  sm_handle_event(ctx, SM_EVENT_WAKEUP);
}

/**
 * @brief  VAD回调 - 语音活动检测
 */

static void vad_callback(bool speech_detected, void *user_data)
{
  sm_context_t *ctx = (sm_context_t *)user_data;

  if (speech_detected)
    {
      printf("[VAD] 检测到语音开始\n");
      speech_capture_begin(ctx);
    }
  else
    {
      printf("[VAD] 语音结束 (累积 %zu 帧)\n", g_speech_frames);
      g_speech_capturing = false;
      sm_handle_event(ctx, SM_EVENT_VOICE_COMPLETE);
    }
}

static void audio_data_callback(const int16_t *data, size_t frames,
                                void *user_data)
{
  (void)user_data;

  /* 累积语音数据到缓冲区，供 ASR 使用 */

  if (g_speech_capturing && g_speech_buf != NULL)
    {
      size_t space = SPEECH_BUF_MAX_FRAMES - g_speech_frames;
      size_t copy = frames < space ? frames : space;
      if (copy > 0)
        {
          memcpy(&g_speech_buf[g_speech_frames], data,
                 copy * sizeof(int16_t));
          g_speech_frames += copy;
        }
    }

  /* 送入唤醒词识别（命令词，纯本地计算）。
   *
   * 只在录音线程里调（kws_dtw 的线程约束就是"接口只允许一个线程调"），
   * 这个回调正是那个线程。帧口径：frames 是**每通道采样点数**，
   * 本板单声道 → 就是采样点数，正好是 kws_feed() 要的口径；
   * 窗边界不用我们管，它内部自己攒 20ms 窗（10ms 跳，50% 重叠）。
   *
   * 命中时只置标志：流程交给主循环的 kws_wake_tick()。这里绝不能做
   * 耗时/阻塞的事 —— 上一次命中时它会连着跑 1~4 次 DTW（合计毫秒级突发），
   * 再叠上 TTS/网络/LVGL 就会把这个 20ms 一次的录音回调拖垮。 */

  if (g_kws_enabled && kws_feed(data, frames) == 1)
    {
      g_kws_hit = true;
    }

  /* 送入声音检测器 */

  if (g_sound_started)
    {
      int ret = sound_detect_feed(&g_sound_ctx, data, frames);

      /* -EINVAL 表示"检测器当前不在收数据状态"：检测线程正把上一窗口拿去处理，
       * state 短暂变成 PROCESSING，这时喂数据是正常被拒，不是故障。
       * 原来每帧打一行，串口被刷屏，真日志全淹没了。 */
      if (ret < 0 && ret != -ENOSPC && ret != -EINVAL)
        {
          printf("[安全] 音频送入检测器失败: %d\n", ret);
        }
    }
}

/**
 * @brief  唤醒词命中后的处理（只在主循环里跑，每 100ms 一次）
 *
 * 录音线程只置 g_kws_hit，流程在这里推。为什么不在录音回调里直接
 * sm_handle_event(SM_EVENT_WAKEUP)：命中那一下要跑 DTW（毫秒级突发），
 * 而且我们这个应用里"喂帧"和"推流程"本来就各在一个线程，分开更清楚。
 *
 * 三路：
 *   1) 没模板（g_kws_enabled == false）：回调根本没喂过帧，走不到这里；
 *   2) 正忙（播报 / 送 ASR / 等大模型 / 追问流程在跑）：丢弃这次命中，
 *      不打断正在进行的事 —— 老人说话时喇叭在响（或刚响完）是常态，
 *      这时候插一脚会把当前那轮对话搅乱；
 *   3) 常听（没在送 ASR/等大模型/播报）：醒。
 *
 * 醒法就是 speech_capture_begin()，和"VAD 检测到有人说话"逐字相同的一段。
 *
 * ★ 唤醒词本身算不算要送给 ASR 的内容：**不算**，所以这里要把已经累积的
 *   帧清零（speech_capture_begin 第一行）。理由是 kws_feed 的命中时机：
 *   整句 DTW 必须等人说完，它是**尾静音 300ms 之后**才报命中的，那时候
 *   "你好，openvela"这句话已经一块块经过回调、躺在 g_speech_buf 里了。
 *   不清零就等于把唤醒词跟着后面的命令一起送去 ASR，大模型会以为用户
 *   说了一句叫它名字的话（今天没接线时就是这个行为：喊一句唤醒词就会
 *   触发一次"识别 + 回复"）。清零之后送 ASR 的只有命中之后的音频，
 *   也就是用户真正要说的话。
 *
 * 和录音线程的写竞争：g_speech_frames 这里（主循环）也在写，理论上可能和
 * 录音线程那一次 memcpy/累加交错，最坏结果只是"这次清零没完全生效"
 * （唤醒词残留一点尾巴）或"清零后又被加上一块"。不会越界：录音线程的
 * 写入量是按快照算过上限的（copy ≤ 上限 − 快照值）。为这点概率加锁会把
 * 每 20ms 一次的录音回调一起锁进去，不值。
 */

static void kws_wake_tick(sm_context_t *ctx)
{
  if (!g_kws_hit)
    {
      return;
    }

  g_kws_hit = false;

  if (!g_kws_enabled)
    {
      return;
    }

  /* 正忙就丢弃：判据和 listen_supervise_tick() 的"正忙"保持一致 */

  if (audio_is_playing(&g_audio_ctx) ||
      g_ask_phase != ASK_PHASE_IDLE ||
      sm_get_state(ctx) == SM_STATE_AI_TALKING)
    {
      printf("[KWS] 命中唤醒词，但当前正忙（AITalking/播报/追问），丢弃\n");
      return;
    }

  printf("[KWS] 唤醒词命中：从这一刻起收用户的话"
         "（丢掉已累积的 %zu 帧 = 唤醒词本身）\n", g_speech_frames);

  speech_capture_begin(ctx);
}

/**
 * @brief  初始化音频并启动VAD监听
 */

static int start_audio_listening(sm_context_t *ctx)
{
  int ret;

  /* 启用VAD检测 */

  audio_vad_enable(&g_audio_ctx, vad_callback, ctx);

  /* 开始录音 */

  audio_record_config_t record_cfg =
  {
    .enable_vad = true,
    .silence_timeout_ms = AUDIO_VAD_SILENCE_TIMEOUT_MS,
    .min_speech_ms = AUDIO_VAD_MIN_SPEECH_MS,
    .data_callback = audio_data_callback,
    .user_data = ctx
  };

  ret = audio_record_start(&g_audio_ctx, &record_cfg);
  if (ret < 0)
    {
      printf("[错误] 启动录音失败: %d\n", ret);
      return ret;
    }

  printf("[音频] 开始语音监听\n");
  g_audio_started = true;
  return OK;
}

/**
 * @brief  停止音频监听
 */

static void stop_audio_listening(void)
{
  /* 先关"要常听"的意图：主循环里的监听守护一直在看这个标志，
   * 不收掉的话退出流程刚把设备还回去，守护就把麦又打开了。 */

  g_listen_wanted = false;

  if (!g_audio_started)
    {
      return;
    }

  audio_record_stop(&g_audio_ctx);
  audio_vad_disable(&g_audio_ctx);
  g_audio_started = false;
  printf("[音频] 停止语音监听\n");
}

/****************************************************************************
 * 跨 app"让路"请求（薄壳在 ai_companion_yield.c）
 *
 * robot_ui 播提醒/提示音前调 ai_companion_audio_yield(true) 请我们让出麦克风，
 * 播完再调 ai_companion_mic_reclaim() 要回去。为什么需要它、为什么必须配对，
 * 写在 ai_companion_yield.h 里；这里只讲实现上的三个取舍：
 *
 *   1) **设备动作只由 hello_app 自己的线程发起**，就两条：让路线程
 *      （yield_worker_task，下面这一节）和监听守护（listen_supervise_tick，跑在
 *      主循环线程里）。薄壳那边一个设备都不碰，只登记请求方向。
 *      原来是 robot_ui 的播报线程替我们停（另一个 task group 调我们的
 *      stop_audio_listening → audio_record_stop → pthread_join 我们的录音线程 +
 *      动我们的设备 fd），一次 AUDIOIOC_STOP 报 ENOTTY、录音线程 300ms 没退出
 *      之后，整个 hello_app 组就死了 —— 详见 ai_companion_yield.h 头上那段
 *      串口原文。**同组内动自己的设备是安全的**，这也是那次事故后定下的原则。
 *      为什么专门起一条线程、而不是继续用主循环那一拍：主循环会被几十秒的阻塞
 *      调用占住（ASR 的 HTTPS / 追问里同步阻塞的 TTS），让路请求在那几十秒里
 *      根本没人受理 —— 详见让路线程那一节头上。
 *   2) "停"直接复用 stop_audio_listening()，不另写一套。那个函数除了
 *      audio_record_stop()（等录音线程收摊，回收上限 300ms）之外，还会把
 *      g_listen_wanted 收掉 —— 这正是让监听守护在让路期间**别把麦抢回来**的
 *      关键：守护只看这个意图标志，一看是 false 就整个不动作。
 *   3) "收回"走和监听守护**同一段**开麦逻辑（start_audio_listening()），
 *      外加守护重开前那套清场（g_speech_capturing / g_speech_frames /
 *      KWS 流式状态）。不共用的话两边迟早会漂移：比如守护那边记得清 KWS
 *      的自适应本底、这边忘了清，下一次听就带着上一段音频的本底起判。
 *   4) 换线程之后多了一件原来不用想的事：**主循环那条线程也可能正在用同一个
 *      音频设备**（TTS 出声：audio_play_start() 内部会停录音、播放线程收尾时
 *      又会把录音重开）。所以让路线程动设备之前有两道闸（mic_audio_busy() +
 *      g_mic_device_lock），监听守护那边也加了一道（它重开麦时先抢同一把锁）。
 ****************************************************************************/

/* 让路失败的兜底上限（毫秒）：认领请求之后等这么久，麦克风还没真正回到驱动手上
 * （录音线程仍卡在 read() 里 / AUDIO_IN: AUDIOIOC_STOP 报错），就只打一行日志、
 * **不再做任何设备动作**。这条线不是"超时重试"，而是"到此为止"：
 *
 *   - 为什么取 1500ms：和调用方的轮询上限（robot_ui 的 REMINDER_YIELD_WAIT_MS
 *     = 1500ms）对齐。调用方等不到让路就会自己出声，串口上那时应该已经有一行
 *     说明"为什么没让成"，出问题时一眼能对上；
 *   - 为什么不补救（重发 STOP / close / 往录音线程上再 join 一次）：那正是这次
 *     事故的成因。设备停不下来是驱动的事（任务 B 修 sf32lb52_audio），应用层硬来
 *     只会把"线程阻塞在 read 里、fd 状态不明"的残局搅得更乱；
 *   - 为什么不重试：让路是"来不及就拉倒"的事，调用方有自己的超时，重试只会把
 *     让路的窗口拖长，反而更可能撞上它开始出声的那一刻。 */

#define MIC_RELEASE_DEADLINE_MS  1500

/* 让路线程两拍之间最多等这么久（毫秒）。它就是"让路被受理"的时延上限：
 * 主循环被 ASR/TTS 阻塞几十秒也一样是这个数，所以**不许调大**（原来的做法是
 * 等主循环那一拍，同样 100ms —— 换线程只是把"等"从别处挪到自己身上）。
 * 等于 MAIN_LOOP_INTERVAL_MS 是巧合也是刻意：两种做法在"没被阻塞"时的让路
 * 时延一样，换线程只赢在"主循环被阻塞时"这一种情况。 */

#define YIELD_WORKER_WAIT_MS     100

/* "主正在出声"的收尾余量（毫秒）：见 mic_audio_busy() 的说明。
 * 500ms 的来历：audio_resume_record() 内部只有"开设备 + 起线程"两步（没有会空等
 * 的循环），正常几毫秒；500ms 是给设备慢启动/调度抖动留的余量，同时短于调用方
 * 那 1500ms 的耐心（让路最多因为它晚 500ms 受理）。 */

#define MIC_PLAY_TAIL_GRACE_MS   500

/* 让路请求被推迟的上限（毫秒）：见 yield_worker_task() 里那段。
 * 3000ms = 调用方轮询上限（1500ms）的 2 倍：超过它，这次提示音早就自己试过了，
 * 再推迟下去就只是"功能整个失效"，所以照常按让路处理。 */

#define YIELD_DEFER_MAX_MS       3000

/* 让路的处理状态。换线程之后写者是**让路线程**（yield_worker_task；让路线程起不
 * 来时才是 main_loop_task 的 mic_hold_tick()），别的线程只读 —— 全 volatile，
 * 给 ai_companion_mic_released() 那条跨 app 的读路径和监听守护用：
 *   g_mic_hold_active         —— 让路请求已被认领（正在让路 / 已让出），
 *                                也是"让路期间不许重开麦"的那道闸
 *   g_mic_hold_at             —— 认领那一刻的时间戳（算上面那个 1500ms 用）
 *   g_mic_hold_release_failed —— 兜底已经判过并打过日志了，别重复刷屏
 *   g_mic_device_busy         —— 认领者正拿着设备开关（impl 正在跑）。查询函数
 *                                见到它就报"还没让出"（宁可比真的晚一点点，
 *                                也不能让调用方以为设备空了） */

static volatile bool     g_mic_hold_active;
static volatile uint32_t g_mic_hold_at;
static volatile bool     g_mic_hold_release_failed;
static volatile bool     g_mic_device_busy;

/* ---- 让路线程的共享量 ----
 *
 * g_yield_worker_up       —— 线程在跑（mic_hold_tick 靠它决定"只唤醒"还是
 *                            "退回老做法自己动手"）。由创建者置 true、线程收尾时置 false
 * g_yield_worker_created  —— 成功创建过（退出时 join 用；只有 main() 读写）
 * g_mic_req_handled       —— 已经受理过的请求方向（电平语义：和
 *                            ai_companion_mic_request() 比一比就知道是不是新请求）
 * g_mic_req_pending_at    —— 第一次看到这个请求还没受理的时刻（算推迟上限用）
 * g_mic_play_seen_ms      —— 最近一次看到"主正在出声"的时刻（算收尾余量用）
 * g_mic_hold_defer_logged —— "推迟受理"的日志只打一行，别每 100ms 刷一次 */

static pthread_t         g_yield_worker;
static sem_t             g_yield_worker_sem;
static volatile bool     g_yield_worker_up;
static bool              g_yield_worker_created;
static volatile int      g_mic_req_handled = AI_COMPANION_MIC_REQ_NONE;
static volatile uint32_t g_mic_req_pending_at;
static uint32_t          g_mic_play_seen_ms;
static volatile bool     g_mic_hold_defer_logged;

/* "设备动作权"：hello_app 里所有会开关录音设备的动作串起来用的一把锁。
 *
 * 换线程之后，主循环那条线程虽然可能被阻塞，但它不阻塞的时候可能正在起播 —— 
 * audio_play_start() 内部会做半双工让路（audio_prepare_output → 
 * audio_record_stop：停设备 + 回收录音线程）。让路线程这一刻要是也在 
 * audio_record_stop / audio_record_start，就是"同一个录音线程被 join 两次"/
 * "两条录音线程抢一个设备"——正是要避开的形状。这一小段（起播调用本身，
 * 不是整段出声）用锁串起来就够；**整段出声**那几十秒靠 mic_audio_busy() 推迟，
 * 见 yield_worker_task()。
 *
 * 静态初始化（mimo_location.c 是同一套写法）：不依赖任何初始化时机 —— 
 * 让路线程起不来（pthread_create 失败）时这把锁也照样有效，只是没人跟它抢。
 * 另外它**只保护 hello_app 自己发起的动作**：ai_audio 播放线程收尾时的 
 * audio_resume_record() 在库里，改不到（见 mic_audio_busy 的收尾余量）。 */

static pthread_mutex_t   g_mic_device_lock = PTHREAD_MUTEX_INITIALIZER;

/**
 * @brief  麦克风到底还在不在 hello_app 手里（ai_companion_mic_released 的判据）
 *
 * 三条**设备状态**判据，缺一不可（为什么不用 g_audio_started / ctx->recording
 * 单独下结论见下）：
 *   1) g_audio_started —— "我还想常听"的意图还在（开机开麦失败时它可能是 false，
 *      而设备其实也没在我们手里，所以它只能当"是"的证据，不能当"否"的证据）；
 *   2) audio_record_is_active() —— 设备 START 着、录音线程还活着、没人喊停
 *      （ai_audio.h 里那张说明表）；它看不穿驱动，所以还要第三条；
 *   3) 录音线程真的收尾了（record_thread_valid == false 或者 record_exited
 *      == true）。**这条是专为那次事故加的**：AUDIOIOC_STOP 失败、录音线程
 *      30 次 10ms 轮询后仍没退出时，record_stop / recording 都已经被置位
 *      （audio_record_stop 的收尾），前两条会一致地报"没在录"——可那一刻线程正是
 *      阻塞在 read() 里、设备状态不明，最不该说"设备空了"。
 * 注意这三个字段都只有 hello_app 自己（让路线程 / 主循环 / 录音线程 / 监听守护）
 * 会改，外面读到的可能是慢一拍的旧值；对"再等 50ms 看看"这种用途足够。
 */

static bool mic_still_held(void)
{
  if (!g_audio_ctx.initialized)
    {
      /* 音频都没初始化：设备不可能是我们占着的（还没起来 / 已经收摊）。 */

      return false;
    }

  if (g_audio_started)
    {
      return true;
    }

  if (audio_record_is_active(&g_audio_ctx))
    {
      return true;
    }

  if (g_audio_ctx.record_thread_valid && !g_audio_ctx.record_exited)
    {
      /* 线程 TCB 还挂着、而线程自己还没跑到最后一行：它可能正卡在 read() 里，
       * 设备到底有没有回到驱动手上说不清 —— 按"还在我们手里"算。 */

      return true;
    }

  return false;
}

/**
 * @brief  麦克风现在是否真的不在 hello_app 手里了（1 = 调用方可以放心出声）
 *
 * 跨 app 查询入口，robot_ui 的播报线程轮询它（判据写在 ai_companion_yield.h
 * 的声明处，改判据时两处要一起改）。它只读状态，不碰任何设备。
 */

int ai_companion_mic_released(void)
{
  if (!g_audio_ctx.initialized)
    {
      /* 音频没初始化 —— 包括"robot_ui 比 ai_companion 先起来"这种正常情况。
       * 这时候没人占着麦克风，直接报"已让出"，别让调用方白等 1.5 秒。 */

      return 1;
    }

  if (g_mic_device_busy)
    {
      /* 认领者（让路线程）正拿着设备开关：停常开监听 / 重开麦还在跑。这一刻的
       * 设备状态是"半程"（比如音频层的 recording 已经置回 false、录音线程却还
       * 卡在 read 里），只有它那一手做完才算数 —— 一律按"还没让出"报。
       * 这条是换线程后新加的：判据只许更严，不许更松。 */

      return 0;
    }

  if (!g_mic_hold_active)
    {
      /* 让路请求还没被认领（让路线程最多晚 YIELD_WORKER_WAIT_MS=100ms；
       * 主正在出声时会更晚，见 yield_worker_task）。在认领者动设备之前，
       * 设备还在我们手里 —— 报"没让出"，让调用方接着轮询。 */

      return 0;
    }

  return mic_still_held() ? 0 : 1;
}

void ai_companion_listen_hold_impl(bool hold)
{
  int ret;

  if (hold)
    {
      /* 让路。注意 g_audio_started 与 g_listen_wanted 是两件事：
       * 开机第一次开麦失败时（设备忙 / 驱动没起来）前者假、后者真，
       * 那种情况下也必须把意图收掉，否则监听守护会一直重试着跟我们抢设备。 */

      if (!g_audio_started && !g_listen_wanted)
        {
          printf("[让路] 麦克风本来就不在手里，空转（ai_companion 没在跑？）\n");
          return;
        }

      bool was_started = g_audio_started;

      stop_audio_listening();

      if (was_started)
        {
          printf("[让路] 麦克风已交出：常开监听已停（设备已还回去，等调用方出声）\n");
        }
      else
        {
          printf("[让路] 麦克风本来就没开着，只把\"要常听\"的意图收掉\n");
        }

      return;
    }

  /* 收回。先把"要常听"的意图立起来，再说别的：哪怕下面这次重开失败、或者
   * ai_companion 现在根本没在跑，意图留着，监听守护（或者之后的开机流程）
   * 也会自己把麦开起来。 */

  g_listen_wanted = true;

  if (!g_running || !g_audio_ctx.initialized)
    {
      printf("[让路] ai_companion 没在跑（音频还没初始化 / 正在退出），"
             "只记下\"要常听\"的意图，不碰设备\n");
      return;
    }

  if (audio_record_is_active(&g_audio_ctx))
    {
      printf("[让路] 麦克风本来就在手里（常开监听是活的），不重复开麦\n");
      return;
    }

  /* 重开前清场：让路期间/上一次会话可能死在"人正说话"中间，g_speech_capturing
   * 还挂着 true、g_speech_buf 里有半截累积语音，不清掉下一次进 ASR 的会是跨会话
   * 的脏数据。此刻录音线程已经不在（上面刚判过不活跃），不会有回调往里写。
   * 和监听守护那段清场逐字相同，改动时两处要一起看。 */

  g_speech_capturing = false;
  g_speech_frames = 0;
  g_audio_ctx.record_died = false;    /* 让路期间攒下的"异常中断"到这里作废 */

  if (g_kws_enabled)
    {
      kws_reset();
    }

  ret = start_audio_listening(&g_sm_ctx);
  if (ret < 0)
    {
      printf("[让路] 收回麦克风失败: %d（监听守护会接着重试）\n", ret);
      return;
    }

  printf("[让路] 麦克风已收回：常开监听已重开\n");
}

/**
 * @brief  起播（和让路线程的设备动作串起来，见 g_mic_device_lock）
 *
 * 为什么要包一层：audio_play_start() 内部会做半双工让路
 * （audio_prepare_output → audio_record_stop：停设备 + 回收录音线程），而让路
 * 线程也会调 audio_record_stop / start_audio_listening。两边同时上手就是"同一个
 * 录音线程被 join 两次"或者"两条录音线程抢一个设备"。
 * 这一小段（起播调用本身）拿同一把锁串起来即可 —— 起播很快（校验 + memcpy +
 * 起线程），让路线程等它一下不心疼。**整段出声**那几十秒不用锁，靠
 * mic_audio_busy() 让让路线程自己推迟。
 */

static int audio_play_start_locked(audio_context_t *ctx, const int16_t *data,
                                   size_t frames,
                                   audio_play_complete_cb_t callback,
                                   void *user_data)
{
  int ret;

  pthread_mutex_lock(&g_mic_device_lock);
  ret = audio_play_start(ctx, data, frames, callback, user_data);
  pthread_mutex_unlock(&g_mic_device_lock);

  return ret;
}

/**
 * @brief  hello_app 现在（或刚刚）是不是正在用音频设备出声
 *
 * 给让路线程当闸用：真的时候不许动设备，下一拍再看。两条判据 + 一段余量：
 *   1) audio_is_playing()：播放线程正在跑。出声期间录音本来就是停的（半双工），
 *      此刻让路线程去"停常开监听"只会把 record_stop 置位、g_audio_started 清掉，
 *      可播放线程收尾时会按 record_resume_on_play_end 把录音**重新开起来** ——
 *      结果是"让了路、麦却自己回来了"，正是提示音被挤掉的老毛病。
 *   2) 刚看到过上面这条（MIC_PLAY_TAIL_GRACE_MS 之内）：ai_audio 的播放线程把
 *      ctx->playing 置 false 之后**还要**调 audio_resume_record()（重开录音）
 *      和播放完成回调，那一段它仍然在动录音设备（见 ai_audio.c 播放线程末尾）。
 *      只看第 1 条会以为"放完了、安全了"，正好撞上它开麦 —— 那一下就是两条录音
 *      线程抢同一个设备。所以看到过就再等一段余量，余量大小的来历见
 *      MIC_PLAY_TAIL_GRACE_MS。
 *
 * 被推迟的这段时间设备本来就在 hello_app 自己手里（它正在出声），提示音无论如何
 * 都挤不进来 —— 这是半双工设备层面的取舍，ai_companion_yield.h 里已写明本接口
 * 只管麦克风这一半。等它放完，让路照常受理（还有推迟上限兜底，见
 * YIELD_DEFER_MAX_MS）。
 */

static bool mic_audio_busy(uint32_t now)
{
  if (audio_is_playing(&g_audio_ctx))
    {
      /* 记下这一刻：接下来那一段也算"刚出声"（第 2 条判据）。
       * 0 是保留值（表示还没见过），真拿到 0 就退一格用 1，代价是那一轮少算 1ms。 */

      g_mic_play_seen_ms = (now != 0) ? now : 1;
      return true;
    }

  if (g_mic_play_seen_ms != 0 &&
      (int32_t)(now - g_mic_play_seen_ms) < (int32_t)MIC_PLAY_TAIL_GRACE_MS)
    {
      return true;
    }

  return false;
}

/**
 * @brief  按请求方向动一次设备（**只有 hello_app 自己的线程能调**）
 *
 * 停/开常开麦克风就在下面两处 ai_companion_listen_hold_impl()：
 *   - YIELD：先置 g_mic_hold_active —— 查询函数那边马上知道"让路已被受理"，
 *     同时监听守护那边有 g_listen_wanted == false（stop_audio_listening 收掉的）
 *     兜着，让路期间不会有人把麦抢回来；然后停一次常开监听
 *     （stop_audio_listening，等录音线程收摊的上限 300ms）。**只停这一次**，
 *     成没成都不再重复敲设备（有界兜底见 mic_hold_deadline_check）。
 *   - RECLAIM：清 g_mic_hold_active，再走 start_audio_listening() 重开（立回
 *     "要常听"的意图、清场、开麦）。重开失败不在这里重试，交给监听守护按退避来。
 *
 * 电平语义：请求是"最后一次请求的方向"，不是事件。所以重复的 YIELD 只会被当成
 * "还在让着"（g_mic_hold_active 已经是 true），不会再去停一次设备；而收到
 * RECLAIM 但本来就没让过路时（g_mic_hold_active == false）一个动作都不做。
 *
 * ⚠️ 本函数会动设备（stop_audio_listening / audio_record_stop / 
 * start_audio_listening）。别处 —— 尤其是别的 task group —— 一律不许调它们：
 * 理由见 ai_companion_yield.h 头上那次 hello_app 整个组死掉的事故。
 */

static void mic_hold_apply(int req)
{
  /* 日志里标出这一手是谁做的：正常是让路线程，让路线程起不来时退回主循环 ——
   * 看串口就知道走的是哪条路（验收时要用）。 */

  const char *who = g_yield_worker_up ? "让路线程"
                                      : "主循环线程：让路线程没起来";

  if (req == AI_COMPANION_MIC_REQ_RECLAIM)
    {
      if (!g_mic_hold_active)
        {
          /* 本来就没让过路（robot_ui 可能没登记成功 / 已经收回了）：没东西可收，
           * 只把"这个方向受理过了"记下来。 */

          g_mic_req_handled = req;
          return;
        }

      g_mic_hold_active = false;
      g_mic_hold_release_failed = false;
      g_mic_req_handled = req;

      printf("[让路] 收到收回请求：恢复\"要常听\"的意图，在【%s】里重开常开监听\n",
             who);

      ai_companion_listen_hold_impl(false);
      return;
    }

  if (req != AI_COMPANION_MIC_REQ_YIELD)
    {
      /* NONE：没人请求过（只有开机头一次可能读到），记下就是了。 */

      g_mic_req_handled = req;
      return;
    }

  if (g_mic_hold_active)
    {
      /* 还在让着：不重复停设备，只记下这个方向已经受理。 */

      g_mic_req_handled = req;
      return;
    }

  g_mic_hold_active = true;
  g_mic_hold_at = main_now_ms();
  g_mic_hold_release_failed = false;
  g_mic_req_handled = req;

  printf("[让路] 收到让路请求（robot_ui 要出声）：在【%s】里停常开监听"
         "（只停这一次）\n", who);

  ai_companion_listen_hold_impl(true);
}

/**
 * @brief  在"设备动作权"里受理一次请求；主正在出声就放弃（不认领，下一拍再来）
 *
 * 两道闸合起来保证"同一时刻只有一个线程在动音频设备"：
 *   - 先拿 g_mic_device_lock：主循环里"起播"那一小段也拿同一把锁，
 *     所以这里一旦拿到，audio_prepare_output 的停录音不可能同时在跑；
 *   - 拿到锁**再判一次** mic_audio_busy()：等锁的这段时间里主可能刚起了一段
 *     播放（那把锁只保护起播调用，保护不了整段出声），真在出声就放弃 —— 
 *     设备这会儿在它自己手里，让路等放完再受理（force = 推迟超过上限时越过
 *     这道闸，避免"一直推迟"把整个功能拖死，理由见 YIELD_DEFER_MAX_MS）。
 *
 * @param  req   请求方向
 * @param  force true = 不管主在不在出声都动手（推迟上限到了）
 * @return true = 已受理（g_mic_req_handled 跟上）；false = 现在不能动设备
 */

static bool mic_hold_apply_serialized(int req, bool force)
{
  pthread_mutex_lock(&g_mic_device_lock);

  if (!force && mic_audio_busy(main_now_ms()))
    {
      pthread_mutex_unlock(&g_mic_device_lock);
      return false;
    }

  g_mic_device_busy = true;    /* 查询函数见到它就报"还没让出"（判据只许更严） */
  mic_hold_apply(req);
  g_mic_device_busy = false;

  pthread_mutex_unlock(&g_mic_device_lock);
  return true;
}

/**
 * @brief  让路失败的兜底判定（只打一行日志，一个设备动作都不做）
 *
 * 认领让路之后 MIC_RELEASE_DEADLINE_MS（1.5s）麦克风还没真正回到驱动手上
 * （录音线程仍卡在 read() 里 / AUDIO_IN: AUDIOIOC_STOP 报错），就只打一行日志。
 * 这条线不是"超时重试"，而是"到此为止"：
 *
 *   - 为什么取 1500ms：和调用方的轮询上限（robot_ui 的 REMINDER_YIELD_WAIT_MS
 *     = 1500ms）对齐。调用方等不到让路就会自己出声，串口上那时应该已经有一行
 *     说明"为什么没让成"，出问题时一眼能对上；
 *   - 为什么不补救（重发 STOP / close / 往录音线程上再 join 一次）：那正是这次
 *     事故的成因。设备停不下来是驱动的事，应用层硬来只会把"线程阻塞在 read 里、
 *     fd 状态不明"的残局搅得更乱；
 *   - 为什么不重试：让路是"来不及就拉倒"的事，调用方有自己的超时，重试只会把
 *     让路的窗口拖长，反而更可能撞上它开始出声的那一刻。
 *
 * 判定放在让路线程里跑（每 100ms 一次，节拍和原来的主循环一样）：主循环被
 * ASR/TTS 阻塞也不会漏判。g_mic_hold_at / g_mic_hold_active 的写者从主循环换成
 * 了让路线程，读的人是这里和 ai_companion_mic_released()，看到的都是单个
 * volatile 量，语义没变。
 */

static void mic_hold_deadline_check(void)
{
  uint32_t now;

  if (!g_mic_hold_active || g_mic_hold_release_failed || !mic_still_held())
    {
      return;
    }

  now = main_now_ms();
  if ((int32_t)(now - g_mic_hold_at) < (int32_t)MIC_RELEASE_DEADLINE_MS)
    {
      return;
    }

  g_mic_hold_release_failed = true;

  printf("[让路] 让路失败：请求后 %u ms 麦克风还在我们手里"
         "（录音线程没退出 / AUDIO_IN: AUDIOIOC_STOP 这类失败）。"
         "不再做任何设备动作，这次播报直接进行\n",
         (unsigned)(now - g_mic_hold_at));
}

/**
 * @brief  等一次让路线程的唤醒（最多 YIELD_WORKER_WAIT_MS 毫秒）
 *
 * 为什么是"带超时的等"而不是死等 sem_wait()：信号量只是**快路径** —— 
 * mic_hold_tick() 看到新请求就 post 一下，让让路立刻被受理；可**让路能不能被
 * 及时受理绝不靠它**：主循环被 ASR/TTS 占住时没人来 post，靠的就是这个超时把
 * 本线程叫醒、自己去读一次请求电平。所以这个超时值就是方案对外的时延上限。
 *
 * 时钟被校时（SNTP）跳坏不会让请求丢掉：sem_post 一进来 sem_timedwait 立刻返回
 * （它有"已挂着的信号量先拿"这条语义），绝对时间只影响空转时下一次醒来的时刻 ——
 * 最坏是早醒/晚醒一会儿，不会漏请求。
 */

static void yield_worker_wait(void)
{
  struct timespec ts;
  int ret;

  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_nsec += (long)YIELD_WORKER_WAIT_MS * 1000000L;
  ts.tv_sec  += ts.tv_nsec / 1000000000L;
  ts.tv_nsec %= 1000000000L;

  do
    {
      ret = sem_timedwait(&g_yield_worker_sem, &ts);
    }
  while (ret < 0 && errno == EINTR);
}

/**
 * @brief  让路线程：受理 robot_ui 的"让路 / 收回"请求
 *
 * ★ 这条线程存在的唯一理由：原来的受理点是主循环每 100ms 的 mic_hold_tick()，
 *   可主循环这条线程会被**几十秒**的阻塞调用占住 —— process_ai_dialogue() 里
 *   voice_asr_recognize() 是阻塞 HTTPS，追问流程 ask_speak() 里的
 *   voice_tts_speak() 同理（那边的注释自己写着"阻塞几秒是可以接受的"）。于是
 *   "老人附近刚好有异响触发追问 / 正在对话"的那几十秒里，让路请求根本没人受理
 *   → robot_ui 等满 1.5 秒 → 提示音被驱动拒掉 → 铃不响。
 *
 * 它只做三件事：等唤醒 → 按请求方向动一次设备（mic_hold_apply）→ 更新状态
 * （g_mic_hold_active / g_mic_hold_at / g_mic_device_busy）。不碰 LVGL、
 * 不碰状态机、不碰 KWS/唤醒词/灯控/追问判定。
 *
 * 它和主循环同属 hello_app 这个 task group（同组内动自己的设备是安全的，这是
 * 那次事故后定下的原则），线程属性沿用本文件既有写法（pthread_create 传 NULL =
 * CONFIG_PTHREAD_STACK_DEFAULT；本文件的 main_loop_task、ai_audio.c 的录/放
 * 线程都是这么起的，默认栈 16KB，够它这几步用）。
 *
 * 主正在出声（TTS 播放 + 它的收尾）时**不动设备**，推迟到下一拍再看：
 * 见 mic_audio_busy() 的说明。推迟有上限（YIELD_DEFER_MAX_MS），到点照常处理 —— 
 * 不然万一"正在出声"这个状态卡住了，让路就整个失效（还不如被拖住）。
 *
 * 线程退出：g_running 被置 0（main() 收尾）后最多 YIELD_WORKER_WAIT_MS 醒一次就
 * 退出；要是正动设备，会先把那一手做完（main() 那边 join 它，见 yield_worker_stop）。
 */

static void *yield_worker_task(void *arg)
{
  (void)arg;

  printf("[让路] 让路线程已启动：让路不再等主循环，主循环被 ASR/TTS 阻塞几十秒"
         "也照样受理（两拍间隔上限 %d ms）\n", YIELD_WORKER_WAIT_MS);

  while (g_running)
    {
      int      req = ai_companion_mic_request();
      uint32_t now = main_now_ms();

      if (req == g_mic_req_handled)
        {
          g_mic_req_pending_at = 0;
          g_mic_hold_defer_logged = false;
        }
      else
        {
          bool busy;
          bool force = false;

          if (g_mic_req_pending_at == 0)
            {
              g_mic_req_pending_at = (now != 0) ? now : 1;
            }

          busy = mic_audio_busy(now);

          if (busy &&
              (int32_t)(now - g_mic_req_pending_at) >=
              (int32_t)YIELD_DEFER_MAX_MS)
            {
              /* 推迟太久了：可能"正在出声"这个状态本身卡住了。再推迟下去让路
               * 就整个失效，还不如照常处理（这一刻动设备的风险见
               * mic_hold_apply_serialized 里 force 那一句）。 */

              force = true;
              printf("[让路] 让路请求推迟超过 %u ms（主一直\"在出声\"？），"
                     "照常按让路处理\n", (unsigned)YIELD_DEFER_MAX_MS);
            }
          else if (busy)
            {
              /* 日志只打一行：推迟期间每 100ms 会走到这里一次，别刷屏。 */

              if (!g_mic_hold_defer_logged)
                {
                  g_mic_hold_defer_logged = true;
                  printf("[让路] 让路请求推迟：主正在出声（TTS 播放中 / 刚放完），"
                         "放完再动设备\n");
                }
            }

          if (force || !busy)
            {
              if (mic_hold_apply_serialized(req, force))
                {
                  g_mic_req_pending_at = 0;
                  g_mic_hold_defer_logged = false;
                }
            }
        }

      mic_hold_deadline_check();

      yield_worker_wait();
    }

  g_yield_worker_up = false;
  printf("[让路] 让路线程退出\n");
  return NULL;
}

/**
 * @brief  起让路线程
 * @return true = 已起来；false = 起不来（让路退回主循环处理）
 *
 * 顺序很关键：sem_init + "线程已就绪"这两个动作必须在**创建主循环线程之前**
 * 完成（调用点在 main() 里就是按这个顺序排的）——
 *   - 请求比这里早（robot_ui 先起来、先登记了让路）：线程起来后第一件事就是读
 *     一次请求电平，照样受理；
 *   - 请求比这里晚：mic_hold_tick() 已经在"只唤醒"那条路上（g_yield_worker_up
 *     为真），post 一下信号量就够；
 *   两头都不落在缝里，所以不需要"额外再查一次"。反过来说，g_yield_worker_up
 *   一旦为真就不会再有"主循环自己动手"的路径。
 *
 * 失败（sem_init / pthread_create 出错）就如实返回 false 并打日志：
 * mic_hold_tick() 会退回**原来的做法**（在主循环线程里处理请求）。功能不残，
 * 代价是重新会被主循环的阻塞调用拖住 —— 宁可这样，也不能让让路整个失效。
 */

static bool yield_worker_start(void)
{
  int ret;

  ret = sem_init(&g_yield_worker_sem, 0, 0);
  if (ret < 0)
    {
      printf("[让路] 让路线程起不来（sem_init: %d），让路退回主循环处理"
             "（主循环被 ASR/TTS 拖住时，提示音可能又等不到让路）\n", errno);
      return false;
    }

  ret = pthread_create(&g_yield_worker, NULL, yield_worker_task, NULL);
  if (ret != 0)
    {
      printf("[让路] 让路线程起不来（pthread_create: %d），让路退回主循环处理"
             "（主循环被 ASR/TTS 拖住时，提示音可能又等不到让路）\n", ret);
      sem_destroy(&g_yield_worker_sem);
      return false;
    }

  g_yield_worker_created = true;
  g_yield_worker_up = true;
  return true;
}

/**
 * @brief  收让路线程（退出流程里调，必须在主线程停音频设备之前）
 *
 * g_running 已经是 0（调用点保证），它最多 YIELD_WORKER_WAIT_MS 就醒过来退出。
 * **必须等它退干净**再走下面的 stop_audio_listening()：万一它正在动录音设备，
 * 两边同时上手就是这次事故的形状（跨线程同时开关同一个设备）。
 */

static void yield_worker_stop(void)
{
  if (!g_yield_worker_created)
    {
      return;
    }

  pthread_join(g_yield_worker, NULL);
  g_yield_worker_created = false;
  g_yield_worker_up = false;
}

/**
 * @brief  让路请求心跳（main_loop_task 每 100ms 调一次）
 *
 * ★ 让路线程在跑（正常情况）：本线程**一个设备动作都不做** —— 看到新请求就
 *   post 一下信号量（让路立刻被受理的快路径），然后返回。设备动作全在让路线程
 *   里做，所以主循环被 ASR/TTS 阻塞几十秒也不影响让路。post 丢了也不打紧：
 *   让路线程自己那 100ms 的超时会把请求读到。
 *
 * 让路线程起不来（sem_init / pthread_create 失败）：退回**原来的做法** ——
 * 在本线程里按电平处理请求（mic_hold_apply）+ 有界兜底（mic_hold_deadline_check）。
 * 功能一样，只是重新会被主循环的阻塞调用拖住。这里不打日志，理由是启动时
 * yield_worker_start() 已经如实报过"起不来 + 退回主循环"了。
 */

static void mic_hold_tick(void)
{
  int req = ai_companion_mic_request();

  if (g_yield_worker_up)
    {
      if (req != g_mic_req_handled)
        {
          (void)sem_post(&g_yield_worker_sem);
        }

      return;
    }

  if (req != g_mic_req_handled)
    {
      mic_hold_apply(req);
    }

  mic_hold_deadline_check();
}

/****************************************************************************
 * 常听监听守护（自愈）
 *
 * start_audio_listening() 原来只在开机时调一次，录音线程一旦因为驱动
 * AUDIOIOC_STOP 或下层 5 秒 DMA 超时（audio_in_read 返回 0 / -110）自己
 * 跳出循环，应用就永久变聋：还在跑、界面还在，但再也听不到声音，只能重启板子。
 * 下面这段在主循环里每 100ms 看一眼"录音是不是还活着"，不活就按退避重启。
 ****************************************************************************/

/* 第一次重试前的等待：给"线程自己收尾 / 驱动把设备还回来"留时间。
 * 立刻重开的话 audio_record_start() 会撞上还没退出的旧线程（返回 -EBUSY）。 */

#define LISTEN_SUPERVISE_RETRY_MS   1800

/* 连续失败时的退避上限：设备真坏了就别 1.8 秒一次地去敲它 */

#define LISTEN_SUPERVISE_MAX_MS    10000

/* "忙"的兜底上限（毫秒）：录音已经断了这么久、而那几个"忙"标志还挂着，
 * 就不再认"忙"，强行把麦克风抢回来（见 listen_supervise_tick 里那一段）。
 *
 * 取 45 秒，两个方向都要交代：
 *   - 不能比"正常的忙"短：正常流程里唯一会让录音停下来的事是 TTS 出声
 *     （半双工让路，见 ai_audio.c 的 audio_prepare_output），而单次出声的上限
 *     就是播放缓冲 AUDIO_PLAY_BUFFER_MS = 30 秒；45 秒 = 30 秒 + 15 秒余量，
 *     正常出声绝不会被它掐掉。状态机 AI_TALKING 自己的超时也是 30 秒，
 *     45 秒是它的 1.5 倍，够一次正常回复先跑完。
 *   - 又不能太长：真正会挂住的是**标志本身卡死**（播放线程写不动、追问相位
 *     没收尾、大模型回复永远不来），它们没有任何上限，全靠这个数兜底。
 *     45 秒是"老人还能忍"和"别误伤正常流程"之间的取舍：喊了半天没反应之后，
 *     45 秒内一定能把耳朵抢回来。
 * 注意它计的是"**失去麦克风**的时长"，不是"推理时长"：上面那句
 * audio_record_is_active() 一命中就清零，正常推理期间根本不会累加 ——
 * 这也正是它敢取这么温和的值的原因。 */

#define LISTEN_SUPERVISE_FORCE_MS  45000

/* 失败日志节流：前 5 次每次都打（开头几次最需要看见），之后每 10 次一条 */

#define LISTEN_SUPERVISE_LOG_FIRST      5
#define LISTEN_SUPERVISE_LOG_EVERY     10

static bool     g_listen_retry_armed;      /* 已经排好一次重试，等着到点 */
static uint32_t g_listen_retry_at;         /* 下次重试的时间点 (main_now_ms) */
static uint32_t g_listen_fail_count;       /* 连续失败次数（退避 + 日志节流） */
static uint32_t g_listen_backoff_ms = LISTEN_SUPERVISE_RETRY_MS;
static uint32_t g_listen_dead_since;       /* 从哪一刻起听不见了，0 = 还没断 */
static uint32_t g_listen_last_try_ms;      /* 上次重开录音的时间（只在失败退避时有用） */

/**
 * @brief  监听守护：录音不活跃就按退避把它重新拉起来
 *
 * 只在"应该常听"的时候动手。本板录放是半双工（AUDIOIOC_STOP 会把两条通路
 * 一起停），下面任何一条成立时去开麦都是跟正在跑的那条流程抢设备：
 *   - 正在收尾：g_running / g_listen_wanted（stop_audio_listening 刚关了麦）；
 *   - 正把麦克风让给别的 app：g_mic_hold_active（robot_ui 播提醒/提示音期间，
 *     hello_app 必须保持安静，见 mic_hold_apply）；
 *   - 让路线程正拿着设备开关：g_mic_device_busy（它可能在重开麦那一手中间，
 *     见 mic_hold_apply_serialized）；
 *   - 扬声器在响：TTS 出声期间 audio_play_start() 特意先把录音停掉让路，
 *     播完由播放线程自己按原配置恢复，这里插一脚必然互相打断；
 *   - 状态机在 AI_TALKING：送 ASR / 等大模型 / TTS 合成这一整段；
 *   - 追问流程在跑（说 / 限时听）：它有自己的听说节奏，只听它自己的窗口。
 *
 * 注意 g_speech_capturing 不能当"忙"来跳过重试：录音线程完全可能在 VAD
 * 报过"人开始说话"之后才死，这个标志就永远挂在 true 上，拿它挡重试等于
 * 把"永久变聋"原样搬回来。所以这里改成重开时顺手清掉它和累积长度。
 *
 * ★ 本函数存在的意义就在"忙也得有上限"这一条（LISTEN_SUPERVISE_FORCE_MS）：
 *   上面那几个"忙"判据都是**别人维护的标志**，任何一个卡住不回（播放线程写不动、
 *   追问相位没收尾、大模型回复永远不来），录音断了也没人来救 —— "忙 + 会话已死"
 *   叠加起来就是用户口中的"卡死了、说话没反应"。所以录音断了够久之后，
 *   连"忙"都不认，强行重开。另外还有一条更快的路：录音线程自己报的
 *   ctx->record_died（异常中断）不等任何时间 —— 只要是"下一拍"就能重开
 *   （只留一道"两次重开至少隔 1.8 秒"的防抖，防止设备一起来就死时高频开关设备）。
 */

static void listen_supervise_tick(sm_context_t *ctx)
{
  uint32_t now;
  bool     died;
  bool     stale;
  bool     urgent;
  int ret;

  if (!g_running || !g_listen_wanted || g_mic_hold_active || g_mic_device_busy)
    {
      /* g_mic_hold_active：让路期间（robot_ui 要出声）一帧都不许重开麦。
       * 正常让路时 g_listen_wanted 已经是 false（stop_audio_listening 收掉的）
       * 本来就进不来；这条是兜住"让路没停干净"那种情况 —— 设备停不下来是驱动的
       * 事，守护在这儿按退避反复重开只会跟它抢，把状态搅得更乱（见 mic_hold_apply
       * 里那条"不补救"的说明）。
       *
       * g_mic_device_busy：让路线程正拿着设备开关（停常开监听 / 重开麦那一手还没
       * 做完）。它重开麦时 g_mic_hold_active 已经被清掉了，光靠上面那条挡不住 ——
       * 两边同时 audio_record_start() 会开出两条录音线程抢一个设备。这一拍先不做，
       * 下一拍（100ms 后）再来：本函数本来就是"不活跃就重试"的节奏，让一拍不心疼。 */

      g_listen_dead_since = 0;
      return;
    }

  /* 录音还活着（设备已 START 且线程没退出）：把重试状态复位 */

  if (audio_record_is_active(&g_audio_ctx))
    {
      g_listen_retry_armed = false;
      g_listen_fail_count = 0;
      g_listen_backoff_ms = LISTEN_SUPERVISE_RETRY_MS;
      g_listen_dead_since = 0;
      g_audio_ctx.record_died = false;   /* 活着就说明上一代的死亡标记没意义了 */
      return;
    }

  now = main_now_ms();

  /* 记下"从哪一刻起听不见了"：只在第一次发现不活跃时置位。
   * 0 是保留值（表示还没断），真拿到 0 就退一格用 1，代价是那一轮少算 1ms。 */

  if (g_listen_dead_since == 0)
    {
      g_listen_dead_since = (now != 0) ? now : 1;
    }

  /* 断了多久了（用差值比较，避开 49 天一次的 uint32 回绕）。 */

  stale = ((int32_t)(now - g_listen_dead_since) >=
           (int32_t)LISTEN_SUPERVISE_FORCE_MS);

  /* 扬声器在响：这次先不抢设备。半双工上把麦抢回来就会掐掉正在放的那一段，
   * 而播放本身有上限（≤ AUDIO_PLAY_BUFFER_MS = 30 秒），播完播放线程还会按原
   * 配置把录音恢复起来 —— 也就是说这一条通常自愈。
   * 只有"长期不活跃"的兜底（stale）才越过它：那种情况下"还在放"本身就说明
   * 播放标志也卡住了（正常出声到不了 45 秒），不抢就永远聋着。 */

  if (audio_is_playing(&g_audio_ctx) && !stale)
    {
      g_listen_retry_armed = false;
      return;
    }

  /* 录音线程自己报的"异常中断"（read 返回 0 / 负值，但不是 audio_record_stop()
   * 让的 —— 这两种收尾在 ai_audio.c 录音线程末尾分开记）。认领一次就清掉，
   * 因为这一代录音会话不会再回来了：不用等状态机那 10 / 30 秒超时，也不用等
   * 守护的 1.8 秒排程（只剩下面 urgent 那道防抖下限）。
   * （它放行的"忙"只有 AI_TALKING / 追问这两种：那两件事本来就不占麦克风，
   *   现在把麦重开不会打断它们。） */

  died = g_audio_ctx.record_died;
  if (died)
    {
      g_audio_ctx.record_died = false;
      printf("[监听守护] 录音线程异常中断（不是我们要停的那种），会话已死，"
             "不等状态机那 10/30 秒超时\n");
    }

  /* 正忙：不排重试，也不动退避（退避要按"设备到底能不能起来"算）。
   * 等流程走完，下一轮从"发现不活跃"重新计时，那个 1.8 秒的余量还在。 */

  if (!died && !stale &&
      (g_ask_phase != ASK_PHASE_IDLE ||
       sm_get_state(ctx) == SM_STATE_AI_TALKING))
    {
      g_listen_retry_armed = false;
      return;
    }

  /* 立即重开的条件：会话已死（异常中断 / 长期不活跃），**而且**距上一次重开尝试
   * 已经过了正常的重试间隔。后半句是防抖：设备真坏的时候录音线程会"一起来就死"，
   * 不压这一下就成了每秒十次开关设备 + 十行日志（退避那套本来就是拦这个的）。 */

  urgent = (died || stale) &&
           ((int32_t)(now - g_listen_last_try_ms) >=
            (int32_t)LISTEN_SUPERVISE_RETRY_MS);

  /* 排一次重试再等到点 */

  if (!urgent)
    {
      if (!g_listen_retry_armed)
        {
          g_listen_retry_armed = true;
          g_listen_retry_at = now + g_listen_backoff_ms;
          printf("[监听守护] 录音不活跃，%u ms 后尝试重启语音监听\n",
                 (unsigned)g_listen_backoff_ms);
          return;
        }

      if ((int32_t)(now - g_listen_retry_at) < 0)
        {
          return;
        }
    }

  /* 从这里往下就是真的重开了。日志放在这里而不是判定处：判定是每一拍都跑的，
   * 放前面会刷屏。 */

  if (stale)
    {
      printf("[监听守护] 录音已断 %u ms 却一直\"忙\"（播放/追问/AITalking 卡住），"
             "判定会话已死，强制重开语音监听\n",
             (unsigned)(now - g_listen_dead_since));
    }

  /* 能走到这里，说明"录音断了、扬声器没响、状态机却在 AI_TALKING"——
   * 这就是"会话已死"最典型的样子（正常 AI_TALKING 期间录音是活的，唯一会停它的
   * 是 TTS 出声，而那种情况上面播放那条判据已经挡掉了）。所以顺手把状态机推回
   * 可听状态：AI_TALKING 对 VAD 报的"语音开始 / 结束"（SM_EVENT_WAKEUP /
   * SM_EVENT_VOICE_COMPLETE）是**全部忽略**的（见 ai_state_machine.c 的
   * sm_ai_talking_transition 默认分支），只重开麦、不推状态机的话，用户下一句话
   * 照样进不来，要一直等到 AI_TALKING 自己那 30 秒超时。
   * 推的目标态（IDLE）就是它自己超时那条路的目标态，只是不等那 30 秒。
   * 别的状态一律不动：IDLE / LISTENING / CARE_REMIND / ALARM 都会正常收 VAD 事件，
   * 尤其 CARE_REMIND / ALARM 有自己的收尾节奏，不能被我们拽走。
   * 迟到的回复不会因此丢：播放和界面反馈都在 llm_complete_callback 里，
   * 不依赖状态机停在哪个状态。 */

  if (sm_get_state(ctx) == SM_STATE_AI_TALKING)
    {
      printf("[监听守护] 状态机还卡在 AI_TALKING，推回 IDLE"
             "（否则新听到的语音会被它忽略）\n");
      sm_force_state(ctx, SM_STATE_IDLE);
    }

  /* 真开麦之前先抢"设备动作权"：让路线程也会 start/stop 常开录音，两边同时上手
   * 会开出两条录音线程抢同一个设备（它拿到锁时同理不会跟这里撞）。
   * trylock 失败就放弃这一拍 —— 本函数本来就是"录音不活跃就重试"的节奏，
   * 让一拍不心疼；拿到锁之后再复查一次判定所依赖的量（等锁期间它们可能变了）。 */

  if (pthread_mutex_trylock(&g_mic_device_lock) != 0)
    {
      return;
    }

  if (g_mic_hold_active || !g_listen_wanted || g_mic_device_busy ||
      audio_record_is_active(&g_audio_ctx))
    {
      pthread_mutex_unlock(&g_mic_device_lock);
      return;
    }

  /* "长期不活跃"的判定到此用掉，清掉计时：下面万一重开失败，走的是正常退避
   * （1.8s 起翻倍），不会每一拍都来敲一次设备。 */

  g_listen_dead_since = 0;
  g_listen_last_try_ms = now;

  /* 重开之前清场：上一段会话可能死在"人正说话"中间，g_speech_capturing
   * 还挂着 true、g_speech_buf 里有半截累积语音，不清掉的话下一次进 ASR 的
   * 会是跨会话的脏数据。此刻录音线程已经不在（上面刚判过不活跃），不会有回调
   * 往里写。 */

  g_speech_capturing = false;
  g_speech_frames = 0;

  /* 唤醒词模块的流式状态同理：它可能正好停在"这句话说到一半"（预滚环、
   * 待判句子、自适应本底都是上一段音频的）。麦克风断了这么久，这些状态
   * 已经对不上了，重新开始听之前清一次（kws_dtw.h 的建议）。
   * 这里清是安全的：录音线程已经死了，没有第二个线程在调这个模块 ——
   * kws_feed 只允许一个线程调，那个线程不在，就不存在并发。
   * 代价是清完要 300ms（KWS_SETTLE_FRAMES）重新量本底，这期间不会起端点。
   * 只在功能开了的时候清：没模板时这条路径和以前一个字节都不差。 */

  if (g_kws_enabled)
    {
      kws_reset();
    }

  ret = start_audio_listening(ctx);

  pthread_mutex_unlock(&g_mic_device_lock);

  if (ret == OK)
    {
      printf("[监听守护] 语音监听已恢复（之前连续失败 %u 次）\n",
             (unsigned)g_listen_fail_count);
      g_listen_retry_armed = false;
      g_listen_fail_count = 0;
      g_listen_backoff_ms = LISTEN_SUPERVISE_RETRY_MS;
      return;
    }

  /* 失败就退避：1.8s -> 3.6s -> 7.2s -> 10s（封顶）。
   * 日志按上面的节流打，别让串口被一条刷屏的日志淹掉。 */

  g_listen_fail_count++;
  g_listen_backoff_ms *= 2;
  if (g_listen_backoff_ms > LISTEN_SUPERVISE_MAX_MS)
    {
      g_listen_backoff_ms = LISTEN_SUPERVISE_MAX_MS;
    }

  g_listen_retry_at = main_now_ms() + g_listen_backoff_ms;

  if (g_listen_fail_count <= LISTEN_SUPERVISE_LOG_FIRST ||
      g_listen_fail_count % LISTEN_SUPERVISE_LOG_EVERY == 0)
    {
      printf("[监听守护] 重启语音监听失败 %u 次 (ret=%d)，%u ms 后再试\n",
             (unsigned)g_listen_fail_count, ret,
             (unsigned)g_listen_backoff_ms);
    }
}

/**
 * @brief  异常声音检测回调
 *
 * 检测到异常声之后不再直接跳报警：先问一句「请问需要帮助吗？」，
 * 听老人的回答再决定要不要真报警（流程见下面的"异常声 -> 追问流程"一节）。
 *
 * ⚠️ 本函数跑在**检测线程**里，而且检测线程正拿着 ctx->buffer_lock
 * （见 ai_sound_detect.c 的 sound_detect_thread：sound_detect_process_window()
 * 是在锁里调的）。所以这里绝不能做 TTS/ASR 这种秒级的阻塞动作 ——
 * 那会把喂数据的录音线程一起卡住。这里只做三件事：过滤、防抖、记事件。
 */

static void sound_detect_callback(sound_type_t type, float confidence,
                                  void *user_data)
{
  uint32_t now = main_now_ms();

  (void)user_data;

  printf("[安全] 检测到异常声音: %s (置信度: %.2f)\n",
         sound_detect_get_type_name(type), confidence);

  /* 1. 别把喇叭自己放出来的声音当异常：
   *    放音期间 audio_play_start() 已经把麦克风停了（录音线程退出，没人再喂
   *    检测器），这里的判断是兜住"停之前已经灌进去的那几帧"。 */

  if (audio_is_playing(&g_audio_ctx))
    {
      printf("[安全] 正在放音，忽略这次检测\n");
      return;
    }

  /* 2. 已经有一轮追问在跑：不叠加，也不排队 */

  if (g_ask_phase != ASK_PHASE_IDLE)
    {
      printf("[安全] 追问流程进行中，忽略这次检测\n");
      return;
    }

  /* 3. 防抖：同一阵响声（一秒的检测窗口会连着报好几次）只触发一次 */

  if (g_last_abnormal_ms != 0 &&
      now - g_last_abnormal_ms < ASK_DEBOUNCE_MS)
    {
      printf("[安全] 距上次触发 %u ms，防抖忽略\n",
             (unsigned)(now - g_last_abnormal_ms));
      return;
    }

  g_last_abnormal_ms = now;
  g_ask_pending_type = (int)type;
  g_ask_pending_conf = confidence;
  g_ask_round = 0;
  g_ask_answer_ready = false;
  g_ask_play_done = false;
  g_ask_answer[0] = '\0';
  g_ask_phase = ASK_PHASE_PENDING;

  printf("[追问] 已排入追问流程 (类型=%s, 置信度=%.2f)\n",
         sound_detect_get_type_name(type), confidence);
}

/**
 * @brief  初始化并启动声音检测
 */

static int start_sound_detection(sm_context_t *ctx)
{
  int ret;

  /* 配置声音检测 */

  sound_detect_config_t detect_cfg =
  {
    .mode = DETECT_MODE_REALTIME,
    .threshold = SOUND_DETECT_THRESHOLD_DEFAULT,
    .sample_rate = SOUND_DETECT_SAMPLE_RATE,
    .frame_ms = SOUND_DETECT_FRAME_MS,
    .enable_vad = true,
    .enable_feedback = true,
    .callback = sound_detect_callback,
    .user_data = ctx
  };

  ret = sound_detect_init(&g_sound_ctx, &detect_cfg);
  if (ret < 0)
    {
      printf("[错误] 声音检测初始化失败: %d\n", ret);
      return ret;
    }

  /* TODO: 加载训练好的Edge Impulse模型 */
  /* ret = sound_detect_load_model_file(&g_sound_ctx, "/data/model.bin"); */

  /* 启动实时检测 */

  ret = sound_detect_start(&g_sound_ctx);
  if (ret < 0)
    {
      printf("[错误] 启动声音检测失败: %d\n", ret);
      sound_detect_deinit(&g_sound_ctx);
      return ret;
    }

  printf("[安全] 声音检测已启动\n");
  g_sound_started = true;
  return OK;
}

/**
 * @brief  停止声音检测
 */

static void stop_sound_detection(void)
{
  if (!g_sound_started)
    {
      return;
    }

  sound_detect_stop(&g_sound_ctx);
  sound_detect_deinit(&g_sound_ctx);
  g_sound_started = false;
  printf("[安全] 声音检测已停止\n");
}

/****************************************************************************
 * 异常声 -> 追问流程 (W1)
 *
 * 用户点名的需求：检测到异常声别直接报警，先问一句「请问需要帮助吗？」，
 * 听老人的回答再决定。
 *
 * 状态机（全部在 main_loop_task 的 100ms 心跳里推进）：
 *
 *   IDLE ──sound_detect_callback() 过滤+防抖后置 PENDING──> PENDING
 *   PENDING ──tick: 播 TTS「请问需要帮助吗？」──> SPEAKING
 *   SPEAKING ──播放完成回调置 g_ask_play_done──> LISTENING(deadline = now + 6s)
 *   LISTENING ──录音线程做 ASR 后置 g_ask_answer_ready──> 三路判定：
 *        紧急（需要/救命/帮忙/疼/摔倒…） -> 上报报警 + publish start_alarm -> IDLE
 *        没事（不用/没事/还好…）        -> TTS 安抚一句                     -> IDLE
 *        没听清（空串/没说出来）         -> 还有轮数就再问一次；问满 2 轮就
 *                                          上报"未确认"                     -> IDLE
 *   LISTENING ──tick 发现超过 deadline──> 同上"没听清"
 *
 * 音频链路全部复用现成的：
 *   - 说话：voice_tts_speak() + audio_play_start()（播放内部会停录音、播完自动恢复录音）
 *   - 听  ：常开的 start_audio_listening()（VAD + 累积缓冲）+ process_ai_dialogue() 的 ASR
 ****************************************************************************/

/**
 * @brief  「请问需要帮助吗？」说完的回调（**在播放线程里**）
 *
 * 只置标志，相位交给 ask_flow_tick() 推。
 */

static void ask_play_done_cb(void *user_data)
{
  (void)user_data;
  g_voice_spoke = true;
  g_ask_play_done = true;
}

/**
 * @brief  用现成的 TTS + 播放链路说一句话
 * @param  text  要说的文本
 * @return 0 已开始播放, 负值失败
 */

static int ask_speak(const char *text)
{
  static unsigned char *tts_buf = NULL;
  static size_t tts_buf_size = 0;
  size_t tts_len = 0;
  int ret;

  if (tts_buf == NULL)
    {
      tts_buf_size = ASK_TTS_BUF_BYTES;
      tts_buf = malloc(tts_buf_size);
      if (tts_buf == NULL)
        {
          /* 短句 1 秒 ≈ 32 KB，够用就不至于整条流程哑掉 */
          tts_buf_size = 32 * 1024;
          tts_buf = malloc(tts_buf_size);
        }

      if (tts_buf == NULL)
        {
          printf("[追问] TTS 缓冲分配失败\n");
          return -ENOMEM;
        }
    }

  ret = voice_tts_speak(text, tts_buf, tts_buf_size, &tts_len);
  if (ret < 0 || tts_len == 0)
    {
      printf("[追问] 语音合成失败: %d\n", ret);
      return ret < 0 ? ret : -EIO;
    }

  /* 半双工不用自己管：audio_play_start() 会先停录音，播完按原配置把录音恢复起来
   * （ai_audio.c 的 audio_prepare_output / audio_resume_record），
   * 恢复之后麦克风才重新有人喂检测器 —— 顺带也挡住了"喇叭的声音被判成异常"。
   * 起播用 locked 版本：它内部会停录音，和让路线程的设备动作互斥（见
   * audio_play_start_locked 的说明）。 */

  ret = audio_play_start_locked(&g_audio_ctx, (const int16_t *)tts_buf,
                                tts_len / sizeof(int16_t), ask_play_done_cb, NULL);
  if (ret < 0)
    {
      printf("[追问] 播放启动失败: %d\n", ret);
      return ret;
    }

  printf("[追问] 出声: %s\n", text);
  return OK;
}

/**
 * @brief  录音线程送来这一轮的识别结果（空串 = 没听清）
 */

static void ask_flow_mark_answer(const char *text)
{
  g_ask_answer[0] = '\0';

  if (text != NULL)
    {
      strncpy(g_ask_answer, text, sizeof(g_ask_answer) - 1);
      g_ask_answer[sizeof(g_ask_answer) - 1] = '\0';
    }

  g_ask_answer_ready = true;
}

/* 回答的关键词表。
 * 判定顺序**必须**先否定后紧急：「不需要」「不用」这类否定回答里本身就含
 * 「需要」「用」这些字，先匹配紧急关键词会把"不需要，我没事"判成求救。
 * 表里都是整词/短句而不是单字，宁可判成"没听清"（会再问一次），也不要误报警。 */

static const char *const g_ask_neg_keywords[] =
{
  "不用", "不需要", "没事", "没有", "还好", "没关", "别管", "不痛", "不疼",
  "没摔", "没跌", "不难受", "挺好", "没啥", "不碍事"
};

static const char *const g_ask_sos_keywords[] =
{
  "需要", "救命", "救", "帮", "疼", "痛", "摔倒", "跌", "不舒服", "难受",
  "起不来", "骨折", "流血", "头晕", "恶心", "动不了"
};

/**
 * @brief  按关键词判定一句识别结果
 * @return ASK_VERDICT_*
 */

static int ask_classify(const char *text)
{
  size_t i;

  if (text == NULL || text[0] == '\0')
    {
      return ASK_VERDICT_UNCLEAR;
    }

  for (i = 0; i < sizeof(g_ask_neg_keywords) / sizeof(g_ask_neg_keywords[0]);
       i++)
    {
      if (strstr(text, g_ask_neg_keywords[i]) != NULL)
        {
          return ASK_VERDICT_REASSURED;
        }
    }

  for (i = 0; i < sizeof(g_ask_sos_keywords) / sizeof(g_ask_sos_keywords[0]);
       i++)
    {
      if (strstr(text, g_ask_sos_keywords[i]) != NULL)
        {
          return ASK_VERDICT_EMERGENCY;
        }
    }

  return ASK_VERDICT_UNCLEAR;
}

/**
 * @brief  收尾：清干净标志，并把状态机从 AI_TALKING 放回监听态
 */

static void ask_flow_finish(sm_context_t *ctx)
{
  g_ask_phase = ASK_PHASE_IDLE;
  g_ask_round = 0;
  g_ask_play_done = false;
  g_ask_answer_ready = false;
  g_ask_answer[0] = '\0';

  /* 这一轮的 ASR 是被 VAD 推进 AI_TALKING 之后在 sm_ai_talking_enter() 里做的，
   * 我们没给它发 AI_RESPONSE，收尾时补一下，否则它会一直停在 AI_TALKING
   * 等 30 秒超时。 */
  if (sm_get_state(ctx) == SM_STATE_AI_TALKING)
    {
      sm_handle_event(ctx, SM_EVENT_AI_RESPONSE);
    }

  printf("[追问] 流程结束\n");
}

/**
 * @brief  判定紧急：上报报警 + 让界面弹报警页
 */

static void ask_flow_escalate(sm_context_t *ctx, const char *answer)
{
  char detail[ASK_DETAIL_MAX];

  snprintf(detail, sizeof(detail), "检测到%s（置信度 %.2f），老人回答「%s」",
           sound_detect_get_type_name((sound_type_t)g_ask_pending_type),
           (double)g_ask_pending_conf, answer);

  printf("[追问] 判定: 紧急 -> %s\n", detail);

  if (g_net_started)
    {
      /* ai_network_send_alarm() 内部就是 report_alarm()：publish 到
       * zhi_ai/<client_id>/alarm + 手机推送，所以不用再单独调一次 report_alarm()。 */
      ai_network_send_alarm(&g_net_ctx, "sound_emergency", detail);

      /* 让界面（robot_ui）弹报警页 */
      ai_network_send_start_alarm(&g_net_ctx, detail);
      ai_network_send_face(&g_net_ctx, AI_CMD_FACE_WORRIED);
    }

  /* 确认紧急才进报警状态（原来是"一检测到异常声就进"，误报会直接把设备按在
   * 报警态 60 秒，期间唤醒/对话全被吃掉）。 */
  sm_handle_event(ctx, SM_EVENT_ALARM_DETECTED);

  ask_flow_finish(ctx);
}

/**
 * @brief  判定"没事"：安抚一句就结束
 */

static void ask_flow_soothe(sm_context_t *ctx, const char *answer)
{
  printf("[追问] 判定: 不需要帮忙（回答「%s」）\n", answer);

  /* 这句话也要上屏：主路同进程直调，旁路照旧 MQTT（给手机/PC 端） */
  robot_ui_bridge_voice_reply("那就好，有事再叫我。");

  if (g_net_started)
    {
      ai_network_send_ai_reply(&g_net_ctx, "那就好，有事再叫我。");
      ai_network_send_face(&g_net_ctx, AI_CMD_FACE_HAPPY);
    }

  /* 这句播完不用再听，所以直接收尾（播放完成回调进来时相位已经是 IDLE，
   * tick 不会理它）。 */
  ask_speak("那就好，有事再叫我。");
  ask_flow_finish(ctx);
}

/**
 * @brief  没听清：还有轮数就再问一次，问满了就上报"未确认"
 */

static void ask_flow_unclear(sm_context_t *ctx)
{
  if (g_ask_round < ASK_MAX_ROUNDS)
    {
      printf("[追问] 没听清，再问一次（第 %d 轮）\n", g_ask_round + 1);
      g_ask_phase = ASK_PHASE_PENDING;
      return;
    }

  {
    char type_name[64];

    /* 报"未确认"走 /sound_alarm 这条通道（不上报 /alarm、不推手机）：
     * 两次都没听清就按真报警推给家人，误报比漏报更伤这个场景。 */
    snprintf(type_name, sizeof(type_name), "unconfirmed_%s",
             sound_detect_get_type_name((sound_type_t)g_ask_pending_type));

    printf("[追问] 两轮都没听清，上报未确认: %s\n", type_name);

    if (g_net_started)
      {
        ai_network_report_sound_alarm(&g_net_ctx, type_name,
                                      (int)(g_ask_pending_conf * 100.0f));
        ai_network_send_face(&g_net_ctx, AI_CMD_FACE_WORRIED);
      }
  }

  ask_flow_finish(ctx);
}

/**
 * @brief  按识别结果分三路
 */

static void ask_flow_decide(sm_context_t *ctx, const char *answer)
{
  int verdict = ask_classify(answer);

  printf("[追问] 回答「%s」-> %s\n", answer,
         verdict == ASK_VERDICT_EMERGENCY ? "需要帮助" :
         verdict == ASK_VERDICT_REASSURED ? "不需要帮助" : "没听清");

  if (verdict == ASK_VERDICT_EMERGENCY)
    {
      ask_flow_escalate(ctx, answer);
    }
  else if (verdict == ASK_VERDICT_REASSURED)
    {
      ask_flow_soothe(ctx, answer);
    }
  else
    {
      ask_flow_unclear(ctx);
    }
}

/**
 * @brief  进入 ASK_PHASE_LISTENING：开一个限时窗口
 */

static void ask_flow_listen_start(void)
{
  g_ask_answer_ready = false;
  g_ask_answer[0] = '\0';
  g_ask_deadline_ms = main_now_ms() + ASK_LISTEN_TIMEOUT_MS;
  g_ask_phase = ASK_PHASE_LISTENING;

  printf("[追问] 开始限时听回答（%u 秒）\n", ASK_LISTEN_TIMEOUT_MS / 1000);
}

/**
 * @brief  问一句（第 1 轮 / 第 2 轮用不同的话）
 */

static void ask_flow_begin(sm_context_t *ctx)
{
  const char *question;
  int ret;

  g_ask_round++;

  question = (g_ask_round <= 1) ? "请问需要帮助吗？"
                                : "您还好吗？需要我帮忙吗？";

  printf("[追问] 第 %d 轮: %s\n", g_ask_round, question);

  /* 屏幕上也要把这句话显示出来：主路同进程直调，旁路照旧 MQTT */
  robot_ui_bridge_voice_reply(question);

  if (g_net_started)
    {
      ai_network_send_ai_reply(&g_net_ctx, question);
      ai_network_send_face(&g_net_ctx, AI_CMD_FACE_THINKING);
    }

  /* 清掉上一轮的残留（ASR 累积缓冲 / 结果 / 播放完成标志） */
  g_ask_answer_ready = false;
  g_ask_play_done = false;
  g_ask_answer[0] = '\0';
  g_speech_frames = 0;

  ret = ask_speak(question);
  if (ret < 0)
    {
      /* 说不出来就别干等：按"没听清"走（还有轮数会再试，问满了就上报未确认） */
      printf("[追问] 这一轮说不出来，按没听清处理\n");
      ask_flow_unclear(ctx);
      return;
    }

  g_ask_phase = ASK_PHASE_SPEAKING;
  g_ask_phase_ms = main_now_ms();
}

/**
 * @brief  追问流程心跳（在 main_loop_task 里每 100ms 调一次）
 *
 * 只有这个线程改 g_ask_phase；TTS 合成是阻塞调用（云端 HTTPS），
 * 在这里阻塞几秒是可以接受的 —— 主循环这一轮只做状态机超时检查和一些空壳轮询。
 */

static void ask_flow_tick(sm_context_t *ctx)
{
  uint32_t now = main_now_ms();

  /* 退出中就别再开新动作了：收尾顺序里 stop_audio_listening() 在前、
   * pthread_join(loop_thread) 在后，这里要是还在播/听，会把已经关掉的
   * 录音又拉起来。 */
  if (!g_running)
    {
      return;
    }

  switch (g_ask_phase)
    {
      case ASK_PHASE_PENDING:
        if (g_ask_round >= ASK_MAX_ROUNDS)
          {
            /* 兜底：正常路径到不了这里（ask_flow_unclear() 会先收尾） */
            ask_flow_finish(ctx);
            break;
          }

        ask_flow_begin(ctx);
        break;

      case ASK_PHASE_SPEAKING:
        if (g_ask_play_done ||
            now - g_ask_phase_ms >= ASK_SPEAK_TIMEOUT_MS)
          {
            ask_flow_listen_start();
          }
        break;

      case ASK_PHASE_LISTENING:
        if (g_ask_answer_ready)
          {
            char answer[ASK_ANSWER_MAX];

            memcpy(answer, g_ask_answer, sizeof(answer));
            answer[sizeof(answer) - 1] = '\0';
            ask_flow_decide(ctx, answer);
          }
        else if (now >= g_ask_deadline_ms)
          {
            printf("[追问] 这一轮没听到回答（超时）\n");
            ask_flow_unclear(ctx);
          }
        break;

      default:
        break;
    }
}

/**
 * @brief  关怀回调函数
 */

static void care_callback(care_type_t type, const char *message,
                          void *user_data)
{
  sm_context_t *ctx = (sm_context_t *)user_data;

  printf("[关怀] 触发关怀: %s\n", care_get_type_name(type));
  printf("[关怀] 消息: %s\n", message);

  /* 触发关怀事件 */

  sm_handle_event(ctx, SM_EVENT_CARE_TIMER);

  /* TODO: 将消息转换为语音播放 */
  /* 1. 调用TTS服务 */
  /* 2. 播放语音 */

  printf("[关怀] 关怀完成\n");
}

/**
 * @brief  初始化并启动主动关怀
 */

static int start_care(sm_context_t *ctx)
{
  int ret;

  /* 配置主动关怀 */

  care_config_t care_cfg =
  {
    .enable_greeting = true,
    .enable_health = true,
    .enable_life = true,
    .enable_emotion = true,
    .enable_exercise = true,
    .enable_medicine = true,
    .enable_meal = true,
    .enable_sleep = true,
    .smart_mode = true,
    .callback = care_callback,
    .user_data = ctx
  };

  ret = care_init(&g_care_ctx, &care_cfg);
  if (ret < 0)
    {
      printf("[错误] 主动关怀初始化失败: %d\n", ret);
      return ret;
    }

  /* 启动关怀 */

  ret = care_start(&g_care_ctx);
  if (ret < 0)
    {
      printf("[错误] 启动主动关怀失败: %d\n", ret);
      care_deinit(&g_care_ctx);
      return ret;
    }

  printf("[关怀] 主动关怀已启动\n");
  g_care_started = true;

  /* 打印任务列表 */
  /* 注意: care_task_t 含 256B message 等, 20 个约 9KB, 放栈上会溢出
   * hello_app 任务栈(8KB), 因此用 static 一次性缓冲 */
  static care_task_t tasks[CARE_MAX_TASKS];
  int count = care_get_task_list(&g_care_ctx, tasks, CARE_MAX_TASKS);
  printf("[关怀] 当前关怀任务: %d 个\n", count);

  return OK;
}

/**
 * @brief  停止主动关怀
 */

static void stop_care(void)
{
  if (!g_care_started)
    {
      return;
    }

  care_stop(&g_care_ctx);
  care_deinit(&g_care_ctx);
  g_care_started = false;
  printf("[关怀] 主动关怀已停止\n");
}

/**
 * @brief  模拟语音唤醒检测
 * @note   实际实现由成员一提供音频驱动接口
 */

static void simulate_wakeup_detection(sm_context_t *ctx)
{
  /* TODO: 替换为真实的语音唤醒检测 */
  /* wakeup_listen_start(); */
  /* if (wakeup_word_detected()) { */
  /*     sm_handle_event(ctx, SM_EVENT_WAKEUP); */
  /* } */
}

/**
 * @brief  模拟异常声音检测
 * @note   实际实现由成员二集成声音分类模型
 */

static void simulate_alarm_detection(sm_context_t *ctx)
{
  /* TODO: 替换为真实的声音检测 */
  /* if (sound_detect_alarm()) { */
  /*     sm_handle_event(ctx, SM_EVENT_ALARM_DETECTED); */
  /* } */
}

/**
 * @brief  模拟主动关怀定时器
 * @note   实际实现由成员二设计关怀逻辑
 */

static void simulate_care_timer(sm_context_t *ctx)
{
  /* TODO: 实现定时关怀逻辑 */
  /* static uint32_t last_care_time = 0; */
  /* uint32_t now = get_tick_ms(); */
  /* if (now - last_care_time > CARE_INTERVAL_MS) { */
  /*     sm_handle_event(ctx, SM_EVENT_CARE_TIMER); */
  /*     last_care_time = now; */
  /* } */
}

/**
 * @brief  播放完成回调
 */

static void play_complete_callback(void *user_data)
{
  sm_context_t *ctx = (sm_context_t *)user_data;
  printf("[音频] 语音播放完成\n");

  /* 这轮已经出过声了：语音状态回传据此不在收尾的那一小段里报"正在想" */
  g_voice_spoke = true;

  /* 播放完成后触发AI响应完成事件 */

  sm_handle_event(ctx, SM_EVENT_AI_RESPONSE);
}

/**
 * @brief  AI回复完成回调
 */

static void llm_complete_callback(const char *response, int error,
                                  void *user_data)
{
  sm_context_t *ctx = (sm_context_t *)user_data;

  if (error != 0)
    {
      printf("[LLM] AI请求失败: %d\n", error);
      sm_handle_event(ctx, SM_EVENT_AI_ERROR);
      return;
    }

  printf("[LLM] AI回复: %s\n", response);

  /* 回传界面：AI 的每次回复正文都要上屏。
   * 主路是同进程直调（robot_ui_bridge_voice_reply：清洗 + 主屏对话区 + 镜像
   * 面板对话区，内部全是 lv_async_call，本函数在录音线程里跑也安全）；
   * 旁路是原来的 MQTT ai_reply/set_face，给手机/PC 端 —— 那种"本机界面反馈
   * 必须绕公网跑一个来回"的写法，网络一抖用户看到的就是完全没反应。 */

  robot_ui_bridge_voice_reply(response);

  if (g_net_started)
    {
      ai_network_send_ai_reply(&g_net_ctx, response);
      ai_network_send_face(&g_net_ctx, AI_CMD_FACE_HAPPY);
    }

  /* TTS: 文字转语音 */

  /* TTS 缓冲 256 KB ≈ 16 kHz/单声道/16 bit 的 8 秒；原来 32 KB 只有 1 秒，
   * 超出部分会被 volc_tts.c 静默截断（PCM buffer full）。
   * 必须走堆：静态 256 KB 会把内核 SRAM 顶到 96%（构建报告可查），运行时就没余量了；
   * 堆里有第二块 PSRAM（8 MB），大块分配会落到那边。 */
  static unsigned char *tts_buf = NULL;
  static size_t tts_buf_size = 0;
  size_t tts_len = 0;
  int ret;

  if (tts_buf == NULL)
    {
      tts_buf_size = 256 * 1024;
      tts_buf = malloc(tts_buf_size);
      if (tts_buf == NULL)
        {
          tts_buf_size = 32 * 1024;
          tts_buf = malloc(tts_buf_size);
        }

      if (tts_buf == NULL)
        {
          printf("[TTS] 缓冲分配失败，跳过合成\n");
          sm_handle_event(ctx, SM_EVENT_AI_RESPONSE);
          return;
        }

      printf("[TTS] 缓冲 %zu 字节已分配\n", tts_buf_size);
    }

  ret = voice_tts_speak(response, tts_buf, tts_buf_size, &tts_len);
  if (ret < 0 || tts_len == 0)
    {
      printf("[TTS] 语音合成失败: %d\n", ret);
      sm_handle_event(ctx, SM_EVENT_AI_RESPONSE);
      return;
    }

  printf("[TTS] 合成完成, %zu 字节\n", tts_len);

  /* 播放合成的语音（起播会停录音，和让路线程的设备动作互斥：
   * 见 audio_play_start_locked 的说明） */

  size_t frames = tts_len / sizeof(int16_t);
  ret = audio_play_start_locked(&g_audio_ctx,
                                (const int16_t *)tts_buf, frames,
                                play_complete_callback, ctx);
  if (ret < 0)
    {
      printf("[TTS] 播放失败: %d\n", ret);
      sm_handle_event(ctx, SM_EVENT_AI_RESPONSE);
      return;
    }

  printf("[TTS] 开始播放语音\n");
}

/****************************************************************************
 * 本机灯控意图（"说一句话就把灯打开"）
 *
 * 位置：ASR 出了文本之后、送大模型之前。命中就本机发命令 + 本地回话，
 * 这一轮不再往大模型绕 —— 省一次往返，也免得模型一本正经地说
 * "已经帮您把灯打开了"而实际上什么都没发出去。
 *
 * 判定原则是**宁可漏、不可猜**：只要沾一点否定或歧义就返回 NONE，
 * 让这一轮照原路走大模型（大模型那边不会去动灯，最多多说一句话）。
 * 灯控说错的代价（当着老人把灯开/关掉）比多等一次往返大得多。
 ****************************************************************************/

typedef enum
{
  LIGHT_INTENT_NONE = 0,   /* 不是灯控 -> 交给大模型 */
  LIGHT_INTENT_ON,         /* 开灯 */
  LIGHT_INTENT_OFF,        /* 关灯 */
  LIGHT_INTENT_BAIL        /* 是灯控说法但含否定/歧义 -> 整句交给大模型 */
} light_intent_t;

/* 开灯的说法。只收整词/短句：单字"开""灯"太容易在闲聊里撞上
 * （"开心""灯挺亮的"），误开灯比多说一句"没听懂"严重得多。 */

static const char *const g_light_on_keywords[] =
{
  "开灯", "把灯开", "打开灯", "灯打开", "开一下灯", "灯开一下", "开个灯",
  "点灯", "灯亮起", "亮一点", "亮一些", "亮点儿", "调亮",
  NULL
};

/* 关灯的说法。同理只收整词：不收"暗一点"这种（"暗"也可能是让屏幕暗），
 * 拿不准就让大模型去接。 */

static const char *const g_light_off_keywords[] =
{
  "关灯", "把灯关", "关掉灯", "灯关掉", "关一下灯", "灯关一下", "关个灯",
  "灯关上", "灯关了", "熄灯", "灭灯",
  NULL
};

/* 否定词。命中动作词之后再回头看：**同一个小句里、动作词之前**出现过
 * 这些词就整句不执行（"别开灯""不用开灯""先别关灯"绝对不能真去动灯）。
 * 表里连"没""不能"这种也收进来：多退回一次大模型只是慢一点，
 * 猜错一次是当着老人把灯开了/关了。 */

static const char *const g_light_neg_keywords[] =
{
  "别", "不用", "不要", "先别", "无需", "勿", "不能", "不准", "不许",
  "没有", "没",
  NULL
};

/* 小句分隔符（半角 + 全角标点 + 空白）。按小句判定是为了让
 * "没什么事，把灯打开"里前一句的"没"管不到后一句的"打开灯"。
 *
 * 这里必须是**整串**的表，不能用 strcspn()/strspn() 那种"字节集合"：
 * 全角标点的 UTF-8 里带的字节会和汉字撞上（例："开" = E5 BC 80，
 * "。" = E3 80 82，两者都有 0x80），字节集合匹配会把"开灯"从中间切成
 * "开"和"灯"，灯控就永远命中不了。 */

static const char *const g_light_clause_seps[] =
{
  ",", ".", "!", "?", ";", ":", " ", "\t", "\r", "\n",
  "，", "。", "！", "？", "；", "：", "、",
  NULL
};

/**
 * @brief  p 开头是不是分隔符；是就返回它的字节数，不是返回 0
 */

static size_t light_sep_len(const char *p)
{
  size_t i;

  for (i = 0; g_light_clause_seps[i] != NULL; i++)
    {
      size_t n = strlen(g_light_clause_seps[i]);

      if (strncmp(p, g_light_clause_seps[i], n) == 0)
        {
          return n;
        }
    }

  return 0;
}

/**
 * @brief  在 hay 里找表内最早出现的关键词
 * @param  table  关键词表（以 NULL 结尾）
 * @param  pos    输出：命中位置相对 hay 的字节下标（可为 NULL）
 * @return 命中的关键词（**指向表内的字符串，不是 hay 里的位置**），
 *         没命中返回 NULL。要位置就读 pos，别拿返回值和 hay 相减。
 */

static const char *light_kw_find(const char *const *table, const char *hay,
                                 size_t *pos)
{
  const char *best = NULL;
  size_t best_pos = 0;
  size_t i;

  for (i = 0; table[i] != NULL; i++)
    {
      const char *p = strstr(hay, table[i]);

      if (p != NULL && (best == NULL || (size_t)(p - hay) < best_pos))
        {
          best = table[i];
          best_pos = (size_t)(p - hay);
        }
    }

  if (best != NULL && pos != NULL)
    {
      *pos = best_pos;
    }

  return best;
}

/**
 * @brief  判定一个小句的灯控意图
 * @param  clause  已经用 '\0' 截好的小句
 * @return LIGHT_INTENT_ON / _OFF / _NONE / _BAIL（否定或歧义）
 */

static light_intent_t light_clause_intent(const char *clause)
{
  const char *on_kw;
  const char *off_kw;
  size_t on_pos = 0;
  size_t off_pos = 0;
  size_t kw_pos;
  size_t i;

  on_kw = light_kw_find(g_light_on_keywords, clause, &on_pos);
  off_kw = light_kw_find(g_light_off_keywords, clause, &off_pos);

  /* 一句里又开又关（"把灯关掉再打开"）= 拿不准，退回大模型 */

  if (on_kw != NULL && off_kw != NULL)
    {
      printf("[意图] 灯控语句同时含开/关，拿不准，交给大模型: %s\n", clause);
      return LIGHT_INTENT_BAIL;
    }

  if (on_kw == NULL && off_kw == NULL)
    {
      return LIGHT_INTENT_NONE;
    }

  kw_pos = (on_kw != NULL) ? on_pos : off_pos;

  /* 反向动作字：这句已经命中灯控词之后再看一眼有没有反方向的字
   * （"把灯关掉再打开"的"开"、"开一下灯再关掉"的"关"）。
   * 只查单字（"开"/"关"），所以"把窗帘打开"这种没有灯字的话根本走不到这里，
   * 不会被误判成开灯。 */

  if (strstr(clause, (on_kw != NULL) ? "关" : "开") != NULL)
    {
      printf("[意图] 灯控语句含反方向动作字，拿不准，交给大模型: %s\n", clause);
      return LIGHT_INTENT_BAIL;
    }

  /* 否定词落在动作词**之前**才算否定这句话：动作词后面的否定词管的是
   * 下一个动作（"把灯打开…别关"），不归这一句。 */

  for (i = 0; g_light_neg_keywords[i] != NULL; i++)
    {
      const char *p = strstr(clause, g_light_neg_keywords[i]);

      if (p != NULL && (size_t)(p - clause) < kw_pos)
        {
          printf("[意图] 灯控语句含否定词「%s」，不执行，交给大模型: %s\n",
                 g_light_neg_keywords[i], clause);
          return LIGHT_INTENT_BAIL;
        }
    }

  return (on_kw != NULL) ? LIGHT_INTENT_ON : LIGHT_INTENT_OFF;
}

/**
 * @brief  判定整句 ASR 结果的灯控意图（按标点切成小句逐句看）
 * @return LIGHT_INTENT_ON / _OFF / _NONE（含拿不准的情况）
 */

static light_intent_t light_intent_classify(const char *text)
{
  light_intent_t found = LIGHT_INTENT_NONE;
  char buf[LIGHT_TEXT_MAX];
  char *p;
  size_t len;

  if (text == NULL || text[0] == '\0')
    {
      return LIGHT_INTENT_NONE;
    }

  /* 太长的句子不是灯控命令，也可能被截断到看不见关键词，不猜 */

  if (strlen(text) >= sizeof(buf))
    {
      printf("[意图] 语句过长，灯控不判定，交给大模型\n");
      return LIGHT_INTENT_NONE;
    }

  /* 逐小句要就地截断成 C 串，先拷一份（调用方还要用原文） */

  strncpy(buf, text, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  p = buf;
  while (*p != '\0')
    {
      light_intent_t one;
      const char *sep;
      size_t sep_pos = 0;
      size_t sep_len;
      char saved;

      /* 先跳过连着的标点/空白（上一轮只处理到小句末尾，分隔符留在原地） */

      sep_len = light_sep_len(p);
      if (sep_len > 0)
        {
          p += sep_len;
          continue;
        }

      /* 这个小句一直取到下一个分隔符为止。
       * light_kw_find() 返回的是**表里的字符串**，长度必须走 sep_pos，
       * 拿返回值和 p 相减会算出一个越界的"长度"。 */

      sep = light_kw_find(g_light_clause_seps, p, &sep_pos);
      len = (sep != NULL) ? sep_pos : strlen(p);

      saved = p[len];
      p[len] = '\0';                    /* 就地截出这一小句 */

      one = light_clause_intent(p);

      p[len] = saved;                   /* 还原分隔符，下一轮开头跳过它 */

      /* 有一个小句拿不准，整句就拿不准（"别开灯…把灯打开"这种别去猜） */

      if (one == LIGHT_INTENT_BAIL)
        {
          return LIGHT_INTENT_NONE;
        }

      if (one != LIGHT_INTENT_NONE)
        {
          /* 几个小句指了不同方向（"先关灯…再开灯"）也算拿不准 */

          if (found != LIGHT_INTENT_NONE && found != one)
            {
              printf("[意图] 灯控语句前后矛盾，拿不准，交给大模型: %s\n", text);
              return LIGHT_INTENT_NONE;
            }

          found = one;
        }

      p += len;
    }

  return found;
}

/**
 * @brief  说一句灯控回话（复用现成的 TTS + 播放链路）
 * @return 0 已开始播放, 负值失败（调用方负责收尾状态机）
 */

static int light_speak(sm_context_t *ctx, const char *text)
{
  static unsigned char *tts_buf = NULL;
  static size_t tts_buf_size = 0;
  size_t tts_len = 0;
  int ret;

  if (tts_buf == NULL)
    {
      tts_buf_size = LIGHT_TTS_BUF_BYTES;
      tts_buf = malloc(tts_buf_size);
      if (tts_buf == NULL)
        {
          /* 短句 1 秒 ≈ 32 KB，够用就不至于整个功能哑掉 */
          tts_buf_size = 32 * 1024;
          tts_buf = malloc(tts_buf_size);
        }

      if (tts_buf == NULL)
        {
          printf("[意图] TTS 缓冲分配失败，这句不播了\n");
          return -ENOMEM;
        }
    }

  ret = voice_tts_speak(text, tts_buf, tts_buf_size, &tts_len);
  if (ret < 0 || tts_len == 0)
    {
      printf("[意图] 灯控回话合成失败: %d\n", ret);
      return ret < 0 ? ret : -EIO;
    }

  /* 半双工不用自己管：audio_play_start() 会先停录音，播完按原配置把录音恢复起来
   * （和追问流程的 ask_speak() 同一条路）。
   * 播放完成回调直接用现成的 play_complete_callback()：它置"这轮出过声"
   * 并发 AI_RESPONSE，让状态机从 AI_TALKING 收尾回 LISTENING。 */

  ret = audio_play_start_locked(&g_audio_ctx, (const int16_t *)tts_buf,
                                tts_len / sizeof(int16_t),
                                play_complete_callback, ctx);
  if (ret < 0)
    {
      printf("[意图] 灯控回话播放启动失败: %d\n", ret);
      return ret;
    }

  printf("[意图] 灯控回话出声: %s\n", text);
  return OK;
}

/**
 * @brief  执行灯控意图：发 device_cmd + 回一句话给界面 + 用 TTS 说出来
 *
 * 返回前必须让状态机离开 AI_TALKING（要么靠 playback 完成回调，要么靠这里的
 * 兜底），否则界面会一直停在"正在识别"，监听守护也不敢开麦（它把 AI_TALKING
 * 当忙）。
 */

static void handle_light_intent(sm_context_t *ctx, light_intent_t intent)
{
  const char *what = (intent == LIGHT_INTENT_ON) ? "开灯" : "关灯";
  const char *cmd = (intent == LIGHT_INTENT_ON) ? LIGHT_CMD_ON : LIGHT_CMD_OFF;
  const char *speech;
  int cmd_ret = -ENOTCONN;

  if (g_net_started)
    {
      cmd_ret = ai_network_send_device_command(&g_net_ctx, LIGHT_DEVICE_ID, cmd);
    }

  if (cmd_ret < 0)
    {
      /* 兜底话术：明确告诉老人灯没被动过，别静默失败 */

      speech = (intent == LIGHT_INTENT_ON) ? LIGHT_FAIL_ON : LIGHT_FAIL_OFF;

      printf("[意图] 命中灯控: %s，但 device_cmd 发送失败(%d)，"
             "兜底回话「%s」\n", what, cmd_ret, speech);
    }
  else
    {
      speech = (intent == LIGHT_INTENT_ON) ? LIGHT_REPLY_ON : LIGHT_REPLY_OFF;

      printf("[意图] 命中灯控: %s -> device_cmd %s=%s，回话「%s」\n",
             what, LIGHT_DEVICE_ID, cmd, speech);
    }

  /* 界面也要看到这句话：主路同进程直调，旁路照旧 MQTT（手机/PC 端） */

  robot_ui_bridge_voice_reply(speech);

  if (g_net_started)
    {
      ai_network_send_ai_reply(&g_net_ctx, speech);
      ai_network_send_face(&g_net_ctx,
                           (cmd_ret < 0) ? AI_CMD_FACE_WORRIED
                                         : AI_CMD_FACE_HAPPY);
    }

  /* 播报期间状态机还留在 AI_TALKING：voice_state 会由 audio_is_playing()
   * 算成"正在说话"，监听守护也正好因为 AI_TALKING 不去碰麦克风。 */

  if (light_speak(ctx, speech) < 0)
    {
      /* 说不出话也得收尾，不能让这一轮悬着 */

      printf("[意图] 灯控回话没能出声，直接收尾\n");
      sm_handle_event(ctx, SM_EVENT_AI_RESPONSE);
    }
}

/**
 * @brief  取一个 UTF-8 码点，并返回它占的字节数
 *
 * 只认规范的 1~4 字节序列；续字节单独出现、或序列尾巴被截断时，退回"单个字节
 * 算一个码点"（后面按"有内容"处理：宁可多发一次，也别把老人说的话吞掉）。
 *
 * @param  p   指向 UTF-8 串里的某个字节（调用方保证 p[0] != '\0'）
 * @param  cp  输出码点
 * @return 该码点占的字节数（>= 1；只有整段续字节都齐了才返回 > 1）
 */

static size_t utf8_next_cp(const unsigned char *p, unsigned int *cp)
{
  size_t n;
  size_t i;

  if (p[0] < 0x80)
    {
      *cp = p[0];
      return 1;
    }

  if ((p[0] & 0xe0) == 0xc0)
    {
      n = 2;
      *cp = p[0] & 0x1f;
    }
  else if ((p[0] & 0xf0) == 0xe0)
    {
      n = 3;
      *cp = p[0] & 0x0f;
    }
  else if ((p[0] & 0xf8) == 0xf0)
    {
      n = 4;
      *cp = p[0] & 0x07;
    }
  else
    {
      *cp = p[0];
      return 1;
    }

  /* 先按首字节定长度，再要求后面全是续字节（高两位 10）。
   * 有一节不满足就说明这句在中间断了，当作非法字节，只吃这一个。 */

  for (i = 1; i < n; i++)
    {
      if ((p[i] & 0xc0) != 0x80)
        {
          *cp = p[0];
          return 1;
        }

      *cp = (*cp << 6) | (p[i] & 0x3f);
    }

  return n;
}

/**
 * @brief  这个码点算不算"没内容的噪声"（空白或标点）
 *
 * 只用来判"ASR 是不是压根没听清"：老人说的话总归有汉字、数字或字母；
 * 整串都是空白和标点（"."、"。。。"、"?! "、全角空格）基本是一声异响被 VAD
 * 切出来的误触发，这种既不该回传界面，也不该拿去问大模型。
 *
 * 注意别把全角字母/数字（Ａ１）也算进来：那是有内容的输入。
 */

static bool utf8_cp_is_noise(unsigned int cp)
{
  /* 控制字符、半角空格、半角删除符、不间断空格、全角空格 */
  if (cp < 0x21 || cp == 0x7f || cp == 0xa0 || cp == 0x3000)
    {
      return true;
    }

  /* ASCII 可见标点：!"#$%&'()*+,-./  :;<=>?@  [\]^_`  {|}~ */

  if ((cp >= 0x21 && cp <= 0x2f) || (cp >= 0x3a && cp <= 0x40) ||
      (cp >= 0x5b && cp <= 0x60) || (cp >= 0x7b && cp <= 0x7e))
    {
      return true;
    }

  /* U+2010 ~ U+205E：破折号、引号、省略号、间隔号等一整段符号 */

  if (cp >= 0x2010 && cp <= 0x205e)
    {
      return true;
    }

  /* 中文标点：、。〃、「」『』（）【】〔〕等 */

  if ((cp >= 0x3001 && cp <= 0x3003) ||
      (cp >= 0x3008 && cp <= 0x3011) ||
      (cp >= 0x3014 && cp <= 0x301f) ||
      (cp >= 0x3030 && cp <= 0x303f) ||
      (cp >= 0xfe10 && cp <= 0xfe19) ||
      (cp >= 0xfe30 && cp <= 0xfe4f))
    {
      return true;
    }

  /* 全角标点（！？，。：；""（）等）与半角 CJK 标点（｡｢｣､･）。
   * 全角字母（Ａ-Ｚａ-ｚ）和全角数字（０-９）不在此列，它们算内容。 */

  if ((cp >= 0xff01 && cp <= 0xff0f) || (cp >= 0xff1a && cp <= 0xff20) ||
      (cp >= 0xff3b && cp <= 0xff40) || (cp >= 0xff5b && cp <= 0xff65))
    {
      return true;
    }

  return false;
}

/**
 * @brief  ASR 原文里有没有真内容（不是空串，也不是只有空白/标点）
 */

static bool user_said_has_content(const char *text)
{
  const unsigned char *p = (const unsigned char *)text;
  unsigned int cp;

  if (text == NULL)
    {
      return false;
    }

  while (*p != '\0')
    {
      p += utf8_next_cp(p, &cp);

      if (!utf8_cp_is_noise(cp))
        {
          return true;
        }
    }

  return false;
}

/**
 * @brief  把 ASR 原文裁到界面能收下的长度（超长按整字截断）
 *
 * 砍点落在某个 UTF-8 多字节序列中间时，把那个残缺的字符整个丢掉 ——
 * 界面是直接拿去显示的，半个字会变成方块。
 *
 * @param  dst       输出缓冲
 * @param  dst_size  输出缓冲大小（含结尾 '\0'）
 * @param  src       原文
 * @return 写进 dst 的字节数（不含结尾 '\0'）
 */

static size_t user_said_cut(char *dst, size_t dst_size, const char *src)
{
  size_t n;

  if (dst_size == 0)
    {
      return 0;
    }

  n = strlen(src);
  if (n > dst_size - 1)
    {
      n = dst_size - 1;
    }

  /* 续字节的高两位固定是 10：砍点正好落在续字节上，说明这个字还缺头，
   * 往前退到它的首字节（下标 n 处最多是结尾 '\0'，不会读过界）。 */

  while (n > 0 && ((unsigned char)src[n] & 0xc0) == 0x80)
    {
      n--;
    }

  memcpy(dst, src, n);
  dst[n] = '\0';
  return n;
}

/**
 * @brief  把用户原话回传界面（本文件唯一的 user_said 发送点）
 *
 * 只从 process_ai_dialogue() 里调一次，位置在拿到非空 ASR 文本之后、**所有分支
 * 之前**：下面的追问判定 / 灯控 / 大模型三条出路都被这一次覆盖，所以每句话只发
 * 一遍 —— handle_light_intent() 里绝不能再发，否则老人说"开灯"时界面会看到两遍。
 *
 * 两条路一起走（和 voice_state_tick 同一个理由）：
 *   1) 同进程直调 robot_ui_bridge_voice_user_said()：本机界面立刻刷屏，不依赖网络；
 *   2) MQTT 照发：手机/PC 端还订阅着这个主题（它们不在这个地址空间里）。
 * 任何一条失败都只打日志：界面少显示一句绝不能反过来影响 ASR -> 大模型这条链路。
 *
 * @param  text  非空的 ASR 原文
 */

static void publish_user_said(const char *text)
{
  char said[USER_SAID_MAX];
  size_t text_len;
  size_t said_len;
  int ret = -ENOTCONN;

  if (text == NULL || text[0] == '\0')
    {
      return;
    }

  text_len = strlen(text);
  said_len = user_said_cut(said, sizeof(said), text);

  if (said_len < text_len)
    {
      printf("[语音] 用户原话过长(%zu 字节)，回传界面按整字截到 %zu 字节\n",
             text_len, said_len);
    }

  /* 主路：同进程直调。清洗（emoji/markdown -> 字库能渲染的纯文本）在界面那边
   * 做（robot_ui_bridge.c 里过 main.c 的 sanitize_for_display），这里给原文。 */

  robot_ui_bridge_voice_user_said(said);

  /* 旁路：MQTT，给手机/PC 端。没连上就只记日志 —— 界面已经刷过了。 */

  if (g_net_started)
    {
      ret = ai_network_publish_command(&g_net_ctx, AI_CMD_ACTION_USER_SAID,
                                       said);
    }

  if (ret < 0)
    {
      printf("[语音] user_said MQTT 回传失败(%d)（界面已由直调刷过，手机端看不到）:"
             " %s\n", ret, said);
    }
  else
    {
      printf("[语音] user_said 已回传: %s\n", said);
    }
}

/**
 * @brief  处理AI对话（在AI_TALKING状态调用）
 */

static void process_ai_dialogue(sm_context_t *ctx)
{
  char text_buf[512] = {0};
  light_intent_t light_intent;
  size_t frames;
  int ret;

  /* 异常声追问流程（W1）：这一轮的识别结果不送大模型，只用来判定
   * "要不要真报警"。判定和收尾都在 ask_flow_tick() 里做（那个线程独占相位），
   * 这里只把文本送过去。 */
  bool ask_mode = (g_ask_phase == ASK_PHASE_LISTENING);

  /* 累积帧数取一次快照，后面的判断和送 ASR 都用它 —— 让路线程的"收回"
   * （start_audio_listening 之前要清场，见 ai_companion_listen_hold_impl）会在
   * **别的线程**里把 g_speech_frames 清零，读到一半被清就会出现"判断时有数据、
   * 送 ASR 时长度变 0"这种自相矛盾的一轮。快照只影响这一次识别（用的是旧值），
   * 不会越界：缓冲是常驻的，帧数只会变小。 */

  frames = g_speech_frames;

  printf("[AI] 开始AI对话处理 (累积 %zu 帧音频)\n", frames);

  /* 如果没有累积到足够的语音数据，跳过 ASR */

  if (g_speech_buf == NULL || frames < 1600)
    {
      printf("[AI] 语音数据不足，跳过\n");

      if (ask_mode)
        {
          ask_flow_mark_answer(NULL);
        }
      else
        {
          sm_handle_event(ctx, SM_EVENT_AI_ERROR);
        }

      return;
    }

  /* ASR: 语音转文字 */

  printf("[AI] 正在进行语音识别...\n");
  ret = voice_asr_recognize((const unsigned char *)g_speech_buf,
                            frames * sizeof(int16_t),
                            text_buf, sizeof(text_buf));
  if (ret < 0)
    {
      printf("[AI] 语音识别失败: %d\n", ret);

      if (ask_mode)
        {
          ask_flow_mark_answer(NULL);
        }
      else
        {
          sm_handle_event(ctx, SM_EVENT_AI_ERROR);
        }

      return;
    }

  if (!user_said_has_content(text_buf))
    {
      /* 空串、或者只有空白/标点（一声异响被 VAD 当成话切出来时很常见）：
       * 当没听见。既不回传界面、也不送大模型，照原来"识别为空"的收尾走 ——
       * 追问流程记一笔空回答，常听链路由 AI_ERROR 退回 IDLE，随后接着听。 */

      printf("[AI] 语音识别结果为空(或只有空白/标点)，当没听见\n");

      if (ask_mode)
        {
          ask_flow_mark_answer(NULL);
        }
      else
        {
          sm_handle_event(ctx, SM_EVENT_AI_ERROR);
        }

      return;
    }

  printf("[AI] 识别结果: %s\n", text_buf);

  /* 用户原话回传界面：全文件唯一的发送点。放在这里（所有分支之前）是因为
   * 下面三条出路 —— 追问判定 / 灯控 / 大模型 —— 都被这一次覆盖：
   *   - 灯控命中那一轮走的也是这一行，所以 handle_light_intent() 里不能再发，
   *     否则老人说"开灯"时界面会把这句显示两遍；
   *   - 追问确认那一轮里老人的回答同样是"用户说的话"，也该上屏，所以这里
   *     没有放在 ask_mode 分支之后。
   * 判定用的还是同一份 text_buf，ask_flow_mark_answer() 拿到的文本没有变化。 */

  publish_user_said(text_buf);

  if (ask_mode)
    {
      ask_flow_mark_answer(text_buf);
      return;
    }

  /* 本机灯控意图：命中就直接做完（device_cmd + 回话 + 播报），
   * 这一轮不再送大模型；拿不准的（否定/歧义）返回 NONE，照原路送大模型。 */

  light_intent = light_intent_classify(text_buf);
  if (light_intent == LIGHT_INTENT_ON || light_intent == LIGHT_INTENT_OFF)
    {
      handle_light_intent(ctx, light_intent);
      return;
    }

  printf("[意图] 灯控未命中，交给大模型\n");

  /* LLM: 发送文字到大模型 */

  if (g_net_started)
    {
      /* 界面上换成"思考中"的表情（回复正文到手时会再发 ai_reply） */
      ai_network_send_face(&g_net_ctx, AI_CMD_FACE_THINKING);
    }

  ret = llm_send_text(&g_llm_ctx, text_buf,
                      NULL, llm_complete_callback, ctx);
  if (ret < 0)
    {
      printf("[AI] 发送LLM请求失败: %d\n", ret);
      sm_handle_event(ctx, SM_EVENT_AI_ERROR);
    }
}

/**
 * @brief  主循环任务
 */

static void *main_loop_task(void *arg)
{
  sm_context_t *ctx = (sm_context_t *)arg;

  printf("主循环任务启动\n");

  while (g_running)
    {
      /* 1. 运行状态机（检查超时等） */

      sm_run(ctx);

      /* 2. 轮询检测各种事件 */

      simulate_wakeup_detection(ctx);
      simulate_alarm_detection(ctx);
      simulate_care_timer(ctx);

      /* 2.5 唤醒词命中处理：录音线程只在回调里置标志，流程在这里推
       *     （没有模板时这个函数每轮读一个 false 就返回） */

      kws_wake_tick(ctx);

      /* 3. 异常声追问流程（限时听 / 重问 / 超时都在这里推进） */

      ask_flow_tick(ctx);

      /* 4. 语音状态有变化就告诉界面（我在听 / 正在想 / 正在说 / 空闲） */

      voice_state_tick(ctx);

      /* 5. 跨 app 让路请求：robot_ui 播提醒/提示音前请我们交出麦克风。
       *    绝不在**别人的**线程里动我们的设备和录音线程（理由见
       *    ai_companion_yield.h 头上那次事故）。设备动作归 hello_app 自己：
       *    正常情况由让路线程做（本线程这一拍只负责唤醒它，一个设备动作都不做 ——
       *    所以本线程被 ASR/TTS 阻塞几十秒也不影响让路）；让路线程没起来时
       *    才退回在本线程里做（见 mic_hold_tick）。 */

      mic_hold_tick();

      /* 6. 监听守护：录音线程死了（驱动 STOP / 5 秒 DMA 超时）就按退避重启，
       *    否则应用会一直跑着但永久听不到声音 */

      listen_supervise_tick(ctx);

      /* 7. 休眠等待 */

      usleep(MAIN_LOOP_INTERVAL_MS * 1000);
    }

  printf("主循环任务退出\n");
  return NULL;
}

/**
 * @brief  状态机调试输出
 */

static void print_system_status(sm_context_t *ctx)
{
  printf("\n========================================\n");
  printf("  智爱陪伴 - AI老人陪伴守护终端\n");
  printf("========================================\n");
  printf("  当前状态: %s\n", sm_get_state_name(ctx->current_state));
  printf("  上一状态: %s\n", sm_get_state_name(ctx->prev_state));
  printf("  最后事件: %s\n", sm_get_event_name(ctx->last_event));
  printf("  状态机状态: %s\n",
         ctx->initialized ? "已初始化" : "未初始化");
  printf("========================================\n\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**
 * @brief  主函数入口
 */

int main(int argc, char *argv[])
{
  int ret;
  pthread_t loop_thread;
  const char *startup_text = NULL;
  const char *mqtt_client_id = NULL;
  bool sound_self_test = false;
  bool ask_self_test = false;

  g_running = 1;

  for (int i = 1; i < argc; i++)
    {
      if (strcmp(argv[i], "--help") == 0)
        {
          print_usage(argv[0]);
          return 0;
        }
      else if (strcmp(argv[i], "--ask") == 0 && i + 1 < argc)
        {
          startup_text = argv[++i];
        }
      else if (strcmp(argv[i], "--client-id") == 0 && i + 1 < argc)
        {
          mqtt_client_id = argv[++i];
        }
      else if (strcmp(argv[i], "--sound-self-test") == 0)
        {
          sound_self_test = true;
        }
      else if (strcmp(argv[i], "--ask-self-test") == 0)
        {
          ask_self_test = true;
        }
      else
        {
          printf("未知或不完整的参数: %s\n", argv[i]);
          print_usage(argv[0]);
          return -EINVAL;
        }
    }

  printf("\n");
  printf("╔══════════════════════════════════════════╗\n");
  printf("║  智爱陪伴 - AI老人陪伴守护终端 v1.0     ║\n");
  printf("║  Powered by openvela                    ║\n");
  printf("╚══════════════════════════════════════════╝\n");
  printf("\n");

  /* 注册信号处理 */

  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  /* 1. 初始化状态机 */

  printf("[初始化] 正在初始化状态机...\n");
  ret = sm_init(&g_sm_ctx);
  if (ret < 0)
    {
      printf("[错误] 状态机初始化失败: %d\n", ret);
      return ret;
    }

  /* 2. 初始化音频模块 */

  printf("[初始化] 正在初始化音频模块...\n");
  ret = audio_init(&g_audio_ctx, NULL);
  if (ret < 0)
    {
      printf("[错误] 音频模块初始化失败: %d\n", ret);
      sm_deinit(&g_sm_ctx);
      return ret;
    }

  /* 3. 初始化LLM模块 */

  printf("[初始化] 正在初始化AI对话模块...\n");
  ret = llm_init(&g_llm_ctx, NULL);
  if (ret < 0)
    {
      printf("[错误] AI对话模块初始化失败: %d\n", ret);
      audio_deinit(&g_audio_ctx);
      sm_deinit(&g_sm_ctx);
      return ret;
    }

  /* 3.5 注册语音 ASR/TTS 后端
   *
   * 两套都注册，然后按配置选：
   *   - 配置里有非空的 api_key + llm_host（开机从 agent_config.json 拷进
   *     /data/ai_agent/config/config.json）-> 用小米 MiMo（mimo_voice.c）；
   *   - 否则回落到火山引擎（volc_asr/volc_tts，需要 volc_api_key 等凭据）。
   * 火山那条路保留不动：队友的凭据以后可能还会用。
   */

  printf("[初始化] 正在注册语音后端...\n");
  mimo_asr_register();
  mimo_tts_register();
  volc_asr_register();
  volc_tts_register();

  if (mimo_voice_available())
    {
      if (voice_asr_set_backend("mimo") == 0
          && voice_tts_set_backend("mimo") == 0)
        {
          printf("[初始化] 语音后端: MiMo (mimo)\n");
        }
      else
        {
          printf("[警告] MiMo 后端注册异常，回落到火山引擎\n");
          voice_asr_set_backend("volcengine");
          voice_tts_set_backend("volcengine");
        }
    }
  else
    {
      printf("[初始化] 语音后端: 火山引擎 (volcengine)"
             "（配置里没有 api_key + llm_host）\n");
      voice_asr_set_backend("volcengine");
      voice_tts_set_backend("volcengine");
    }

  /* 分配语音段累积缓冲区 */

  g_speech_buf = malloc(SPEECH_BUF_MAX_FRAMES * sizeof(int16_t));
  if (g_speech_buf == NULL)
    {
      printf("[警告] 语音缓冲区分配失败\n");
    }

  /* 3.6 初始化唤醒词模块（「你好，openvela」/「Hello，openvela」）
   *
   * 必须在 start_audio_listening() 之前：录音线程一起来就会开始喂帧，
   * 而 kws_dtw 要求它的接口只被一个线程调（喂帧的那个），所以建表、
   * 读模板都得在录音线程存在之前做完。
   *
   * kws_init() 自己会去 /data/kws/ 把 slotN.tpl 读回来（文件名是模块里
   * 定死的 KWS_DATA_DIR + "slot%d.tpl"，路径不用我们拼），
   * 并且它是幂等的：建表只做一次，重复调用只是重新读一遍模板文件。
   *
   * ⚠ /data 在本板是 tmpfs，重启就空 —— 没有模板的情况下 kws_feed
   *   永远返回 0，我们把 g_kws_enabled 置 false 后**连喂都不喂**，
   *   整条链路就退回"只靠 VAD 触发"，和接线之前一模一样。
   *   模板怎么录：kws_dtw.h 的 kws_enroll()（现场跑一次，或以后从别处灌）。 */

  printf("[初始化] 正在初始化唤醒词模块...\n");
  ret = kws_init();
  if (ret < 0)
    {
      printf("[警告] 唤醒词模块初始化失败: %d（唤醒词不可用，本地语音链路照常）\n",
             ret);
    }

  g_kws_ready = kws_ready_count();
  g_kws_enabled = (g_kws_ready > 0);

  if (g_kws_enabled)
    {
      printf("[KWS] 唤醒词就绪：%d 条模板（词：你好，openvela / Hello，openvela）\n",
             g_kws_ready);
    }
  else
    {
      printf("[KWS] 没有模板（%s 是空的，/data 是 tmpfs 重启就丢）→ "
             "唤醒词功能未启用，只靠 VAD 触发，行为与以前一致\n",
             KWS_DATA_DIR);
    }

  /* 设置 AI 对话回调给状态机 (通过 user_data) */

  sm_set_user_data(&g_sm_ctx, (void *)process_ai_dialogue);

  /* 4. 初始化声音检测模块 */

  printf("[初始化] 正在初始化声音检测模块...\n");
  ret = start_sound_detection(&g_sm_ctx);
  if (ret < 0)
    {
      printf("[警告] 声音检测初始化失败: %d\n", ret);
      /* 声音检测失败不退出，继续运行 */
    }

  /* 5. 初始化主动关怀模块 */

  printf("[初始化] 正在初始化主动关怀模块...\n");
  ret = start_care(&g_sm_ctx);
  if (ret < 0)
    {
      printf("[警告] 主动关怀初始化失败: %d\n", ret);
      /* 关怀失败不退出，继续运行 */
    }

  /* 6. 初始化网络模块（**给手机/PC 端的旁路**：AI 回复正文 / 用户原话 /
   *    状态 / 表情 / 报警页从这里 publish 到 zhi_ai/<client_id>/command）
   *
   * 本机界面已经走同进程直调（robot_ui_bridge.h）了，所以这一步失败**不再**
   * 等于"界面没反应" —— 它只影响远端。原来界面反馈全押在这条路上，而开机时
   * RNDIS/DNS 常常还没就绪（真机日志：`MQTT DNS 解析失败` -> `Error 101` ->
   * 这里打"网络初始化失败"），于是 g_net_started 一直是 false、界面整场不动。
   *
   * 不用 ai_network_init()：network_comm.c 在一个固件里只有一份实例，
   * 界面（robot_ui）的 network_task 已经拥有那条 MQTT socket，
   * ai_network_init() 会把它的全局状态 memset 掉、还会把 MQTT 收包回调抢过来，
   * 结果是界面再也收不到 ai_reply。ai_network_start_shared() 只复用那条连接
   * （没人连的时候才兜底连一次），理由写在 ai_network.c 里。 */

  printf("[初始化] 正在初始化网络模块...\n");
  ret = ai_network_start_shared(&g_net_ctx, mqtt_client_id);
  if (ret < 0)
    {
      printf("[警告] 网络初始化失败: %d（手机/PC 端收不到回传，本机界面走直调"
             "不受影响）\n", ret);
    }
  else
    {
      g_net_started = true;
      printf("[网络] 回传主题: zhi_ai/%s/command\n",
             ai_network_get_client_id(&g_net_ctx));
    }

  /* 7. 把「我们自己的工具」挂进框架的工具表
   *
   * 走 tool_registry_register_provider() 这条**包外注册**的公开接口：
   * 框架的工具表长在 ai_agent 里，但它是 CONFIG_BUILD_FLAT 的单地址空间，
   * 我们可以在自己进程里把 provider 注册进去，不用改 ai_agent 源码、不用出补丁。
   *
   * ⚠️ 模型"看得见"这些工具的前提是 **ai_agent 那个 app 已经在跑**
   * （本固件没有开机自启，要手工敲 ai_agent）。这跟 llm_send_text() 能不能用
   * 是同一个前提，不是新增的依赖。注册动作本身不依赖它：tool_registry_init()
   * 不清 provider 数组，先注册后启动也照样带上。
   *
   * 传的是 start_shared 的 ctx：那条 MQTT 连接归界面（robot_ui）所有，
   * 工具发 device_cmd 就是借它发。 */

  ai_tools_provider_set_network_ctx(&g_net_ctx);

  ret = ai_tools_provider_init();
  if (ret < 0)
    {
      printf("[工具] 注册到框架工具表失败: %d（模型看不到我们的工具，"
             "本地语音链路不受影响）\n", ret);
    }
  else
    {
      printf("[工具] 已注册到框架工具表: %s\n", AI_TOOL_SET_LIGHT);
    }

  /* 打印初始状态 */

  print_system_status(&g_sm_ctx);

  /* 5. 启动语音监听 */

  printf("[启动] 正在启动语音监听...\n");

  /* 先把"要常听"的意图立起来再开麦：这一次就是失败（设备忙 / 驱动没起来），
   * 主循环里的监听守护也会照着重试，而不是让应用安静地聋到下次重启。 */

  g_listen_wanted = true;

  ret = start_audio_listening(&g_sm_ctx);
  if (ret < 0)
    {
      printf("[警告] 启动语音监听失败: %d（监听守护会重试）\n", ret);
    }

  /* 1.5 启动让路线程（robot_ui 播提醒/提示音前请我们交出麦克风的受理者）
   *
   * 位置很关键，必须在下面 pthread_create(main_loop_task) **之前**：
   * 让路线程"已就绪"这件事得在主循环开始调 mic_hold_tick() 之前定下来，
   * 否则主循环可能先在它自己线程里替让路把设备动作做了（两条线程都动设备）。
   * 设备已经就绪（上面 start_audio_listening 刚跑过），线程起来就能干活。
   *
   * 起不来（sem_init / pthread_create 失败）不拦启动：yield_worker_start() 会打
   * 一行日志，mic_hold_tick() 退回原来的做法（在主循环线程里处理请求）。 */

  printf("[启动] 正在启动让路线程...\n");
  if (!yield_worker_start())
    {
      printf("[警告] 让路线程没起来：让路退回主循环处理（功能照旧，只是主循环"
             "被 ASR/TTS 阻塞时提示音可能等不到让路）\n");
    }

  /* 2. 启动主循环任务 */

  printf("[启动] 正在启动主循环任务...\n");
  ret = pthread_create(&loop_thread, NULL, main_loop_task, &g_sm_ctx);
  if (ret != 0)
    {
      printf("[错误] 主循环任务创建失败: %d\n", ret);

      /* 让路线程也要收掉，且必须在 stop_audio_listening() **之前**：它可能正拿着
       * 设备开关，两边同时上手就是这次事故的形状（见 yield_worker_stop）。 */

      g_running = 0;
      yield_worker_stop();
      stop_audio_listening();
      stop_care();
      stop_sound_detection();
      llm_deinit(&g_llm_ctx);
      audio_deinit(&g_audio_ctx);
      sm_deinit(&g_sm_ctx);
      return -ret;
    }

  printf("[运行] 系统已启动, 按 Ctrl+C 退出\n\n");

  if (startup_text != NULL)
    {
      ret = llm_send_text(&g_llm_ctx, startup_text, NULL,
                          llm_complete_callback, &g_sm_ctx);
      if (ret < 0)
        {
          printf("[LLM] 启动请求失败: %d\n", ret);
        }
    }

  if (sound_self_test && g_sound_started)
    {
      int16_t *test_audio = calloc(SOUND_DETECT_FRAMES_PER_WINDOW,
                                   sizeof(*test_audio));
      if (test_audio == NULL)
        {
          printf("[自检] 内存不足，跳过测试\n");
        }
      else
        {
      for (size_t i = 0; i < SOUND_DETECT_FRAMES_PER_WINDOW; i += 80)
        {
          test_audio[i] = (i / 80) % 2 == 0 ? INT16_MAX : INT16_MIN;
        }

      ret = sound_detect_feed(&g_sound_ctx, test_audio,
                              SOUND_DETECT_FRAMES_PER_WINDOW);
      printf("[自检] 声音检测测试数据已注入: %s (%d)\n",
             ret == OK ? "成功" : "失败", ret);
          free(test_audio);
        }
    }

  if (ask_self_test && g_sound_started)
    {
      /* 真机上不摔一跤也能验证整条追问链路：
       *   NSH 里 `ai_companion --ask-self-test`
       * 灌一条假的"异常声"进去，追问流程会自己从主循环里跑起来
       * （问一句 → 限时听 6 秒 → 按回答判定/重问）。 */
      ret = sound_detect_report_anomaly(&g_sound_ctx, SOUND_TYPE_KNOCK, 0.9f);
      printf("[自检] 已注入异常声(KNOCK 0.9): %s (%d)\n",
             ret == OK ? "成功" : "失败", ret);
    }

  /* 3. 主线程等待退出 */

  while (g_running)
    {
      sleep(1);
    }

  /* 4. 清理退出 */

  printf("\n[退出] 正在停止系统...\n");

  /* 先收让路线程（g_running 已经是 0，它最多 100ms 就自己退出；要是在动设备，
   * 会先把那一手做完）。**必须在下面 stop_audio_listening() 之前**：两边同时
   * 开关录音设备就是这次事故的形状（跨线程同时上手同一个设备）。 */

  yield_worker_stop();

  /* 停止主动关怀 */

  stop_care();

  /* 先等待录音线程退出，确保不会再调用 sound_detect_feed。 */

  stop_audio_listening();

  /* 然后释放声音检测器。 */

  stop_sound_detection();

  /* 等待主循环任务退出 */

  pthread_join(loop_thread, NULL);

  /* 反初始化LLM模块 */

  llm_deinit(&g_llm_ctx);

  /* 释放语音缓冲区 */

  free(g_speech_buf);
  g_speech_buf = NULL;

  /* 反初始化音频模块 */

  audio_deinit(&g_audio_ctx);

  /* 反初始化状态机 */

  sm_deinit(&g_sm_ctx);

  /* 网络：只关掉自己这一侧的上下文。
   * 那条 MQTT 连接是界面（robot_ui）的 network_task 建的、由它重连/收包，
   * 这里不能去断它 —— 断了界面也跟着掉线。 */

  g_net_started = false;
  memset(&g_net_ctx, 0, sizeof(g_net_ctx));

  printf("[退出] 系统已安全退出\n\n");

  return 0;
}
