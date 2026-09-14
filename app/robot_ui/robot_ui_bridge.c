/**
 * robot_ui_bridge.c - 语音链路推界面的同进程直调实现（编在 robot_ui 这个 app 里）
 *
 * 头文件里写了"为什么要有这个文件"。这里只补三件实现上的事：
 *
 * 1) 本文件里**没有一处**直接碰 LVGL 控件。四个入口只做
 *    「清洗文本 -> 调 touch_ui_xxx() / robot_ui_bridge_post_status()」，
 *    后两者内部全是 lv_async_call，真正的控件操作发生在 LVGL 线程。
 *    所以 ai_companion 的录音线程、主循环任务随便调都不会踩
 *    `_lv_inv_area: Invalidate area is not allowed during rendering` 那个断言
 *    （2026-09-14 就是被这个断言把整个 app 打死的）。
 *
 * 2) 闸门（g_ui_ready）之外还有一道 up_interrupt_context() 检查：万一以后有人
 *    图省事在音频回调/中断里调进来，宁可丢一次显示也不能在中断里 malloc + 取锁。
 *
 * 3) 顺序有讲究：先把面板投出去、再投文字。两者都是 lv_async_call，投递顺序
 *    就是执行顺序；反过来先写后弹，"你说：…"会落在面板还没建好的空档里丢掉。
 */

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <stdio.h>
#include <string.h>
#include <stddef.h>

#include "robot_ui_bridge.h"

#include "robot_ui.h"
#include "touch_ui.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* 界面就绪标志。robot_ui 的 main() 建完控件才置起来（见头文件）。
 * volatile：一个任务写、别的任务读，不加锁 —— 它只是一位"开关"，
 * 读到旧值最坏是"这一次推送被丢掉"，不会读到半截值。 */
static volatile bool g_ui_ready;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**
 * @brief  这次推送能不能做（闸门 + 中断上下文）
 *
 * why = 不能做的原因，给日志用；能返回 true 时 why 不动。
 */

static bool bridge_can_push(const char **why)
{
  if (!g_ui_ready)
    {
      *why = "界面还没就绪";
      return false;
    }

  if (up_interrupt_context())
    {
      *why = "在中断上下文里，不能投界面";
      return false;
    }

  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void robot_ui_bridge_set_ready(bool ready)
{
  g_ui_ready = ready;
  printf("[Bridge] 语音直调通道%s\n", ready ? "已就绪" : "关闭");
}

bool robot_ui_bridge_is_ready(void)
{
  return g_ui_ready;
}

/**
 * @brief  显示用户刚说的话（ASR 原文）
 */

void robot_ui_bridge_voice_user_said(const char *text)
{
  char shown[ROBOT_UI_BRIDGE_TEXT_MAX];
  const char *why = NULL;

  if (text == NULL || text[0] == '\0')
    {
      return;
    }

  if (!bridge_can_push(&why))
    {
      printf("[Bridge] user_said 没上屏（%s）: %s\n", why, text);
      return;
    }

  /* 先弹面板再写文字：面板没开着的话 touch_ui_set_voice_user_text()
   * 本身是空操作，这条文字就没了 —— 顺序不能反。 */

  robot_ui_bridge_panel_autoshow();

  if (robot_ui_bridge_sanitize_text(text, shown, sizeof(shown)) == 0)
    {
      /* 整句都被清洗掉了（比如只订了 emoji）：界面不动，别拿空串去刷对话区
       * —— 那会把上一句擦掉，看着像面板坏了。 */
      printf("[Bridge] user_said 清洗后为空，界面不动\n");
      return;
    }

  touch_ui_set_voice_user_text(shown);
  printf("[Bridge] 用户原话已上屏: %s\n", shown);
}

/**
 * @brief  显示 AI 的回复正文
 */

void robot_ui_bridge_voice_reply(const char *text)
{
  char shown[ROBOT_UI_BRIDGE_TEXT_MAX];
  const char *why = NULL;

  if (text == NULL || text[0] == '\0')
    {
      return;
    }

  if (!bridge_can_push(&why))
    {
      printf("[Bridge] ai_reply 没上屏（%s）: %s\n", why, text);
      return;
    }

  if (robot_ui_bridge_sanitize_text(text, shown, sizeof(shown)) == 0)
    {
      printf("[Bridge] ai_reply 清洗后为空，界面不动\n");
      return;
    }

  /* 和 MQTT 那条路一致：状态栏/表情按"正在回复"走，对话区写进主屏。
   * 状态行（面板上那一行）不在这里改 —— 那是 voice_state 的事，保持一个来源。 */

  robot_ui_bridge_post_status(ROBOT_STATUS_SPEAKING, ROBOT_FACE_HAPPY, shown);

  /* 镜像面板开着时，同一段回复也写进面板的对话区（没开就是空操作） */
  touch_ui_set_voice_reply(shown);

  printf("[Bridge] AI 回复已上屏: %s\n", shown);
}

/**
 * @brief  更新语音状态
 */

void robot_ui_bridge_voice_state(int state)
{
  const char *why = NULL;

  if (!bridge_can_push(&why))
    {
      printf("[Bridge] voice_state=%d 没上屏（%s）\n", state, why);
      return;
    }

  /* 映射规则与 robot_ui/main.c 的 MQTT 分支逐字一致：
   *   listening 状态栏[聆听中] 普通表情 / thinking 状态栏[聆听中] 思考表情
   *   speaking  状态栏[回复中] 普通表情 / idle     状态栏[在线]   普通表情
   * （界面没有"思考中"这一档状态栏，thinking 保持[聆听中]，不然会闪回[在线]）
   *
   * 「在说」要把语音镜像面板弹出来（MQTT 那条路也是这么做的，见
   * on_ai_command_received 的 voice_state 分支）：面板不弹出来，老人根本
   * 不知道要去哪里看"它听清了什么、回了什么"。
   *
   * ⚠️ **「在听」绝对不能弹**：hello_app 是**开机自启**的，它一启动就进入
   * "我在听"并推一条 LISTENING —— 那样开机第一眼看到的就是语音面板，而不是
   * 主菜单（用户实测反馈："为什么一开机就是语音聊天页面？我希望看到主菜单"）。
   * 现在的策略：只在"确实有语音活动"时才弹 ——
   *   - 用户说的话到了（`robot_ui_bridge_voice_user_said()` → 界面侧那处 autoshow）
   *   - 机器人正在回话（SPEAKING，下面这一处）
   * thinking 也不弹（识别一出来就先走 user_said 那处）。
   * 抑制规则（用户关过就不再弹）在 voice_mirror_autoshow 里。 */

  switch (state)
    {
      case ROBOT_UI_BRIDGE_VOICE_LISTENING:
        robot_ui_bridge_post_status(ROBOT_STATUS_LISTENING, ROBOT_FACE_HAPPY,
                                    NULL);
        touch_ui_set_voice_state(TOUCH_VOICE_STATE_LISTENING);
        break;

      case ROBOT_UI_BRIDGE_VOICE_THINKING:
        robot_ui_bridge_post_status(ROBOT_STATUS_LISTENING, ROBOT_FACE_THINKING,
                                    NULL);
        touch_ui_set_voice_state(TOUCH_VOICE_STATE_THINKING);
        break;

      case ROBOT_UI_BRIDGE_VOICE_SPEAKING:
        robot_ui_bridge_panel_autoshow();
        robot_ui_bridge_post_status(ROBOT_STATUS_SPEAKING, ROBOT_FACE_HAPPY,
                                    NULL);
        touch_ui_set_voice_state(TOUCH_VOICE_STATE_SPEAKING);
        break;

      case ROBOT_UI_BRIDGE_VOICE_IDLE:
        robot_ui_bridge_post_status(ROBOT_STATUS_IDLE, ROBOT_FACE_HAPPY, NULL);
        touch_ui_set_voice_state(TOUCH_VOICE_STATE_IDLE);
        break;

      default:
        /* 不认识的状态不改界面：宁可少刷一次，也不要把界面停在一个错的字上 */
        printf("[Bridge] voice_state 不认识的档位: %d\n", state);
        break;
    }
}

/**
 * @brief  自动弹出语音镜像面板（按界面那边的抑制规则）
 */

void robot_ui_bridge_voice_panel_open(void)
{
  const char *why = NULL;

  if (!bridge_can_push(&why))
    {
      printf("[Bridge] 弹面板被挡（%s）\n", why);
      return;
    }

  robot_ui_bridge_panel_autoshow();
}
