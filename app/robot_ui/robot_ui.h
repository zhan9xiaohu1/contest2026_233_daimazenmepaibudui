/**
 * robot_ui.h - 智爱陪伴机器人界面头文件
 * SF32LB52-DevKit-LCD LVGL 界面开发
 */

#ifndef ROBOT_UI_H
#define ROBOT_UI_H

#include <lvgl.h>

/* 机器人表情类型 */
typedef enum {
    ROBOT_FACE_HAPPY,      // 开心
    ROBOT_FACE_THINKING,   // 思考
    ROBOT_FACE_SLEEPY,     // 困倦
    ROBOT_FACE_SURPRISED,  // 惊讶
    ROBOT_FACE_WORRIED,    // 担心
    ROBOT_FACE_ALARM,      // 报警
    ROBOT_FACE_MAX
} robot_face_t;

/* 机器人状态。
 *
 * ⚠️ 这是个**显示用**的档位，不是新的一路状态来源：语音那四档（听/想/说/空闲）
 * 全部来自 hello_app 的 voice_state_probe()，经 ui_post() 投到 LVGL 线程后
 * 由 robot_ui_set_status() 落到主界面「AI 回复区」那一行小字上。
 *
 * THINKING 是 2026-09-16 补的一档：原来"在想"借的是 LISTENING（只靠表情区分），
 * 主界面那一行小字就没法把"在听"和"在想"分开 —— 用户要的正是这四档各显各的。
 * 加在 LISTENING 后面只是为了读起来顺；这些值不落盘、不上网、也没有任何数组按
 * 它下标，所以挪位置是安全的（全仓库只有 robot_ui.c 和 main.c 那几处 switch/投递）。 */
typedef enum {
    ROBOT_STATUS_IDLE,      // 待机（空闲）
    ROBOT_STATUS_LISTENING, // 监听（在听）
    ROBOT_STATUS_THINKING,  // 识别/思考中（在想）
    ROBOT_STATUS_SPEAKING,  // 说话（在说）
    ROBOT_STATUS_REMINDING, // 提醒中
    ROBOT_STATUS_ALARM,     // 报警
    ROBOT_STATUS_MAX
} robot_status_t;

/* 界面类型 */
typedef enum {
    UI_VIEW_MAIN,       // 主界面
    UI_VIEW_REMIND,     // 提醒界面
    UI_VIEW_ALARM,      // 报警界面
    UI_VIEW_SETTING,    // 设置界面
    UI_VIEW_MAX
} ui_view_t;

/* 初始化 UI */
void robot_ui_init(void);

/* 更新机器人表情 */
void robot_ui_set_face(robot_face_t face);

/* 更新 AI 回复 */
void robot_ui_set_ai_reply(const char *text);

/* 更新状态：写的是**主界面「AI 回复区」里那一行语音状态小字**
 * （create_ai_reply_area() 建的 lbl_status），不在状态栏上 ——
 * 状态栏那三样 2026-09-14 已经被用户要求删掉了。
 *
 * 空闲 -> 「空闲」灰 / 在听 -> 「在听…」蓝 / 在想 -> 「在想…」橙 /
 * 在说 -> 「在说…」绿；提醒中、报警两档借用同一行，颜色见 robot_ui.c。
 *
 * ⚠️ 只能在 LVGL 线程里调（它直接改控件）。别的线程一律走 main.c 的 ui_post() /
 * ui_post_*，内部是 lv_async_call —— 跨线程碰 LVGL 会踩 rendering_in_progress
 * 断言把 app 打死（2026-09-14 实测）。
 * ⚠️ 认不出来的档位一个字都不改（robot_ui.c 里那条 default 分支），
 * 宁可少刷一次，也不要把这一行停在一个错的字上。 */
void robot_ui_set_status(robot_status_t status);

/* 更新状态栏上的网络状态（ASCII，如 "NET OK" / "NET --"）。
 * 必须从 LVGL 所在的任务调用（main 的主循环），不要从 network_task 直接调。
 */
void robot_ui_set_net_status(const char *text);

/* 显示提醒 */
void robot_ui_show_reminder(const char *title, const char *content);

/* 报错提示页（顶层覆盖，一个大按钮点掉）。
 * ⚠️ 只能在 LVGL 线程里调：别的线程（语音工作线程 / MQTT 线程 / 播放线程）
 * 用 main.c 的 ui_post_error()，内部 lv_async_call 投过来 —— 直接跨线程碰
 * LVGL 会踩 "_lv_inv_area: Invalidate area is not allowed during rendering"
 * 断言，把整个 app 打死（2026-09-14 实测）。 */
void robot_ui_show_error(const char *title, const char *content);

/* 显示报警 */
void robot_ui_show_alarm(const char *content);

/* 关闭报警 */
void robot_ui_close_alarm(void);

/* 刷新状态栏时钟（从 main 主循环调用，与 LVGL 定时器双重保障） */
void robot_ui_update_time(void);

/* 切换界面 */
void robot_ui_switch_view(ui_view_t view);

#endif /* ROBOT_UI_H */
