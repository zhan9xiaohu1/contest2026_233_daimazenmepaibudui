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

/* 机器人状态 */
typedef enum {
    ROBOT_STATUS_IDLE,      // 待机
    ROBOT_STATUS_LISTENING, // 监听
    ROBOT_STATUS_SPEAKING,  // 说话
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

/* 更新状态 */
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
