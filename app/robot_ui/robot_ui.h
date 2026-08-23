/**
 * robot_ui.h - 智爱陪伴机器人界面头文件
 * SF32LB52-DevKit-LCD LVGL 界面开发
 */

#ifndef __ROBOT_UI_H
#define __ROBOT_UI_H

#include <lvgl.h>

/* 机器人表情类型 */
typedef enum {
    ROBOT表情_HAPPY,      // 开心
    ROBOT表情_THINKING,   // 思考
    ROBOT表情_SLEEPY,     // 困倦
    ROBOT表情_SURPRISED,  // 惊讶
    ROBOT表情_WORRIED,    // 担心
    ROBOT表情_ALARM,      // 报警
    ROBOT表情_MAX
} robot表情_t;

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
void robot_ui_set表情(robot表情_t表情);

/* 更新 AI 回复 */
void robot_ui_set_ai_reply(const char *text);

/* 更新状态 */
void robot_ui_set_status(robot_status_t status);

/* 显示提醒 */
void robot_ui_show_reminder(const char *title, const char *content);

/* 显示报警 */
void robot_ui_show_alarm(const char *content);

/* 关闭报警 */
void robot_ui_close_alarm(void);

/* 切换界面 */
void robot_ui_switch_view(ui_view_t view);

#endif /* __ROBOT_UI_H */
