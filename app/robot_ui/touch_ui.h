/**
 * touch_ui.h - 老人友好触摸交互界面
 * SF32LB52-DevKit-LCD LVGL 界面开发
 */

#ifndef TOUCH_UI_H
#define TOUCH_UI_H

#include <lvgl.h>
#include <stdint.h>
#include <stdbool.h>

/* ==================== 菜单类型 ==================== */
typedef enum {
    MENU_TYPE_MAIN,         // 主菜单
    MENU_TYPE_REMIND,       // 提醒菜单
    MENU_TYPE_SETTING,      // 设置菜单
    MENU_TYPE_ABOUT,        // 关于
    MENU_TYPE_MAX
} menu_type_t;

/* ==================== 模式类型 ==================== */
typedef enum {
    MODE_NORMAL,            // 正常模式
    MODE_LISTENING,         // 监听模式
    MODE_SLEEP,             // 休眠模式
    MODE_ALARM,             // 报警模式
    MODE_MAX
} robot_mode_t;

/* ==================== 设置项 ==================== */
typedef struct {
    uint8_t volume;         // 音量 0-100
    uint8_t brightness;     // 亮度 0-100
    bool auto_remind;       // 自动提醒开关
    uint16_t remind_interval; // 提醒间隔（分钟）
} settings_t;

/* ==================== 初始化 ==================== */
void touch_ui_init(void);

/* ==================== 菜单操作 ==================== */
void touch_ui_show_menu(menu_type_t type);
void touch_ui_hide_menu(void);
void touch_ui_go_back(void);

/* ==================== 模式切换 ==================== */
void touch_ui_set_mode(robot_mode_t mode);
robot_mode_t touch_ui_get_mode(void);

/* ==================== 设置操作 ==================== */
settings_t* touch_ui_get_settings(void);
void touch_ui_save_settings(void);
void touch_ui_show_setting_detail(const char *title, const char *content);

/* ==================== 提醒操作 ==================== */
void touch_ui_add_reminder(const char *title, const char *time);
void touch_ui_show_reminder_list(void);
void touch_ui_clear_reminders(void);

/* ==================== 触摸反馈 ==================== */
void touch_ui_vibrate(int duration_ms);
void touch_ui_play_sound(const char *sound_type);

#endif /* TOUCH_UI_H */
