/**
 * robot_ui.c - 智爱陪伴机器人界面实现
 * SF32LB52-DevKit-LCD LVGL 界面开发
 */

#include "robot_ui.h"
#include <stdio.h>
#include <string.h>

/* ==================== 全局变量 ==================== */
static lv_obj_t *scr_main = NULL;      // 主屏幕
static lv_obj_t *scr_alarm = NULL;     // 报警屏幕

/* 主界面组件 */
static lv_obj_t *lbl_status = NULL;    // 状态标签
static lv_obj_t *lbl_time = NULL;      // 时间标签
static lv_obj_t *lbl表情 = NULL;       // 表情标签
static lv_obj_t *lbl_ai_reply = NULL;  // AI回复标签
static lv_obj_t *lbl_reminder = NULL;  // 提醒标签

/* 按钮 */
static lv_obj_t *btn_remind = NULL;    // 提醒按钮
static lv_obj_t *btn_setting = NULL;   // 设置按钮
static lv_obj_t *btn_alarm = NULL;     // 报警按钮

/* 动画 */
static lv_anim_t anim表情 = {0};       // 表情动画
static lv_anim_t anim_blink = {0};     // 闪烁动画

/* 当前状态 */
static robot表情_t current表情 = ROBOT表情_HAPPY;
static robot_status_t current_status = ROBOT_STATUS_IDLE;

/* 样式 */
static lv_style_t style_bg;           // 背景样式
static lv_style_t style_btn;          // 按钮样式
static lv_style_t style_btn_alarm;    // 报警按钮样式
static lv_style_t style_text;         // 文字样式
static lv_style_t style表情;          // 表情样式

/* ==================== 表情数据 ==================== */
/* ASCII 表情 */
static const char *表情数组[] = {
    "(◕‿◕)",     // 开心
    "(◕_◕)",     // 思考
    "(◕‿◕)zZZ", // 困倦
    "(◕o◕)",     // 惊讶
    "(◕~◕)",     // 担心
    "(⚠️⚠️⚠️)",  // 报警
};

/* ==================== 内部函数声明 ==================== */
static void init_styles(void);
static void create_main_screen(void);
static void create_alarm_screen(void);
static void create_status_bar(lv_obj_t *parent);
static void create表情_area(lv_obj_t *parent);
static void create_ai_reply_area(lv_obj_t *parent);
static void create_bottom_buttons(lv_obj_t *parent);
static void btn_event_handler(lv_event_t *e);
static void anim表情_update(void *var, int32_t val);
static void anim_blink_update(void *var, int32_t val);

/* ==================== 初始化样式 ==================== */
static void init_styles(void)
{
    /* 背景样式 - 深蓝色 */
    lv_style_init(&style_bg);
    lv_style_set_bg_color(&style_bg, lv_color_hex(0x1A1A2E));
    lv_style_set_bg_opa(&style_bg, LV_OPA_COVER);
    lv_style_set_border_width(&style_bg, 0);
    lv_style_set_radius(&style_bg, 0);

    /* 按钮样式 - 绿色 */
    lv_style_init(&style_btn);
    lv_style_set_bg_color(&style_btn, lv_color_hex(0x4CAF50));
    lv_style_set_bg_opa(&style_btn, LV_OPA_COVER);
    lv_style_set_radius(&style_btn, 15);
    lv_style_set_shadow_width(&style_btn, 10);
    lv_style_set_shadow_color(&style_btn, lv_color_hex(0x388E3C));

    /* 报警按钮样式 - 红色 */
    lv_style_init(&style_btn_alarm);
    lv_style_set_bg_color(&style_btn_alarm, lv_color_hex(0xF44336));
    lv_style_set_bg_opa(&style_btn_alarm, LV_OPA_COVER);
    lv_style_set_radius(&style_btn_alarm, 15);
    lv_style_set_shadow_width(&style_btn_alarm, 10);
    lv_style_set_shadow_color(&style_btn_alarm, lv_color_hex(0xD32F2F));

    /* 文字样式 - 白色 */
    lv_style_init(&style_text);
    lv_style_set_text_color(&style_text, lv_color_hex(0xFFFFFF));
    lv_style_set_text_font(&style_text, &lv_font_montserrat_16);

    /* 表情样式 - 大号字体 */
    lv_style_init(&style表情);
    lv_style_set_text_color(&style表情, lv_color_hex(0xFFEB3B));
    lv_style_set_text_font(&style表情, &lv_font_montserrat_32);
}

/* ==================== 创建主屏幕 ==================== */
static void create_main_screen(void)
{
    /* 创建主屏幕 */
    scr_main = lv_obj_create(NULL);
    lv_obj_add_style(scr_main, &style_bg, 0);

    /* 创建状态栏 */
    create_status_bar(scr_main);

    /* 创建表情区域 */
    create表情_area(scr_main);

    /* 创建 AI 回复区域 */
    create_ai_reply_area(scr_main);

    /* 创建底部按钮 */
    create_bottom_buttons(scr_main);

    /* 加载主屏幕 */
    lv_scr_load(scr_main);
}

/* ==================== 创建状态栏 ==================== */
static void create_status_bar(lv_obj_t *parent)
{
    /* 状态栏容器 */
    lv_obj_t *bar = lv_obj_create(parent);
    lv_obj_set_size(bar, LV_PCT(100), 40);
    lv_obj_align(bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0x2D2D44), 0);
    lv_obj_set_style_border_width(bar, 0, 0);
    lv_obj_set_style_radius(bar, 0, 0);
    lv_obj_set_flex_flow(bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(bar, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    /* 状态文本 */
    lbl_status = lv_label_create(bar);
    lv_label_set_text(lbl_status, "● 在线");
    lv_obj_set_style_text_color(lbl_status, lv_color_hex(0x4CAF50), 0);
    lv_obj_set_style_text_font(lbl_status, &lv_font_montserrat_14, 0);

    /* 时间标签 */
    lbl_time = lv_label_create(bar);
    lv_label_set_text(lbl_time, "12:00");
    lv_obj_set_style_text_color(lbl_time, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(lbl_time, &lv_font_montserrat_14, 0);

    /* 电量/网络图标（简化为文字） */
    lv_obj_t *lbl_signal = lv_label_create(bar);
    lv_label_set_text(lbl_signal, "📶 100%");
    lv_obj_set_style_text_color(lbl_signal, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(lbl_signal, &lv_font_montserrat_14, 0);
}

/* ==================== 创建表情区域 ==================== */
static void create表情_area(lv_obj_t *parent)
{
    /* 表情容器 */
    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_set_size(container, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_align(container, LV_ALIGN_TOP_MID, 0, 60);
    lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(container, 0, 0);
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    /* 表情标签 */
    lbl表情 = lv_label_create(container);
    lv_label_set_text(lbl表情,表情数组[ROBOT表情_HAPPY]);
    lv_obj_add_style(lbl表情, &style表情, 0);
    lv_obj_set_style_text_font(lbl表情, &lv_font_montserrat_32, 0);

    /* 状态文字 */
    lbl_reminder = lv_label_create(container);
    lv_label_set_text(lbl_reminder, "我是智爱陪伴，很高兴见到你！");
    lv_obj_set_style_text_color(lbl_reminder, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(lbl_reminder, &lv_font_montserrat_14, 0);
    lv_obj_set_style_pad_top(lbl_reminder, 10, 0);

    /* 启动表情动画 */
    lv_anim_init(&anim表情);
    lv_anim_set_var(&anim表情, lbl表情);
    lv_anim_set_values(&anim表情, -10, 10);
    lv_anim_set_time(&anim表情, 1000);
    lv_anim_set_playback_time(&anim表情, 1000);
    lv_anim_set_repeat_count(&anim表情, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_exec_cb(&anim表情, (lv_anim_exec_xcb_t)lv_obj_set_y);
    lv_anim_start(&anim表情);
}

/* ==================== 创建 AI 回复区域 ==================== */
static void create_ai_reply_area(lv_obj_t *parent)
{
    /* AI 回复容器 */
    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_set_size(container, LV_PCT(90), 120);
    lv_obj_align(container, LV_ALIGN_CENTER, 0, 20);
    lv_obj_set_style_bg_color(container, lv_color_hex(0x2D2D44), 0);
    lv_obj_set_style_radius(container, 20, 0);
    lv_obj_set_style_border_width(container, 2, 0);
    lv_obj_set_style_border_color(container, lv_color_hex(0x4CAF50), 0);
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(container, 15, 0);

    /* AI 说话图标 */
    lv_obj_t *lbl_icon = lv_label_create(container);
    lv_label_set_text(lbl_icon, "🤖 AI");
    lv_obj_set_style_text_color(lbl_icon, lv_color_hex(0x4CAF50), 0);
    lv_obj_set_style_text_font(lbl_icon, &lv_font_montserrat_14, 0);

    /* AI 回复内容 */
    lbl_ai_reply = lv_label_create(container);
    lv_label_set_text(lbl_ai_reply, "你好！今天天气怎么样？\n有什么我可以帮你的吗？");
    lv_obj_set_style_text_color(lbl_ai_reply, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(lbl_ai_reply, &lv_font_montserrat_16, 0);
    lv_label_set_long_mode(lbl_ai_reply, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_ai_reply, LV_PCT(100));
    lv_obj_set_style_text_align(lbl_ai_reply, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_pad_top(lbl_ai_reply, 5, 0);
}

/* ==================== 创建底部按钮 ==================== */
static void create_bottom_buttons(lv_obj_t *parent)
{
    /* 按钮容器 */
    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_set_size(container, LV_PCT(100), 80);
    lv_obj_align(container, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(container, 0, 0);
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(container, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    /* 提醒按钮 */
    btn_remind = lv_btn_create(container);
    lv_obj_set_size(btn_remind, 90, 50);
    lv_obj_add_style(btn_remind, &style_btn, 0);
    lv_obj_add_event_cb(btn_remind, btn_event_handler, LV_EVENT_CLICKED, (void *)UI_VIEW_REMIND);
    lv_obj_t *lbl_btn1 = lv_label_create(btn_remind);
    lv_label_set_text(lbl_btn1, "📋 提醒");
    lv_obj_set_style_text_font(lbl_btn1, &lv_font_montserrat_14, 0);
    lv_obj_center(lbl_btn1);

    /* 设置按钮 */
    btn_setting = lv_btn_create(container);
    lv_obj_set_size(btn_setting, 90, 50);
    lv_obj_add_style(btn_setting, &style_btn, 0);
    lv_obj_add_event_cb(btn_setting, btn_event_handler, LV_EVENT_CLICKED, (void *)UI_VIEW_SETTING);
    lv_obj_t *lbl_btn2 = lv_label_create(btn_setting);
    lv_label_set_text(lbl_btn2, "⚙️ 设置");
    lv_obj_set_style_text_font(lbl_btn2, &lv_font_montserrat_14, 0);
    lv_obj_center(lbl_btn2);

    /* 报警按钮 */
    btn_alarm = lv_btn_create(container);
    lv_obj_set_size(btn_alarm, 90, 50);
    lv_obj_add_style(btn_alarm, &style_btn_alarm, 0);
    lv_obj_add_event_cb(btn_alarm, btn_event_handler, LV_EVENT_CLICKED, (void *)UI_VIEW_ALARM);
    lv_obj_t *lbl_btn3 = lv_label_create(btn_alarm);
    lv_label_set_text(lbl_btn3, "🚨 报警");
    lv_obj_set_style_text_font(lbl_btn3, &lv_font_montserrat_14, 0);
    lv_obj_center(lbl_btn3);
}

/* ==================== 按钮事件处理 ==================== */
static void btn_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    ui_view_t view = (ui_view_t)lv_event_get_user_data(e);

    if (code == LV_EVENT_CLICKED) {
        switch (view) {
            case UI_VIEW_REMIND:
                robot_ui_show_reminder("健康提醒", "该吃药了哦~");
                break;
            case UI_VIEW_SETTING:
                // TODO: 打开设置界面
                break;
            case UI_VIEW_ALARM:
                robot_ui_show_alarm("检测到异常！\n请确认是否需要帮助");
                break;
            default:
                break;
        }
    }
}

/* ==================== 创建报警屏幕 ==================== */
static void create_alarm_screen(void)
{
    /* 创建报警屏幕 */
    scr_alarm = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_alarm, lv_color_hex(0xF44336), 0);
    lv_obj_set_style_bg_opa(scr_alarm, LV_OPA_COVER, 0);

    /* 报警图标 */
    lv_obj_t *icon = lv_label_create(scr_alarm);
    lv_label_set_text(icon, "🚨");
    lv_obj_set_style_text_font(icon, &lv_font_montserrat_48, 0);
    lv_obj_align(icon, LV_ALIGN_CENTER, 0, -60);

    /* 报警文字 */
    lv_obj_t *text = lv_label_create(scr_alarm);
    lv_label_set_text(text, "紧急报警");
    lv_obj_set_style_text_color(text, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(text, &lv_font_montserrat_24, 0);
    lv_obj_align(text, LV_ALIGN_CENTER, 0, 0);

    /* 报警详情 */
    lv_obj_t *detail = lv_label_create(scr_alarm);
    lv_label_set_text(detail, "检测到异常情况\n家人已收到通知");
    lv_obj_set_style_text_color(detail, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(detail, &lv_font_montserrat_16, 0);
    lv_obj_align(detail, LV_ALIGN_CENTER, 0, 40);

    /* 返回按钮 */
    lv_obj_t *btn_back = lv_btn_create(scr_alarm);
    lv_obj_set_size(btn_back, 120, 50);
    lv_obj_align(btn_back, LV_ALIGN_CENTER, 0, 100);
    lv_obj_set_style_bg_color(btn_back, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_radius(btn_back, 25, 0);
    lv_obj_add_event_cb(btn_back, btn_event_handler, LV_EVENT_CLICKED, (void *)UI_VIEW_MAIN);

    lv_obj_t *lbl_back = lv_label_create(btn_back);
    lv_label_set_text(lbl_back, "返回");
    lv_obj_set_style_text_color(lbl_back, lv_color_hex(0xF44336), 0);
    lv_obj_center(lbl_back);

    /* 报警闪烁动画 */
    lv_anim_init(&anim_blink);
    lv_anim_set_var(&anim_blink, scr_alarm);
    lv_anim_set_values(&anim_blink, LV_OPA_COVER, LV_OPA_50);
    lv_anim_set_time(&anim_blink, 500);
    lv_anim_set_playback_time(&anim_blink, 500);
    lv_anim_set_repeat_count(&anim_blink, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_exec_cb(&anim_blink, (lv_anim_exec_xcb_t)lv_obj_set_style_bg_opa);
}

/* ==================== 初始化 UI ==================== */
void robot_ui_init(void)
{
    /* 初始化 LVGL（通常在系统启动时已完成） */
    // lv_init();
    // lv_disp_drv_register(...);
    // lv_indev_drv_register(...);

    /* 初始化样式 */
    init_styles();

    /* 创建屏幕 */
    create_main_screen();
    create_alarm_screen();

    /* 设置默认表情 */
    robot_ui_set表情(ROBOT表情_HAPPY);
}

/* ==================== 设置表情 ==================== */
void robot_ui_set表情(robot表情_t表情)
{
    if (表情 >= ROBOT表情_MAX) return;
    
    current表情 =表情;
    
    if (lbl表情) {
        lv_label_set_text(lbl表情,表情数组[表情]);
        
        /* 根据表情改变颜色 */
        switch (表情) {
            case ROBOT表情_HAPPY:
                lv_obj_set_style_text_color(lbl表情, lv_color_hex(0xFFEB3B), 0); // 黄色
                break;
            case ROBOT表情_THINKING:
                lv_obj_set_style_text_color(lbl表情, lv_color_hex(0x2196F3), 0); // 蓝色
                break;
            case ROBOT表情_SLEEPY:
                lv_obj_set_style_text_color(lbl表情, lv_color_hex(0x9E9E9E), 0); // 灰色
                break;
            case ROBOT表情_ALARM:
                lv_obj_set_style_text_color(lbl表情, lv_color_hex(0xF44336), 0); // 红色
                break;
            default:
                lv_obj_set_style_text_color(lbl表情, lv_color_hex(0xFFEB3B), 0);
                break;
        }
    }
}

/* ==================== 设置 AI 回复 ==================== */
void robot_ui_set_ai_reply(const char *text)
{
    if (lbl_ai_reply && text) {
        lv_label_set_text(lbl_ai_reply, text);
    }
}

/* ==================== 设置状态 ==================== */
void robot_ui_set_status(robot_status_t status)
{
    current_status = status;
    
    if (lbl_status) {
        switch (status) {
            case ROBOT_STATUS_IDLE:
                lv_label_set_text(lbl_status, "● 在线");
                lv_obj_set_style_text_color(lbl_status, lv_color_hex(0x4CAF50), 0);
                break;
            case ROBOT_STATUS_LISTENING:
                lv_label_set_text(lbl_status, "● 监听中...");
                lv_obj_set_style_text_color(lbl_status, lv_color_hex(0x2196F3), 0);
                break;
            case ROBOT_STATUS_SPEAKING:
                lv_label_set_text(lbl_status, "● 说话中...");
                lv_obj_set_style_text_color(lbl_status, lv_color_hex(0xFF9800), 0);
                break;
            case ROBOT_STATUS_REMINDING:
                lv_label_set_text(lbl_status, "● 提醒中...");
                lv_obj_set_style_text_color(lbl_status, lv_color_hex(0x9C27B0), 0);
                break;
            case ROBOT_STATUS_ALARM:
                lv_label_set_text(lbl_status, "● 报警！");
                lv_obj_set_style_text_color(lbl_status, lv_color_hex(0xF44336), 0);
                break;
            default:
                break;
        }
    }
}

/* ==================== 显示提醒 ==================== */
void robot_ui_show_reminder(const char *title, const char *content)
{
    /* 创建提醒弹窗 */
    lv_obj_t *mbox = lv_msgbox_create(NULL, title, content, NULL, true);
    lv_obj_center(mbox);
    lv_obj_set_style_bg_color(mbox, lv_color_hex(0x2D2D44), 0);
    lv_obj_set_style_text_color(mbox, lv_color_hex(0xFFFFFF), 0);
}

/* ==================== 显示报警 ==================== */
void robot_ui_show_alarm(const char *content)
{
    /* 切换到报警屏幕 */
    lv_scr_load(scr_alarm);
    
    /* 设置报警表情 */
    robot_ui_set表情(ROBOT表情_ALARM);
    robot_ui_set_status(ROBOT_STATUS_ALARM);
}

/* ==================== 关闭报警 ==================== */
void robot_ui_close_alarm(void)
{
    /* 返回主界面 */
    lv_scr_load(scr_main);
    
    /* 恢复正常状态 */
    robot_ui_set表情(ROBOT表情_HAPPY);
    robot_ui_set_status(ROBOT_STATUS_IDLE);
}

/* ==================== 切换界面 ==================== */
void robot_ui_switch_view(ui_view_t view)
{
    switch (view) {
        case UI_VIEW_MAIN:
            lv_scr_load(scr_main);
            break;
        case UI_VIEW_ALARM:
            lv_scr_load(scr_alarm);
            break;
        default:
            break;
    }
}
