/**
 * robot_ui.c - 智爱陪伴机器人界面实现
 * SF32LB52-DevKit-LCD LVGL 界面开发
 */

#include "robot_ui.h"
#include "touch_ui.h"
#include "network_comm.h"     /* report_alarm()：报警要上报 MQTT + 手机推送 */
#include <stdio.h>
#include <string.h>
#include <time.h>             /* time() / localtime_r()：状态栏时钟用 */
#include <stdbool.h>
#include <unistd.h>           /* usleep()：等 hello_app 交出麦克风时的轮询/续租间隔 */
#include <pthread.h>          /* 报警出声线程（报警声不能做在 LVGL 线程里，见下面那节） */

/* 让 hello_app 交出麦克风（ai_companion_audio_yield(true)）/ 收回
 * （ai_companion_mic_reclaim()），用法和理由与 app/robot_ui/main.c 里提醒那条路
 * 完全一样（那边是 reminder_play_exclusive / reminder_wait_mic_released）：
 * 两个入口都**非阻塞**，只登记请求，真正的停/开设备由 hello_app 自己的线程做，
 * 这里靠轮询 ai_companion_mic_released() 知道让没让成。
 * 头文件是自给自足的（只依赖 stdbool/stdint），实现在 app/hello_app 里，
 * 符号在最终链接时解析（单一大镜像）；头文件路径由 CMakeLists.txt 的
 * INCLUDE_DIRECTORIES `../hello_app` 提供，无需改构建脚本。 */
#include "ai_companion_yield.h"

/* 状态栏时钟：定义在本文件后面（ui_clock_refresh / ui_clock_timer_cb），
 * 状态栏创建时立刻刷新一次，并挂一个定时器周期刷新。 */
static void ui_clock_refresh(void);
static void ui_clock_timer_cb(lv_timer_t *t);
static lv_timer_t *clock_timer = NULL;

/* 板级报警模块（board/contest_board/src/sf32lb52_alarm.h）。
 * 头文件路径由 CMakeLists.txt 的 INCLUDE_DIRECTORIES ${NUTTX_BOARD_ABS_DIR}/src 提供。 */
#include "sf32lb52_alarm.h"

/* 中文字库（实现在 lv_font_ui_16/20/24.c，见 CMakeLists.txt 的 SRCS）。
 * 字符集是常用汉字全集（GB2312 6763 字 + ASCII + CJK 标点 + 全角），
 * 按改动前 montserrat 的字号分三档：
 *   14/16/18 -> lv_font_ui_16   20/22/24 -> lv_font_ui_20   >=28 -> lv_font_ui_24 */
LV_FONT_DECLARE(lv_font_ui_16);
LV_FONT_DECLARE(lv_font_ui_20);
LV_FONT_DECLARE(lv_font_ui_24);

/* ==================== 全局变量 ==================== */
static lv_obj_t *scr_main = NULL;      // 主屏幕
static lv_obj_t *scr_alarm = NULL;     // 报警屏幕

/* 主界面组件 */
/* lbl_status / lbl_net：**状态栏里已经不再创建这两个标签**（2026-09-14 用户要求
 * 删掉"[在线]"和"NET --"，见 create_status_bar() 的注释）。这里保留声明是因为
 * robot_ui_set_status() / robot_ui_set_net_status() 还在（内部判 NULL 后直接返回），
 * 这样 main.c 里那十几处调用点不用改。它们恒为 NULL，不会有任何显示。 */
static lv_obj_t *lbl_status = NULL;    // 状态标签（已不再创建）
static lv_obj_t *lbl_time = NULL;      // 时间标签
static lv_obj_t *lbl_net = NULL;       // 网络状态标签（已不再创建）
static lv_obj_t *lbl_face = NULL;      // 表情标签
static lv_obj_t *lbl_ai_reply = NULL;  // AI回复标签
static lv_obj_t *lbl_reminder = NULL;  // 提醒标签

/* 按钮 */
static lv_obj_t *btn_remind = NULL;    // 提醒按钮
static lv_obj_t *btn_setting = NULL;   // 设置按钮
static lv_obj_t *btn_alarm = NULL;     // 报警按钮

/* 报警页的 "!!!" 图标：闪烁只动它一个（原来动的是整屏，见 alarm_blink_timer_cb） */
static lv_obj_t *lbl_alarm_icon = NULL;

/* 动画 */
static lv_anim_t anim_face = {0};      // 表情动画

/* 报警闪烁：低频定时器，创建后先暂停，报警时 resume（见 robot_ui_show_alarm） */
static lv_timer_t *alarm_blink_timer = NULL;

/* 当前状态 */
static robot_face_t current_face = ROBOT_FACE_HAPPY;
static robot_status_t current_status = ROBOT_STATUS_IDLE;

/* 样式 */
static lv_style_t style_bg;           // 背景样式
static lv_style_t style_btn;          // 按钮样式
static lv_style_t style_btn_alarm;    // 报警按钮样式
static lv_style_t style_text;         // 文字样式
static lv_style_t style_face;         // 表情样式

/* ==================== 表情数据 ==================== */
/* ASCII 表情 */
static const char *face_array[] = {
    "(^_^)",     // 开心
    "(=_=)",     // 思考
    "(~_~)zZZ", // 困倦
    "(O_O)",     // 惊讶
    "(>_<)",     // 担心
    "(! ! !)",   // 报警
};

/* ==================== 内部函数声明 ==================== */
static void init_styles(void);
static void create_main_screen(void);
static void create_alarm_screen(void);
static void create_status_bar(lv_obj_t *parent);
static void create_face_area(lv_obj_t *parent);
static void create_ai_reply_area(lv_obj_t *parent);
static void create_bottom_buttons(lv_obj_t *parent);
static void btn_event_handler(lv_event_t *e);
static void alarm_blink_timer_cb(lv_timer_t *t);

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
    lv_style_set_text_font(&style_text, &lv_font_ui_16);

    /* 表情样式 - 大号字体 */
    lv_style_init(&style_face);
    lv_style_set_text_color(&style_face, lv_color_hex(0xFFEB3B));
    lv_style_set_text_font(&style_face, &lv_font_ui_24);
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
    create_face_area(scr_main);

    /* 创建 AI 回复区域 */
    create_ai_reply_area(scr_main);

    /* 创建底部按钮 */
    create_bottom_buttons(scr_main);

    /* 加载主屏幕 */
    lv_scr_load(scr_main);
}

/* ==================== 状态栏时钟 ==================== */

/* 时区偏移：本构建没开 CONFIG_LIBC_LOCALTIME，`localtime` 实际就是 `gmtime`，
 * 也就是系统时钟（RTC）给的是 **UTC**。这里手动加偏移显示北京时间（UTC+8）。
 * 换时区只改这一个值即可（负值表示西半球）。
 *
 * 注：更"正规"的做法是开 CONFIG_LIBC_LOCALTIME 并设 TZ，但那会换掉
 * mktime/localtime 的实现，而 `RTC_SET_RELATIVE`（sf32lb_rtc.c 的 add_timeout）
 * 正好依赖现在这套简化日历换算，**不建议现在动**；显示层加偏移最省事也最可控。 */
#define UI_TZ_OFFSET_SEC  (8 * 3600)

/* 从系统时钟（硬件 RTC）读一次，刷新状态栏上的时间标签（本地时间）。 */
static void ui_clock_refresh(void)
{
    time_t now = time(NULL);
    struct tm tm_buf;

    if (lbl_time == NULL) {
        return;
    }

    /* time() <= 0：说明时钟还没被设过（板上没有 RTC 备份电池，
     * 冷启动会读到 1900/2000 这类值）—— 用 "--:--" 明确表示"时间不可信"，
     * 比继续显示一个假的时间好。 */
    if (now <= 0) {
        lv_label_set_text(lbl_time, "--:--");
        return;
    }

    /* UTC -> 本地时间（跨天由加法自然进位） */
    if (UI_TZ_OFFSET_SEC != 0) {
        now += UI_TZ_OFFSET_SEC;
    }

    if (localtime_r(&now, &tm_buf) == NULL) {
        lv_label_set_text(lbl_time, "--:--");
        return;
    }

    lv_label_set_text_fmt(lbl_time, "%02d:%02d",
                          tm_buf.tm_hour, tm_buf.tm_min);
}

static void ui_clock_timer_cb(lv_timer_t *t)
{
    (void)t;
    ui_clock_refresh();
}

/* ==================== 创建状态栏 ==================== */
/* 状态栏上的「菜单」按钮：主菜单的保底入口。
 * 主菜单只在开机时显示一次，关掉之后要么右滑（手势，见 touch_ui.c），
 * 要么点这里。两者都调 touch_ui_show_menu(MENU_TYPE_MAIN)。 */
static void menu_button_event_handler(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        touch_ui_show_menu(MENU_TYPE_MAIN);
    }
}

static void create_status_bar(lv_obj_t *parent)
{
    /* 状态栏容器。**现在只放两样：时钟 + 「菜单」按钮。**
     *
     * 2026-09-14 用户要求删掉原来那三样：「[在线]」状态、「WiFi 100%」、「NET --」——
     * 对老人没有任何用处（电量/信号本来就是写死的假值，网络状态只会让人困惑），
     * 挤在 40px 高的栏里还占地方。删掉之后：
     *   - 时间保留（它是真的，联网自动对时之后是准的）；
     *   - 「菜单」按钮**做大**（130×40，占满整栏高度）—— 它是手势失灵时进主菜单
     *     唯一的保底入口，老人手指粗，越大越好按。
     *
     * 注意：`robot_ui_set_status()` / `robot_ui_set_net_status()` 两个 setter
     * **保留**了（对应标签不再创建、恒为 NULL，它们内部都有 NULL 检查）——
     * 这样 main.c 里那十几处调用点一行都不用改，只是不再有任何显示效果。 */
    lv_obj_t *bar = lv_obj_create(parent);
    /* 高度 40 → **64**：菜单按钮要做到 52 高才够好点（40 的栏里塞不下），
     * 顺便让整块状态栏不那么挤。 */
    lv_obj_set_size(bar, LV_PCT(100), 64);
    lv_obj_align(bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0x2D2D44), 0);
    lv_obj_set_style_border_width(bar, 0, 0);
    lv_obj_set_style_radius(bar, 0, 0);
    lv_obj_set_style_pad_all(bar, 0, 0);
    lv_obj_remove_flag(bar, LV_OBJ_FLAG_SCROLLABLE);
    /* ⚠️ 这块屏是 **390×450 的圆角屏**：四个角是物理圆角，贴边的控件既会被
     * 视觉裁掉、也不好点按（用户反馈"菜单栏不好点"就是这个原因）。
     * 所以这里**不用 flex 排布**，改成两个子对象各自对齐：
     *   时间  -> 居中（用户要求）
     *   按钮  -> 右对齐 + 内缩 28px（躲开右上圆角）
     * 圆角半径大约 20~30px，取 28 作为安全内缩。 */

    /* 时间标签（居中）：从系统时钟（硬件 RTC）读真实时间，并定时刷新。
     *
     * 原来这里是写死的 lv_label_set_text(lbl_time, "12:00")，所以主页面顶上
     * **永远显示 12:00**，跟板子时间毫无关系。
     *
     * 数据源用 time(NULL)：NuttX 启动时用 RTC 初始化系统时钟，
     * 而 RTC_SET_TIME（含 NSH 的 `date -s`）会同步系统时钟，所以它就是板子的时间。
     * 板上没有 RTC 备份电池，掉电后时间会丢，开机后靠联网自动对时校正
     * （见 time_sync.c）；也可以手动：date -s "Sep 13 12:00:00 2026"
     *
     * 注意：本构建没开 CONFIG_LIBC_LOCALTIME，`localtime` 实际就是 `gmtime`，
     * 所以**显示的是 UTC**，比北京时间少 8 小时。要显示本地时间得自己加偏移
     * （这里刻意不加，保持"屏幕上显示的就是 RTC 里的值"，免得排查时对不上）。 */
    lbl_time = lv_label_create(bar);
    lv_obj_set_style_text_color(lbl_time, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(lbl_time, &lv_font_ui_24, 0);
    lv_obj_align(lbl_time, LV_ALIGN_CENTER, 0, 0);           /* 时间居中 */
    ui_clock_refresh();                                      /* 先立刻显示一次 */
    clock_timer = lv_timer_create(ui_clock_timer_cb, 10000, NULL);  /* 每 10 秒刷一次 */

    /* 「菜单」按钮：主菜单的保底入口（手势不灵时也能进）。
     * 120×52 = 比原来的 LV_SIZE_CONTENT×36 大得多（宽度几乎翻倍、高度 +44%），
     * 而且**从右上圆角里挪出来**（右移 -28px）—— 之前贴在屏幕角上，
     * 圆角把可点区域切掉一块，所以"不好点"。 */
    lv_obj_t *btn_menu = lv_btn_create(bar);
    lv_obj_set_size(btn_menu, 120, 52);
    lv_obj_align(btn_menu, LV_ALIGN_RIGHT_MID, -28, 0);
    lv_obj_add_style(btn_menu, &style_btn, 0);
    lv_obj_add_event_cb(btn_menu, menu_button_event_handler,
                        LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl_menu = lv_label_create(btn_menu);
    lv_label_set_text(lbl_menu, "菜单");
    lv_obj_set_style_text_font(lbl_menu, &lv_font_ui_24, 0);
    lv_obj_center(lbl_menu);
}

/* ==================== 更新网络状态 ==================== */
void robot_ui_set_net_status(const char *text)
{
    if (lbl_net == NULL || text == NULL) {
        return;
    }

    if (strcmp(lv_label_get_text(lbl_net), text) == 0) {
        return;     /* 没变就不动，省一次重绘 */
    }

    lv_label_set_text(lbl_net, text);
    lv_obj_set_style_text_color(lbl_net,
                                strncmp(text, "NET OK", 6) == 0
                                    ? lv_color_hex(0x4CAF50)
                                    : lv_color_hex(0xFFC107),
                                0);
}

/* ==================== 创建表情区域 ==================== */
static void create_face_area(lv_obj_t *parent)
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
    lbl_face = lv_label_create(container);
    lv_label_set_text(lbl_face, face_array[ROBOT_FACE_HAPPY]);
    lv_obj_add_style(lbl_face, &style_face, 0);
    lv_obj_set_style_text_font(lbl_face, &lv_font_ui_24, 0);

    /* 状态文字 */
    lbl_reminder = lv_label_create(container);
    lv_label_set_text(lbl_reminder, "Hello! I am ZhiAi.");
    lv_obj_set_style_text_color(lbl_reminder, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(lbl_reminder, &lv_font_ui_24, 0);
    lv_obj_set_style_pad_top(lbl_reminder, 10, 0);

    /* 启动表情动画 - 上下浮动 */
    lv_anim_init(&anim_face);
    lv_anim_set_var(&anim_face, lbl_face);
    lv_anim_set_values(&anim_face, -10, 10);
    lv_anim_set_time(&anim_face, 1000);
    lv_anim_set_playback_time(&anim_face, 1000);
    lv_anim_set_repeat_count(&anim_face, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_exec_cb(&anim_face, (lv_anim_exec_xcb_t)lv_obj_set_y);
    lv_anim_start(&anim_face);
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
    lv_label_set_text(lbl_icon, "[AI]");
    lv_obj_set_style_text_color(lbl_icon, lv_color_hex(0x4CAF50), 0);
    lv_obj_set_style_text_font(lbl_icon, &lv_font_ui_24, 0);

    /* AI 回复内容 */
    lbl_ai_reply = lv_label_create(container);
    lv_label_set_text(lbl_ai_reply, "Hello!\nHow can I help you?");
    lv_obj_set_style_text_color(lbl_ai_reply, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(lbl_ai_reply, &lv_font_ui_24, 0);
    lv_label_set_long_mode(lbl_ai_reply, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_ai_reply, LV_PCT(100));
    lv_obj_set_style_text_align(lbl_ai_reply, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_pad_top(lbl_ai_reply, 5, 0);
}

/* ==================== 创建底部按钮 ==================== */
static void create_bottom_buttons(lv_obj_t *parent)
{
    /* 按钮容器 */
    /* ⚠️ 这块屏是圆角屏（390×450）：容器铺满 100% 宽的话，最左/最右那个按钮
     * 会压在**左下/右下圆角**上 —— 视觉被切一块，触摸也可能点不到。
     * 所以左右各内缩 28px，三个按钮整体往里收（用户反馈"不好点"）。 */
    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_set_size(container, LV_PCT(100), 80);
    lv_obj_align(container, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(container, 0, 0);
    lv_obj_set_style_pad_left(container, 28, 0);
    lv_obj_set_style_pad_right(container, 28, 0);
    lv_obj_set_style_pad_bottom(container, 6, 0);
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(container, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    /* 提醒按钮 */
    btn_remind = lv_btn_create(container);
    lv_obj_set_size(btn_remind, 90, 50);
    lv_obj_add_style(btn_remind, &style_btn, 0);
    lv_obj_add_event_cb(btn_remind, btn_event_handler, LV_EVENT_CLICKED, (void *)UI_VIEW_REMIND);
    lv_obj_t *lbl_btn1 = lv_label_create(btn_remind);
    /* 中文标签（字库已经是 GB2312 全集，"提醒"两个字有字形）——
     * 原来写的 "Remind"，主屏其它按钮（报警）都是中文，统一一下更认得出。 */
    lv_label_set_text(lbl_btn1, "提醒");
    lv_obj_set_style_text_font(lbl_btn1, &lv_font_ui_24, 0);
    lv_obj_center(lbl_btn1);

    /* 设置按钮 */
    btn_setting = lv_btn_create(container);
    lv_obj_set_size(btn_setting, 90, 50);
    lv_obj_add_style(btn_setting, &style_btn, 0);
    lv_obj_add_event_cb(btn_setting, btn_event_handler, LV_EVENT_CLICKED, (void *)UI_VIEW_SETTING);
    lv_obj_t *lbl_btn2 = lv_label_create(btn_setting);
    lv_label_set_text(lbl_btn2, "Setting");
    lv_obj_set_style_text_font(lbl_btn2, &lv_font_ui_24, 0);
    lv_obj_center(lbl_btn2);

    /* 报警按钮 */
    btn_alarm = lv_btn_create(container);
    lv_obj_set_size(btn_alarm, 90, 50);
    lv_obj_add_style(btn_alarm, &style_btn_alarm, 0);
    lv_obj_add_event_cb(btn_alarm, btn_event_handler, LV_EVENT_CLICKED, (void *)UI_VIEW_ALARM);
    lv_obj_t *lbl_btn3 = lv_label_create(btn_alarm);
    lv_label_set_text(lbl_btn3, "报警");
    lv_obj_set_style_text_font(lbl_btn3, &lv_font_ui_24, 0);
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
                /* 主屏左下角这个按钮以前只弹一句写死的英文示例（"Time to take
                 * medicine!"），和真正的提醒列表毫无关系。现在直接走**主菜单
                 * 「查看提醒」同一个入口** —— 多一个冗余入口，老人不用先点开主菜单。 */
                touch_ui_play_sound("click");
                touch_ui_show_menu(MENU_TYPE_REMIND);
                break;
            case UI_VIEW_SETTING:
                touch_ui_show_menu(MENU_TYPE_SETTING);
                break;
            case UI_VIEW_ALARM:
                robot_ui_show_alarm("Abnormal detected!\nPlease confirm if help is needed.");
                break;
            case UI_VIEW_MAIN:
                /* 报警界面 Back 按钮:关闭报警，返回主界面 */
                robot_ui_close_alarm();
                break;
            default:
                break;
        }
    }
}

/* ==================== 创建报警屏幕 ==================== */

/* 报警闪烁回调：在"满不透明"和"半透明"之间切换 "!!!" 图标的整体不透明度。
 *
 * 原来这里是绑在 **整个 scr_alarm** 上的无限 lv_anim（改 bg_opa）：每帧都让
 * 整屏（390x450）失效，实测一秒 80 次全屏重绘（正常空闲 20~30），用户看到的
 * 就是报警页一直在闪、而且整屏半透明时画面发暗。现在失效面积只剩一个小图标，
 * 频率也从 80Hz 降到 2Hz（500ms 一次）。 */
static void alarm_blink_timer_cb(lv_timer_t *t)
{
    (void)t;

    if (lbl_alarm_icon == NULL)
    {
        return;
    }

    lv_opa_t opa = lv_obj_get_style_opa(lbl_alarm_icon, 0);
    lv_obj_set_style_opa(lbl_alarm_icon, (opa == LV_OPA_COVER) ? LV_OPA_50 : LV_OPA_COVER, 0);
}

static void create_alarm_screen(void)
{
    /* 创建报警屏幕 */
    scr_alarm = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_alarm, lv_color_hex(0xF44336), 0);
    lv_obj_set_style_bg_opa(scr_alarm, LV_OPA_COVER, 0);

    /* 报警图标（闪烁只动它，不透明度的切换见 alarm_blink_timer_cb） */
    lbl_alarm_icon = lv_label_create(scr_alarm);
    lv_label_set_text(lbl_alarm_icon, "!!!");
    lv_obj_set_style_text_font(lbl_alarm_icon, &lv_font_ui_24, 0);
    lv_obj_set_style_text_color(lbl_alarm_icon, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(lbl_alarm_icon, LV_ALIGN_CENTER, 0, -60);

    /* 报警文字 */
    lv_obj_t *text = lv_label_create(scr_alarm);
    lv_label_set_text(text, "紧急");
    lv_obj_set_style_text_color(text, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(text, &lv_font_ui_24, 0);
    lv_obj_align(text, LV_ALIGN_CENTER, 0, 0);

    /* 报警详情 */
    lv_obj_t *detail = lv_label_create(scr_alarm);
    lv_label_set_text(detail, "检测到异常\n已通知家人");
    lv_obj_set_style_text_color(detail, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(detail, &lv_font_ui_24, 0);
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
    lv_obj_set_style_text_font(lbl_back, &lv_font_ui_24, 0);
    lv_obj_center(lbl_back);

    /* 报警闪烁：2Hz 低频定时器，创建后先暂停（报警页还没显示，不必闪）。
     * 报警时由 robot_ui_show_alarm() resume。 */
    alarm_blink_timer = lv_timer_create(alarm_blink_timer_cb, 500, NULL);
    lv_timer_pause(alarm_blink_timer);
}

/* ==================== 初始化 UI ==================== */
void robot_ui_init(void)
{
    /* 初始化样式 */
    init_styles();

    /* 创建屏幕 */
    create_main_screen();
    create_alarm_screen();

    /* 设置默认表情 */
    robot_ui_set_face(ROBOT_FACE_HAPPY);
}

/* ==================== 设置表情 ==================== */
void robot_ui_set_face(robot_face_t face)
{
    if (face >= ROBOT_FACE_MAX) return;

    current_face = face;

    if (lbl_face) {
        lv_label_set_text(lbl_face, face_array[face]);

        /* 根据表情改变颜色 */
        switch (face) {
            case ROBOT_FACE_HAPPY:
                lv_obj_set_style_text_color(lbl_face, lv_color_hex(0xFFEB3B), 0); // 黄色
                break;
            case ROBOT_FACE_THINKING:
                lv_obj_set_style_text_color(lbl_face, lv_color_hex(0x2196F3), 0); // 蓝色
                break;
            case ROBOT_FACE_SLEEPY:
                lv_obj_set_style_text_color(lbl_face, lv_color_hex(0x9E9E9E), 0); // 灰色
                break;
            case ROBOT_FACE_ALARM:
                lv_obj_set_style_text_color(lbl_face, lv_color_hex(0xF44336), 0); // 红色
                break;
            default:
                lv_obj_set_style_text_color(lbl_face, lv_color_hex(0xFFEB3B), 0);
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
                lv_label_set_text(lbl_status, "[在线]");
                lv_obj_set_style_text_color(lbl_status, lv_color_hex(0x4CAF50), 0);
                break;
            case ROBOT_STATUS_LISTENING:
                lv_label_set_text(lbl_status, "[聆听中]");
                lv_obj_set_style_text_color(lbl_status, lv_color_hex(0x2196F3), 0);
                break;
            case ROBOT_STATUS_SPEAKING:
                lv_label_set_text(lbl_status, "[回复中]");
                lv_obj_set_style_text_color(lbl_status, lv_color_hex(0xFF9800), 0);
                break;
            case ROBOT_STATUS_REMINDING:
                lv_label_set_text(lbl_status, "[提醒中]");
                lv_obj_set_style_text_color(lbl_status, lv_color_hex(0x9C27B0), 0);
                break;
            case ROBOT_STATUS_ALARM:
                lv_label_set_text(lbl_status, "[报警!]");
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
    lv_obj_t *mbox = lv_msgbox_create(NULL);
    if (mbox == NULL) return;

    lv_msgbox_add_title(mbox, title);
    lv_msgbox_add_text(mbox, content);
    touch_ui_msgbox_add_close_x(mbox);
    lv_obj_center(mbox);
    lv_obj_set_style_bg_color(mbox, lv_color_hex(0x2D2D44), 0);
    lv_obj_set_style_text_color(mbox, lv_color_hex(0xFFFFFF), 0);

    /* 字体必须显式指定：msgbox 默认吃 LVGL 主题字体，而本工程的默认字体是
     * 16px 的 simsun（约 1436 个字形），界面用到的一些汉字（"散""嘱"…）
     * 它会画成方块。这里统一成 build 里真正的三档字库之一，和 touch_ui 的
     * 弹窗一致（20px：比 24px 少占地方，长一点的提醒语不容易顶出屏幕）。 */
    lv_obj_set_style_text_font(mbox, &lv_font_ui_20, 0);
}

/* ==================== 报错提示页 ==================== */
/*
 * 为什么单独做一页：以前网络不通/请求失败时，界面只是"没反应"或者停在
 * "处理中…"，用户根本不知道出了什么事（现场原话："卡死不好看"）。
 * 这一页是**顶层覆盖**（建在当前活动屏上，不切屏、不动报警页），
 * 一个大按钮，点一下就关。
 *
 * ⚠️ 只能在 LVGL 线程里调；别的线程（语音工作线程 / MQTT 线程 / 播放线程）
 * 要弹它请用 main.c 的 ui_post_error()，那条路会 lv_async_call 投过来。
 */

static lv_obj_t *err_panel = NULL;

static void err_close_event_handler(lv_event_t *e)
{
    (void)e;

    if (err_panel != NULL) {
        lv_obj_del(err_panel);
        err_panel = NULL;
    }

    touch_ui_play_sound("back");
}

void robot_ui_show_error(const char *title, const char *content)
{
    lv_obj_t *scr = lv_scr_act();

    if (scr == NULL) {
        return;
    }

    /* 同一时刻只留一页：又报错了就换内容，不叠罗汉 */

    if (err_panel != NULL) {
        lv_obj_del(err_panel);
        err_panel = NULL;
    }

    err_panel = lv_obj_create(scr);
    lv_obj_set_size(err_panel, LV_PCT(92), LV_PCT(78));
    lv_obj_center(err_panel);
    lv_obj_set_style_bg_color(err_panel, lv_color_hex(0x3A1F1F), 0);
    lv_obj_set_style_bg_opa(err_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(err_panel, 3, 0);
    lv_obj_set_style_border_color(err_panel, lv_color_hex(0xE53935), 0);
    lv_obj_set_style_radius(err_panel, 20, 0);
    lv_obj_set_flex_flow(err_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(err_panel, LV_FLEX_ALIGN_SPACE_EVENLY,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_remove_flag(err_panel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *t = lv_label_create(err_panel);
    lv_label_set_text(t, (title != NULL && title[0] != '\0') ? title : "出错了");
    lv_obj_set_style_text_font(t, &lv_font_ui_24, 0);
    lv_obj_set_style_text_color(t, lv_color_hex(0xFFCDD2), 0);

    lv_obj_t *c = lv_label_create(err_panel);
    lv_label_set_text(c, (content != NULL) ? content : "");
    lv_obj_set_style_text_font(c, &lv_font_ui_20, 0);
    lv_obj_set_style_text_color(c, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_width(c, LV_PCT(95));
    lv_label_set_long_mode(c, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(c, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *btn = lv_btn_create(err_panel);
    lv_obj_set_size(btn, LV_PCT(70), 70);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0xE53935), 0);
    lv_obj_set_style_radius(btn, 16, 0);
    lv_obj_add_event_cb(btn, err_close_event_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t *bl = lv_label_create(btn);
    lv_label_set_text(bl, "知道了");
    lv_obj_set_style_text_font(bl, &lv_font_ui_24, 0);
    lv_obj_center(bl);

    printf("[UI] 报错页: %s | %s\n",
           (title != NULL) ? title : "", (content != NULL) ? content : "");
}

/* ==================== 报警出声：先请 hello_app 让路 ====================
 *
 * 根因（2026-09-15 定案，含现场串口实证）：**报警这条路一次让路都没做**。
 *   - hello_app 的 ai_companion 是常开麦：开机起就占着 /dev/audio/audio0 的录音
 *     通路；本板音频是**半双工**、驱动里只有一个方向标志。
 *   - 板级报警模块 alarm_audio_open() 走 open -> CONFIGURE -> CONFIGURE(音量)
 *     -> START（board/contest_board/src/sf32lb52_alarm.c）。录音会话还是 RUNNING
 *     时，NuttX 音频上层（nuttx/audio/audio.c）那两处判断会**静默跳过** ——
 *     只有 state==OPEN 才把 CONFIGURE 转给驱动、只有 PREPARED/XRUN 才调下层
 *     start —— 最后照样 `return OK`：报警模块以为成功了，一行日志都没有。
 *   - 于是报警的 PCM 全写在录音会话上，DAC/功放从未打开、DMA 永不完成 →
 *     驱动每 5 秒打一条 `AUDIO: 等播放完成超时（buflen=1600），本块丢弃`，
 *     报警音一块都出不去，**全程无声**（现场日志实证）。
 *   - 对照：robot_ui 自己的提醒/播报**有完整让路**（app/robot_ui/main.c 的
 *     reminder_play_exclusive → ai_companion_audio_yield + 轮询 + reclaim），
 *     所以它们正常出声 —— 只有报警这条漏了。
 *
 * 做法：照机器人这边现成的提醒那条路，**同一套调用、同一组超时档位**：
 *     ai_companion_audio_yield(true)                      （登记让路，非阻塞）
 *     -> 有界轮询 ai_companion_mic_released()             （首轮 1.5s + 补等两次）
 *     -> alarm_trigger() + report_alarm_queued()          （出声 + 上报）
 *     -> ai_companion_mic_reclaim()                       （close_alarm 时还回去）
 * 三个入口都只登记请求/读状态，一个设备都不碰；真正的停/开设备在 hello_app
 * 自己的线程里（协议细节和三次踩坑史见 app/hello_app/ai_companion_yield.h）。
 *
 * ⚠️ 为什么"等让路 + 出声"要放进一条**工作线程**，而不是就地做在
 * robot_ui_show_alarm() 里：
 *   show_alarm() 跑在 **LVGL 线程**（报警按钮的回调 / main.c 里 ui_post_panel 的
 *   投递）。那套让路协议**禁止在 LVGL 线程里轮询**（最长 3.3 秒，在 LVGL 线程里等
 *   就是把界面冻住 —— 这个项目已经因为同类问题冻过一次，见 ai_companion_yield.h
 *   的第二条约束）。所以这里拆成两半：
 *     ① LVGL 线程（robot_ui_show_alarm）：切页面 -> 登记让路请求 + 起出声线程；
 *     ② 工作线程（alarm_sound_worker）：有界等麦克风 -> 出声 + 上报 -> 报警期间
 *        续租，直到报警页关掉才收摊。
 *   出声/上报因此比原来晚"让路那一下"（通常 100~400ms，最坏 3.3 秒）：
 *   页面仍然是第一步切出来的（"先切页面"那条踩坑教训不动），只有声音在等麦克风。
 *   这一步不能省：报警声若抢在让路之前出声，第一轮就撞上上面那个"静默跳过"，
 *   而那一轮里每块 1600 字节都要等驱动 5 秒超时（20 块 ≈ 100 秒），连 60 秒的
 *   自动解除都过去了 —— 现场看到的正是"全程无声"。
 *
 * ⚠️ 为什么报警期间要**续租**（每 1 秒把 yield(true) 再登记一次）：
 *   hello_app 对让路有一道 10 秒的收回看门狗（MIC_HOLD_WATCHDOG_MS）：让出去之后
 *   10 秒没收到新的登记，它就认定调用方漏了回收，把麦克风**强制收回去**并重开常开麦
 *   （见 ai_companion_yield.h 的"可选的续租"那段）。而报警页是模态的，用户点了
 *   "返回"才结束，中间可能十几秒到几十秒（报警本身最长 60 秒才自动解除）：
 *   不续租的话 10 秒一到 hello_app 重开麦，报警后面几轮 open 又撞上录音 → 又没声。
 *   续租只是把同一个方向再登记一次（接口是**电平语义、幂等**，重复登记不会叠出
 *   多次让路）；本模块自己的让路状态仍然是**只申请一次、只释放一次**
 *   （alarm_yield_held 那两个布尔量），不看调了几次。
 */

/* 有界等待的档位：和提醒那条路（app/robot_ui/main.c 的 REMINDER_YIELD_*）**同一组
 * 取值、同一个理由** —— 都是"登记请求 + 对方下一拍动手"：首轮 1.5 秒等不到多半只是
 * 对方那一拍正好在忙别的事，补等两次给它机会（间隔 400ms 是给 hello_app 一拍 100ms
 * 留四拍余量）。上限 1.5 + 2×(0.4+0.5) = 3.3 秒，到点就照常出声：报警是安全功能，
 * **等不到也必须响**（宁可撞一下，也不能因为没有麦克风就不出声）。 */
#define ALARM_YIELD_WAIT_MS        1500   /* = REMINDER_YIELD_WAIT_MS */
#define ALARM_YIELD_RETRY_TIMES       2   /* = REMINDER_YIELD_RETRY_TIMES */
#define ALARM_YIELD_RETRY_GAP_MS    400   /* = REMINDER_YIELD_RETRY_GAP_MS */
#define ALARM_YIELD_RETRY_WAIT_MS   500   /* = REMINDER_YIELD_RETRY_WAIT_MS */
#define ALARM_YIELD_POLL_MS          50   /* 轮询粒度 = REMINDER_YIELD_POLL_MS */

/* 续租周期：看门狗 10 秒，1 秒一次留足余量。每次续租只是一次变量写 +（方向没变时）
 * 一次信号量唤醒，代价可以忽略；报警最长 60 秒，这条线程自己也是睡着的。 */
#define ALARM_YIELD_RENEW_MS       1000

/* 出声线程的栈：它在等让路那一小段里只做轮询 / printf 和两个非阻塞调用，
 * 但 printf 和让路查询本身都要吃一些栈，照 main.c 里等价的那条线程
 * （voice_open_thread，同样"等让路 + 开麦"）给 16 KB，创建写法也一致
 * （pthread_attr + DETACHED）。 */
#define ALARM_WORKER_STACK_SIZE    16384

/* 报警让路的共享状态。写者有两条线程：LVGL 线程（show_alarm / close_alarm）和
 * 报警出声线程，所以用一把锁护住，判据只看这三个布尔量。
 *   alarm_yield_held    —— 这次报警借了麦克风、还没还（申请一次、释放一次都以它为准）
 *   alarm_sound_pending —— 有一次"出声 + 上报"还没做（show_alarm 每次调用置位）
 *   alarm_worker_up     —— 出声线程活着（活着就不再起第二条） */
static pthread_mutex_t alarm_audio_lock = PTHREAD_MUTEX_INITIALIZER;
static bool alarm_yield_held    = false;
static bool alarm_sound_pending = false;
static bool alarm_worker_up     = false;

/**
 * @brief  轮询等 hello_app 交出麦克风（首轮 + 补等），上限 3.3 秒
 *         —— app/robot_ui/main.c 的 reminder_wait_mic_released() 的同款写法
 *
 * @return true  = 麦克风已经不在 hello_app 手里，可以出声；
 *         false = 补等全用完还没让开，调用方**照常出声**（报警不能因为等不到就不响）。
 *
 * ⚠️ 只能在**工作线程**里调（最长会让当前线程等 3.3 秒，见本节头上那条约束）。
 */
static bool alarm_wait_mic_released(void)
{
    long waited_ms = 0;
    int  attempt;

    /* attempt 0 = 首轮（上限 ALARM_YIELD_WAIT_MS），之后是补等 */

    for (attempt = 0; attempt <= ALARM_YIELD_RETRY_TIMES; attempt++) {
        long limit = (attempt == 0) ? (long)ALARM_YIELD_WAIT_MS
                                    : (long)ALARM_YIELD_RETRY_WAIT_MS;
        long waited = 0;
        bool released;

        if (attempt > 0) {
            usleep(ALARM_YIELD_RETRY_GAP_MS * 1000);
            waited_ms += ALARM_YIELD_RETRY_GAP_MS;
        }

        /* waited == 0 也是正常情况：hello_app 压根没占着麦克风（没在跑 /
         * 本来就没开麦），接口直接报"不在它手里"，一秒都不用等。 */
        released = ai_companion_mic_released();

        while (!released && waited < limit) {
            usleep(ALARM_YIELD_POLL_MS * 1000);   /* 50 ms 一探：够细，也不占 CPU */
            waited += ALARM_YIELD_POLL_MS;
            released = ai_companion_mic_released();
        }

        waited_ms += waited;

        if (released) {
            printf("[Alarm] 麦克风不在 hello_app 手里（等了 %ld ms），可以出声\n",
                   waited_ms);
            return true;
        }
    }

    /* "没让成"的明细只在这里打一次（补等各打一行的话，真出问题时串口上会连出
     * 三行一样的字，反而看不出等了多久、试了几次）。为什么没让成看 hello_app
     * 那边 `[让路] 让路失败：…` 那一行。 */
    printf("[Alarm] 让路轮询了 %d 次、共 %ld ms，hello_app 一直没交出麦克风"
           "（照常出声，这一声可能不响）\n",
           ALARM_YIELD_RETRY_TIMES + 1, waited_ms);
    return false;
}

/**
 * @brief  出声 + 上报（原样搬过来的两句，参数、顺序一个字都没改）
 *
 * 顺序是踩坑换来的：**先出声、再上报**（上报走网络 MQTT/TLS，慢或卡住时声音
 * 必须已经出去了）。页面那一步在调用方（robot_ui_show_alarm）里，仍然排在最前面。
 *
 * ⚠️ 只许工作线程调（出声要动设备，见本节头上那段）。
 */
static void alarm_sound_do(void)
{
    int ret;

    ret = alarm_trigger(ALARM_LEVEL_EMERGENCY, "ui",
                        "报警已触发，请尽快确认");
    if (ret != OK) {
        printf("robot_ui: alarm_trigger failed: %d\n", ret);
    }

    /* 上报：MQTT 发到 zhi_ai/<client_id>/alarm（+ 手机推送）。
     * 用**排队版** report_alarm_queued()，不能用 report_alarm()：本线程不是
     * network_task 的 task group，跨组直接 send() 那个 fd 必然失败（真机日志：
     * `[ALARM] report_alarm type=ui … ret=-1` 紧跟 `MQTT publish failed: -1`）。
     * 排队版只把消息拷进队列、由 network_task 去发，可从任意线程调。 */
    ret = report_alarm_queued("ui", "用户按下报警按钮");
    if (ret < 0) {
        /* 排队口的负值只表示**这条没进队列**：-EINVAL topic 空、-ENOSPC 队列满、
         * -EMSGSIZE 载荷超长（语义见 mqtt_publish_queued()）。跟 MQTT 连没连上
         * 无关 —— 连接由 network_task 自己维持，连不上是它那边重连的事。 */
        printf("robot_ui: report_alarm_queued 没入队: %d"
               "（队列满或载荷超长，这条不会发出去）\n", ret);
    }
}

/**
 * @brief  报警出声线程：等让路 -> 出声 + 上报 -> 报警期间续租，直到报警结束
 *
 * 存活期 = 一次报警：close_alarm 会把 alarm_yield_held 清掉，本线程据此收摊。
 * 等让路、出声、上报、续租**全在这一条线程里串行**（设备动作只有一条线程在做）。
 */
static void *alarm_sound_worker(void *arg)
{
    (void)arg;

    /* ① 有界等麦克风真的交出来。等不到也照常往下走：报警是安全功能，
     *    宁可撞一下（这一声可能不响），也不能因为对方没让就不响。 */
    alarm_wait_mic_released();

    for (;;) {
        bool pending;

        pthread_mutex_lock(&alarm_audio_lock);

        /* ② 出声 + 上报。show_alarm 每调用一次置一次 pending，这里认领；
         *    报警页还开着时又来新的触发（声音检测 / MQTT 反复调 show_alarm）
         *    就再走一遍 —— alarm_trigger 对同级重复触发只更新 reason/text，
         *    不会响两遍（见 board/contest_board/src/sf32lb52_alarm.c）。
         *    判据带上 alarm_yield_held：用户很快点了"返回"（页面已经关了）
         *    就不要在这时候再补一声铃出来。两个判据在同一把锁里读，不会看串。 */
        pending = alarm_sound_pending && alarm_yield_held;
        alarm_sound_pending = false;

        if (!alarm_yield_held) {
            /* 报警页已经关了：收摊。和 show_alarm 共用这把锁 —— "收摊"和
             * "刚关掉就又被 show_alarm 拉起来"只差一步，两边都在锁里改状态，
             * 就不会出现"线程走了、新的触发挂在那里没人做"（那一声报警就永远
             * 不响了）：show_alarm 要么先看到 worker_up 还是真（那它就把 held
             * 置回去，本线程下面那一圈照常出声），要么看到 worker_up 已经是假
             * （那它自己会起一条新线程）。 */
            alarm_worker_up = false;
            pthread_mutex_unlock(&alarm_audio_lock);
            return NULL;
        }

        pthread_mutex_unlock(&alarm_audio_lock);

        if (pending) {
            alarm_sound_do();
        }

        /* ③ 续租（理由见本节头上那段），顺带让出 CPU。 */
        usleep(ALARM_YIELD_RENEW_MS * 1000);
        ai_companion_audio_yield(true);
    }
}

/**
 * @brief  LVGL 线程侧：登记让路请求（只登记一次）并保证出声线程活着
 *
 * 幂等：让路请求只在第一次登记（alarm_yield_held 那道判据），重复调用不会叠出
 * 多次让路、也不会起第二条线程；每次调用只把"这一次要出声 + 上报"挂到 pending。
 * 线程起不来时在这里就地出声兜底 —— 不能因为线程起不来就让这次报警一声不响。
 */
static void alarm_yield_begin(void)
{
    pthread_attr_t attr;
    pthread_t      tid;
    bool           need_yield = false;
    bool           need_start = false;

    pthread_mutex_lock(&alarm_audio_lock);

    if (!alarm_yield_held) {
        alarm_yield_held = true;
        need_yield = true;
    }

    alarm_sound_pending = true;

    if (!alarm_worker_up) {
        alarm_worker_up = true;
        need_start = true;
    }

    pthread_mutex_unlock(&alarm_audio_lock);

    if (need_yield) {
        printf("[Alarm] 让路 —— 先请 hello_app 交出麦克风（非阻塞）\n");
        ai_companion_audio_yield(true);      /* 只登记请求，立刻返回 */
    }

    if (!need_start) {
        return;
    }

    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    pthread_attr_setstacksize(&attr, ALARM_WORKER_STACK_SIZE);

    if (pthread_create(&tid, &attr, alarm_sound_worker, NULL) != 0) {
        pthread_attr_destroy(&attr);

        /* 线程起不来：报警**绝不能因此变成一声不响** —— 就地出声 + 上报
         * （等于退回"没有让路"的老行为，这一声可能不响，但页面、上报都在）。
         * 让路请求**留着不收回**：下一拍 hello_app 真的交出来了，报警后面几轮
         * open 就能响；收回统一留给 close_alarm（配对纪律不破，见下）。 */
        printf("[Alarm] 出声线程起不来（栈/资源不足），就地出声："
               "这一声可能不响，且没有人续租让路\n");

        pthread_mutex_lock(&alarm_audio_lock);
        alarm_worker_up     = false;
        alarm_sound_pending = false;
        pthread_mutex_unlock(&alarm_audio_lock);

        alarm_sound_do();
        return;
    }

    pthread_attr_destroy(&attr);
}

/**
 * @brief  LVGL 线程侧：报警结束，把麦克风还给 hello_app（幂等，只还一次）
 *
 * reclaim 是非阻塞的（只登记方向，重开常开麦由 hello_app 自己的线程做），
 * 所以放在 close_alarm 里无条件调：还出去的一定会还回来。
 * 出声线程看到 alarm_yield_held 变假就自己收摊（最多晚 1 秒那一拍）。
 */
static void alarm_yield_end(void)
{
    bool need_reclaim;

    pthread_mutex_lock(&alarm_audio_lock);
    need_reclaim = alarm_yield_held;
    alarm_yield_held    = false;      /* 出声线程据此收摊 */
    alarm_sound_pending = false;
    pthread_mutex_unlock(&alarm_audio_lock);

    if (need_reclaim) {
        printf("[Alarm] 报警结束：把麦克风还给 hello_app\n");
        ai_companion_mic_reclaim();   /* 非阻塞、无条件可调 */
    }
}

/* ==================== 显示报警 ==================== */
void robot_ui_show_alarm(const char *content)
{
    /* ① 先把红色报警页面切出来（**必须第一步**）。
     *
     * 这一步原来排在报警声和上报后面，实测踩了坑：上报走网络（MQTT + TLS 推送），
     * 一旦这条路上出问题/卡住，红色页面就永远切不出来 —— 现场看到的现象就是
     * "按了报警，声音在响但屏幕上没有报警页，也退不出来"。
     * 先刷页面，后面无论网络怎么慢，用户至少能看到报警界面并点"返回"。
     */
    lv_scr_load(scr_alarm);

    /* 设置报警表情 */
    robot_ui_set_face(ROBOT_FACE_ALARM);
    robot_ui_set_status(ROBOT_STATUS_ALARM);

    /* 启动报警闪烁：先复位成满不透明，再让 2Hz 定时器跑起来 */
    if (lbl_alarm_icon != NULL)
    {
        lv_obj_set_style_opa(lbl_alarm_icon, LV_OPA_COVER, 0);
    }
    if (alarm_blink_timer != NULL)
    {
        lv_timer_resume(alarm_blink_timer);
    }

    /* ② 设备级动作：**先请 hello_app 让路**，再让喇叭真的响起来（板级报警模块）。
     *
     * 放在这个函数里、而不是各个调用点，是因为界面上的"报警"按钮走的是
     *   btn_event_handler() -> robot_ui_show_alarm()
     * 根本不经过 main.c；而这里是所有报警入口（按钮 / MQTT 的 start_alarm /
     * 声音检测回调）唯一的汇合点，改一处就全接上了。也不会重复触发：
     * alarm_trigger() 对同级或更低的重复触发只更新 reason/text，不重来。
     *
     * ★ 让路（2026-09-15 补）：出声之前必须先请 hello_app 交出麦克风 —— 常开麦
     *   占着设备时，报警的 CONFIGURE/START 会被音频上层**静默跳过**，结果是
     *   全程无声（根因链和现场实证见上面那一节的说明）。
     *   这里只做两件**非阻塞**的事：登记让路请求（只登记一次）+ 起一条出声工作
     *   线程（在那条线程里：有界等让路 -> alarm_trigger + 上报 -> 报警期间续租）。
     *   出声 + 上报（原来的 ②③）因此比原来晚"让路那一下"（通常 100~400ms，
     *   最坏 3.3 秒），参数和顺序一个字都没改，见 alarm_sound_do()。
     *
     *   "等让路"**绝不能**就地做在这个函数里：它跑在 **LVGL 线程**，而轮询最长
     *   3.3 秒 —— 等在 LVGL 线程里就是把界面冻住（见上面那一节）。所以一切跟
     *   等/出声有关的事都在工作线程里做。
     *   重复调用（现场日志里每几秒一次）不会叠出多次让路、也不会起第二条线程。
     *
     * 这里**不受 main.c 里 g_ai_initialized / #if 0 的影响**：robot_ui.c
     * 完全不引用那个标志，所以 AI 初始化整块停用也照样出声。
     */
    alarm_yield_begin();
}

/* ==================== 关闭报警 ==================== */
void robot_ui_close_alarm(void)
{
    int ret;

    /* 设备级动作：停掉报警声（没有在报警时是安全空操作） */
    ret = alarm_clear();
    if (ret != OK) {
        printf("robot_ui: alarm_clear failed: %d\n", ret);
    }

    /* 报警结束：把麦克风还给 hello_app（非阻塞、**只还一次**）。
     * 让路是 show_alarm 那次借的，配对纪律要求"借了就得还" —— 漏一次 reclaim
     * hello_app 就一直聋着（它的让路看门狗 10 秒后会强制收回兜底，但正常路径
     * 不该走到那里）。放在 alarm_clear() 之后：先把对方的出声请求停掉，再把
     * 设备还回去。理由和 reclaim 为什么非阻塞见本节前面那一节。 */
    alarm_yield_end();

    /* 停止报警闪烁（并复位成满不透明，免得下次进报警页时停在半透明的状态） */
    if (alarm_blink_timer != NULL)
    {
        lv_timer_pause(alarm_blink_timer);
    }
    if (lbl_alarm_icon != NULL)
    {
        lv_obj_set_style_opa(lbl_alarm_icon, LV_OPA_COVER, 0);
    }

    /* 返回主界面 */
    lv_scr_load(scr_main);

    /* 恢复正常状态 */
    robot_ui_set_face(ROBOT_FACE_HAPPY);
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

/* ==================== 时钟刷新（主循环调用） ==================== */
void robot_ui_update_time(void)
{
    ui_clock_refresh();
}
