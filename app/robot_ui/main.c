/**
 * main.c - 智爱陪伴应用入口
 * 在 openvela 中运行
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <unistd.h>
#include "robot_ui.h"

/* LVGL 定时器 */
static void lvgl_timer_handler(void)
{
    lv_timer_handler();
}

/* 主函数 */
int main(int argc, char *argv[])
{
    printf("智爱陪伴机器人启动中...\n");

    /* 初始化 UI */
    robot_ui_init();

    /* 设置初始状态 */
    robot_ui_set_status(ROBOT_STATUS_IDLE);
    robot_ui_set表情(ROBOT表情_HAPPY);
    robot_ui_set_ai_reply("你好！我是智爱陪伴。\n有什么我可以帮你的吗？");

    printf("UI 初始化完成！\n");

    /* 主循环 */
    while (1) {
        lvgl_timer_handler();
        usleep(5000); // 5ms 刷新周期
    }

    return 0;
}
