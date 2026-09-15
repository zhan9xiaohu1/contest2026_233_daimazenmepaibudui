/****************************************************************************
 * vendor/sifli/boards/sf32lb52/sf32lb52_devkit_lcd/sf32lb52_devkit_lcd.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __VENDOR_SIFLI_BOARDS_SF32LB52_SF32LB52_DEVKIT_LCD_SF32LB52_DEVKIT_LCD_H
#define __VENDOR_SIFLI_BOARDS_SF32LB52_SF32LB52_DEVKIT_LCD_SF32LB52_DEVKIT_LCD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of GPIO pins */
#define BOARD_NGPIOIN     1
#define BOARD_NGPIOOUT    1
#define BOARD_NGPIOINT    1

#define GPIO_IN1          (GET_PIN_2(hwp_gpio1, 34))
#define GPIO_OUT1         (GET_PIN_2(hwp_gpio1, 26))

#define GPIO_INT1         (GET_PIN_2(hwp_gpio1, 34))



int sf32lb52_devkit_lcd_bringup(void);

/****************************************************************************
 * Name:  sf32lb_lcd_panel_reinit
 *
 * Description:
 *   面板重新初始化（黑屏救回）。**实现在 vendor 树的面板驱动里**：
 *   vendor/sifli/boards/sf32lb52/drivers/lcd/sf32lb_lcd.c —— 它要够到
 *   s_drv_lcd.hlcdc 和面板 ops，那些是那份文件的 static，拿不到这里来。
 *   声明放本头文件是因为 app 的 CMakeLists.txt 都有
 *   ${NUTTX_BOARD_ABS_DIR}/src（driver 那个目录不在 app 的 -I 里）。
 *
 *   重发一遍面板初始化序列（含拉一次 RESET 脚）+ 重设像素格式/亮度 +
 *   DisplayOn；只动面板和 LCDC，**不碰 LVGL**。要"重初始化 + 全屏重绘"一起
 *   做的，用 robot_ui 的 robot_ui_bridge_panel_reinit()。
 *
 *   Returned Value: OK / -ENODEV / -EINVAL / -ENOSYS，见驱动里那段注释。
 ****************************************************************************/

int sf32lb_lcd_panel_reinit(void);

#ifdef CONFIG_INPUT_BUTTONS
int sf32lb52_button_initialize(const char *devname);
#endif


#ifdef CONFIG_DEV_GPIO
int sifli_gpio_initialize(void);
#endif

#endif /* __VENDOR_SIFLI_BOARDS_SF32LB52_SF32LB52_DEVKIT_LCD_SF32LB52_DEVKIT_LCD_H */

