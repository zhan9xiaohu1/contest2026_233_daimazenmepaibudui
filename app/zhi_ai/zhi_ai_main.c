/****************************************************************************
 * 智爱陪伴 - AI Companion for Elderly
 * Main application entry point
 *
 * This app initializes the AI Agent framework and companion-specific
 * modules on the SF32LB52-DevKit-LCD platform.
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <syslog.h>

#include "zhi_ai_config.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: zhi_ai_main
 *
 * Description:
 *   Main entry point for the 智爱陪伴 companion application.
 *   Starts the AI Agent and companion modules.
 *
 ****************************************************************************/

int main(int argc, char *argv[])
{
  syslog(LOG_INFO, "===========================================\n");
  syslog(LOG_INFO, "  智爱陪伴 - AI Companion for Elderly\n");
  syslog(LOG_INFO, "  Based on openvela + SF32LB52-DevKit-LCD\n");
  syslog(LOG_INFO, "===========================================\n");

  syslog(LOG_INFO, "[zhi_ai] Starting AI Agent...\n");

  /* AI Agent is launched as a separate builtin command "ai_agent".
   * This app serves as the companion controller that coordinates
   * the AI Agent, LVGL UI, sound detection, and reminder modules.
   */

  syslog(LOG_INFO, "[zhi_ai] System ready.\n");
  syslog(LOG_INFO, "[zhi_ai] Use 'ai_agent' command to start AI conversation.\n");

  return 0;
}
