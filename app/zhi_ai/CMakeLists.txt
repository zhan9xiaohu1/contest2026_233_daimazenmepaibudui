if(CONFIG_LVX_USE_CONTEST2026_233_ZHI_AI)
  nuttx_add_application(
    NAME zhi_ai
    SRCS zhi_ai_main.c
    STACKSIZE ${CONFIG_ZHI_AI_STACKSIZE}
    PRIORITY ${CONFIG_ZHI_AI_PRIORITY})
endif()
