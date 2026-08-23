# app/robot_ui/CMakeLists.txt

if(CONFIG_LVX_USE_CONTEST2026_233_ZHI_AI)
  nuttx_add_application(
    NAME robot_ui
    SRCS main.c robot_ui.c
    STACKSIZE ${CONFIG_DEFAULT_TASK_STACKSIZE}
    PRIORITY 100
  )
endif()
