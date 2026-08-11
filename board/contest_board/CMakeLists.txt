# ##############################################################################
# Contest 2026 Team 233 - 智爱陪伴 Board CMakeLists.txt
# Based on sf32lb52_devkit_lcd
# ##############################################################################

add_subdirectory(src)

file(GLOB EXTRA_LIBS
     ${NUTTX_BOARD_ABS_DIR}/libs/${CONFIG_ARCH_BOARD_CUSTOM_NAME}/*.a)

set_property(GLOBAL APPEND PROPERTY NUTTX_EXTRA_LIBRARIES ${EXTRA_LIBS})

set_property(GLOBAL PROPERTY LD_SCRIPT ${NUTTX_BOARD_ABS_DIR}/scripts/ld.script)
