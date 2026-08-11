# ##############################################################################
# Contest 2026 Team 233 - 智爱陪伴 Board src/CMakeLists.txt
# Based on sf32lb52_devkit_lcd
# ##############################################################################

set(SRCS sifli_ap.c sifli_gpio.c sf32lb52_buttons.c bsp_init.c bsp_lcd_tp.c bsp_pinmux.c bsp_power.c )

nuttx_add_romfs(
  NAME
  etc
  MOUNTPOINT
  etc
  RCSRCS
  etc/init.d/rcS
  etc/init.d/rc.sysinit
  RCRAWS
  etc/group
  etc/1.txt
  PATH
  ${CMAKE_CURRENT_LIST_DIR}/etc)

target_link_libraries(board PRIVATE romfs_etc)

target_sources(board PRIVATE ${SRCS})

target_compile_definitions(board PRIVATE SOC_BF0_HCPU
                                         SF32LB52X
                                         HAL_TICK_PER_SECOND=1000
                                         USE_HAL_DRIVER)

target_include_directories(board PRIVATE ${NUTTX_CHIP_ABS_DIR}/include)
target_include_directories(board PRIVATE ${NUTTX_CHIP_ABS_DIR}/../drivers/Include)
target_include_directories(board PRIVATE ${NUTTX_CHIP_ABS_DIR}/../drivers/cmsis/Include)
target_include_directories(board PRIVATE ${NUTTX_CHIP_ABS_DIR}/../drivers/cmsis/sf32lb52x)
target_include_directories(board PRIVATE ${NUTTX_CHIP_ABS_DIR}/../external/CMSIS/Include)
target_include_directories(board PRIVATE ${NUTTX_CHIP_ABS_DIR}/../boards/include/config/sf32lb52x)

target_compile_options(board PRIVATE
    $<$<COMPILE_LANGUAGE:C>:-w>
    $<$<COMPILE_LANGUAGE:CXX>:-w>)
