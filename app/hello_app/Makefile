############################################################################
# AI Companion App - Makefile
# 智爱陪伴 - AI老人陪伴守护终端
############################################################################

include $(APPDIR)/Make.defs

# 程序配置

PROGNAME  = ai_companion
PRIORITY  = SCHED_PRIORITY_DEFAULT
STACKSIZE = 4096
MODULE    = $(CONFIG_LVX_USE_DEMO_CONTEST2026_000_HELLO_APP)

# 源文件

MAINSRC = ai_companion_main.c

CSRCS = ai_state_machine.c \
        ai_audio.c \
        ai_llm.c \
        ai_sound_detect.c \
        ai_care.c

# 头文件路径

CFLAGS += -I$(APPDIR)/demos/contest2026_233_hello_app

include $(APPDIR)/Application.mk
