# SF32LB52 音频驱动使用说明

> 智爱陪伴 —— openvela 音频输入输出接口（成员一交付物）
> 硬件：SF32LB52-DevKit-LCD（板上 MEMS 麦克风 + AW8155 功放 + 外接喇叭）

## 1. 概述

在 openvela（NuttX）上为 SF32LB52 实现了标准音频设备驱动，注册为 **`/dev/audio/audio0`**，应用层通过 NuttX 标准音频接口操作：

- **播放**（喇叭输出）：`write()` 写入 PCM 数据 → AUDPRC TX0 DMA → codec DAC → AW8155 功放 → 喇叭
- **录音**（麦克风输入）：`read()` 读取 PCM 数据 ← AUDPRC RX0 DMA ← codec ADC ← 板上 MEMS 麦克风
- 采样率 8k/16k/44.1k/48k，16bit，单声道

## 2. 硬件通路

```
播放：内存 → AUDPRC TX0 DMA → AUDPRC → codec DAC → AW8155(PA10) → 喇叭
录音：麦克风 → codec ADC → AUDPRC → AUDPRC RX0 DMA → 内存
```

- codec（AUDCODEC）：模拟前端（DAC/ADC 模拟通路），不走 DMA
- AUDPRC：数字音频处理器，数据通路（TX0 播放 DMA / RX0 录音 DMA）
- 功放：AW8155，PA10（AU_PA_EN）控制

## 3. 驱动接口（应用层用法）

### 3.1 播放（输出）

```c
#include <nuttx/audio/audio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

int fd = open("/dev/audio/audio0", O_WRONLY);

/* 配置：16kHz 单声道 16bit */
struct audio_caps_desc_s capdesc;
memset(&capdesc, 0, sizeof(capdesc));
capdesc.caps.ac_len      = sizeof(struct audio_caps_s);
capdesc.caps.ac_type     = AUDIO_TYPE_OUTPUT;
capdesc.caps.ac_channels = 1;
capdesc.caps.ac_controls.hw[0] = 16000;  /* 采样率 */
capdesc.caps.ac_controls.b[2]  = 16;     /* 位深 */
ioctl(fd, AUDIOIOC_CONFIGURE, (unsigned long)&capdesc);

ioctl(fd, AUDIOIOC_START, 0);           /* 启动通路（含功放） */

write(fd, pcm_data, bytes);             /* 播放 PCM 数据（同步，播完返回） */

ioctl(fd, AUDIOIOC_STOP, 0);            /* 停止（关功放） */
close(fd);
```

### 3.2 录音（输入）

```c
int fd = open("/dev/audio/audio0", O_RDONLY);

struct audio_caps_desc_s capdesc;
memset(&capdesc, 0, sizeof(capdesc));
capdesc.caps.ac_len      = sizeof(struct audio_caps_s);
capdesc.caps.ac_type     = AUDIO_TYPE_INPUT;   /* 输入方向 */
capdesc.caps.ac_channels = 1;
capdesc.caps.ac_controls.hw[0] = 16000;
capdesc.caps.ac_controls.b[2]  = 16;
ioctl(fd, AUDIOIOC_CONFIGURE, (unsigned long)&capdesc);

ioctl(fd, AUDIOIOC_START, 0);

read(fd, pcm_buf, bytes);               /* 采集 PCM 数据（同步，采满返回） */

ioctl(fd, AUDIOIOC_STOP, 0);
close(fd);
```

> 说明：`write`/`read` 为同步接口（内部 DMA + 完成中断 + 信号量），一次调用对应一段连续音频。`write` 返回实际写入字节数，`read` 返回实际采集字节数；返回 0 表示失败（设备未启动/超时）。

## 4. 测试命令 `audio_test`

编译进固件（`CONFIG_EXAMPLES_AUDIO_TEST=y`），nsh 下运行：

```sh
# 播放 1kHz 正弦波 3 秒（听喇叭）
audio_test 3000 1000

# 录音 3 秒（对麦克风说话），打印峰值/均值判断是否采到声音
audio_test record 3000
```

录音输出示例（采到声音）：

```
READ done: 32000 of 32000 bytes
RECORD peak=16383 avg=5182 (16k mono 16bit)
RECORD OK: 检测到声音
```

`peak` > 500 表示有信号；接近 0 表示静音/通路问题。

## 5. 音量控制

音量通过 `AUDIOIOC_CONFIGURE` + `AUDIO_TYPE_FEATURE` 设置（dB 值，如 -18）：

```c
capdesc.caps.ac_type = AUDIO_TYPE_FEATURE;
capdesc.caps.ac_format.hw = AUDIO_FU_VOLUME;
capdesc.caps.ac_controls.hw[0] = -18;  /* 音量 dB */
ioctl(fd, AUDIOIOC_CONFIGURE, (unsigned long)&capdesc);
```

默认音量 -18dB。

## 6. 集成到 AI 应用（成员二参考）

`ai_audio` 模块从"模拟接口"切换为真实音频：

1. 初始化：`open("/dev/audio/audio0", ...)` + CONFIGURE（按需 16k/单声道/16bit）
2. 录音：`read()` 拿 PCM 数据 → 上传/处理
3. 播放：`write()` 播放回复音频（需先做 TTS/采样率转换到 16k 16bit mono）
4. 注意：`read`/`write` 是同步阻塞的，建议放独立线程；录音播放不要同时进行（单工）

## 7. 改动文件清单

### 团队仓库（本仓库）

| 文件 | 说明 |
|---|---|
| `app/audio_test/`（新增） | 音频测试命令（播放/录音） |
| `app/robot_ui/CMakeLists.txt` | 修复构建开关（误用 zhi_ai 的开关） |
| `board/contest_board/configs/sf32lb52_ai/defconfig` | 启用 `CONFIG_EXAMPLES_AUDIO_TEST` |
| `contest2026_233_daimazenmepaibudui.xml` | manifest 增加 audio_test linkfile |
| `docs/audio_driver_usage.md` | 本文档 |

### 上游 openvela（vendor_sifli 仓库，另行提交）

| 文件 | 说明 |
|---|---|
| `vendor/sifli/boards/sf32lb52/sf32lb52_devkit_lcd/src/sf32lb52_audio.c`（新增） | 音频驱动（~1000 行） |
| 同目录 `sf32lb52_audio.h`（新增） | 驱动头 |
| 同目录 `CMakeLists.txt` | 加入驱动源文件 |
| 同目录 `sifli_ap.c` | bringup 注册 `/dev/audio0` |
| `apps/builtin/builtin_list.h` / `builtin_proto.h` | 注册 audio_test 命令 |
| `apps/packages/demos/Kconfig` | 收录 audio_test 的 Kconfig |

> vendor_sifli 的改动在 openvela 工作区的上游仓库，需通过 PR 提交到 open-vela/vendor_sifli（或按队伍流程处理）。

## 8. 已知限制

- 单工：录音/播放不能同时进行（硬件通路只配置一个方向）
- 录音无增益调节接口（codec 默认增益）
- 播放数据需为 16k/16bit/单声道 PCM（内部不做重采样）
