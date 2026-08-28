/****************************************************************************
 * app/audio_test/main.c
 *
 * SF32LB52 /dev/audio/audio0 音频通路测试
 * 生成 1kHz 正弦波并播放（喇叭到货后应能听到"哔——"声）
 *
 * 用法: audio_test [duration_ms] [freq_hz]
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/audio/audio.h>

#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define AUDIO_DEV      "/dev/audio/audio0"
#define SAMPLE_RATE    16000
#define CHANNELS       1
#define BITS           16

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int gen_sine(FAR int16_t *buf, int nsamples, int freq)
{
  int i;

  for (i = 0; i < nsamples; i++)
    {
      buf[i] = (int16_t)(32767.0 * 0.5 *
               sin(2.0 * M_PI * freq * (double)i / SAMPLE_RATE));
    }

  return nsamples;
}

static void print_caps(int fd)
{
  struct audio_caps_s caps;
  int ret;

  memset(&caps, 0, sizeof(caps));
  caps.ac_len     = sizeof(caps);
  caps.ac_type    = AUDIO_TYPE_QUERY;
  caps.ac_subtype = AUDIO_TYPE_QUERY;

  ret = ioctl(fd, AUDIOIOC_GETCAPS, (unsigned long)&caps);
  printf("GETCAPS ret=%d type=0x%02x fmt=0x%08lx\n",
         ret, caps.ac_controls.b[0],
         (unsigned long)caps.ac_format.hw);
}

static int configure_output(int fd)
{
  struct audio_caps_desc_s capdesc;
  int ret;

  memset(&capdesc, 0, sizeof(capdesc));
  capdesc.caps.ac_len      = sizeof(struct audio_caps_s);
  capdesc.caps.ac_type     = AUDIO_TYPE_OUTPUT;
  capdesc.caps.ac_channels = CHANNELS;
  capdesc.caps.ac_controls.hw[0] = SAMPLE_RATE;
  capdesc.caps.ac_controls.b[2]  = BITS;

  ret = ioctl(fd, AUDIOIOC_CONFIGURE, (unsigned long)&capdesc);
  printf("CONFIGURE ret=%d (%dHz %dch %dbit)\n",
         ret, SAMPLE_RATE, CHANNELS, BITS);
  return ret;
}

static int configure_input(int fd)
{
  struct audio_caps_desc_s capdesc;
  int ret;

  memset(&capdesc, 0, sizeof(capdesc));
  capdesc.caps.ac_len      = sizeof(struct audio_caps_s);
  capdesc.caps.ac_type     = AUDIO_TYPE_INPUT;
  capdesc.caps.ac_channels = CHANNELS;
  capdesc.caps.ac_controls.hw[0] = SAMPLE_RATE;
  capdesc.caps.ac_controls.b[2]  = BITS;

  ret = ioctl(fd, AUDIOIOC_CONFIGURE, (unsigned long)&capdesc);
  printf("CONFIGURE ret=%d (%dHz %dch %dbit input)\n",
         ret, SAMPLE_RATE, CHANNELS, BITS);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int duration_ms = 1000;
  int freq        = 1000;
  int nsamples;
  FAR int16_t *buf;
  int fd;
  int ret;
  ssize_t n;
  bool record = false;

  if (argc > 1)
    {
      if (strcmp(argv[1], "record") == 0)
        {
          record = true;
        }
      else
        {
          duration_ms = atoi(argv[1]);
        }
    }

  if (argc > 2)
    {
      freq = atoi(argv[2]);
    }

  nsamples = SAMPLE_RATE * duration_ms / 1000;

  if (record)
    {
      printf("audio_test: record %dms via %s\n", duration_ms, AUDIO_DEV);
    }
  else
    {
      printf("audio_test: play %dHz %dms via %s\n", freq, duration_ms,
             AUDIO_DEV);
    }

  buf = (FAR int16_t *)malloc(nsamples * sizeof(int16_t));
  if (buf == NULL)
    {
      printf("malloc failed\n");
      return 1;
    }

  if (!record)
    {
      gen_sine(buf, nsamples, freq);
    }

  fd = open(AUDIO_DEV, record ? O_RDONLY : O_WRONLY);
  if (fd < 0)
    {
      printf("open %s failed: %d\n", AUDIO_DEV, fd);
      free(buf);
      return 1;
    }

  print_caps(fd);

  ret = record ? configure_input(fd) : configure_output(fd);
  if (ret < 0)
    {
      printf("configure failed: %d\n", ret);
      goto out;
    }

  ret = ioctl(fd, AUDIOIOC_START, 0);
  printf("START ret=%d\n", ret);
  if (ret < 0)
    {
      goto out;
    }

  if (record)
    {
      int i;
      int peak = 0;
      long sum = 0;

      n = read(fd, buf, nsamples * sizeof(int16_t));
      printf("READ done: %zd of %d bytes\n", n, nsamples * 2);

      for (i = 0; i < nsamples; i++)
        {
          int v = buf[i];

          if (v < 0)
            {
              v = -v;
            }

          if (v > peak)
            {
              peak = v;
            }

          sum += v;
        }

      printf("RECORD peak=%d avg=%ld (16k mono 16bit)\n",
             peak, nsamples > 0 ? sum / nsamples : 0);
      printf("RECORD %s\n", peak > 500 ? "OK: 检测到声音" :
             "静音: 麦克风无信号或通路未通");
    }
  else
    {
      n = write(fd, buf, nsamples * sizeof(int16_t));
      printf("WRITE done: %zd of %d bytes\n", n, nsamples * 2);
    }

  ioctl(fd, AUDIOIOC_STOP, 0);
  printf("STOP done\n");

out:
  close(fd);
  free(buf);
  printf("audio_test: done\n");
  return 0;
}
