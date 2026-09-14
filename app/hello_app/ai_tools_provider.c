/****************************************************************************
 * AI Tools Provider Implementation
 * 智爱陪伴 - 把我们的工具注册进 openvela 框架的 ai_agent 工具表
 *
 * 实现要点（背景见 ai_tools_provider.h 的「运行时机」）：
 *
 *   1. 本文件是 hello_app 编的，但 execute 回调**跑在 ai_agent 的任务里** ——
 *      ai_agent 的 agent_loop_task 收到消息后会重新取一次工具清单
 *      （tool_registry_get_tools_json()），把清单拼进请求；模型要求调用工具时
 *      由 tool_registry_execute() 先查内置工具、再查 MCP、最后才轮到我们这些
 *      注册进来的 provider。所以回调和 ai_companion 的主循环是并发的：
 *      这里既不能碰 LVGL，也不能碰 hello_app 的界面状态。
 *
 *   2. JSON 全部手拼，**不用 cJSON**：hello_app 的编译参数里没有 cJSON 的
 *      include 路径（mimo_voice.c / mimo_location.c 的文件头都记着这条），
 *      而且工具参数只有两个短字符串，引 cJSON 换来的是一份额外的解析代码和
 *      堆拷贝，板子上不划算。
 *
 *   3. 工具清单的 JSON 形状是框架定的：数组元素取 name / description /
 *      input_schema 三个键（llm_parse.c 的 build_openai_tools_array() 就是按
 *      这三个键转成 OpenAI 的 function 格式）。input_schema 是标准 JSON
 *      Schema。
 *
 *   4. 工具执行**不吞错**：网络没连上、参数不合法，都把原因写进 output 回给
 *      模型，让它用自己的话告诉老人（返回 ERROR 会让工具调用链断掉，老人
 *      只会等到一句干巴巴的失败）。只有「这个名字不是我们的工具」才回
 *      ERROR，让后面注册的 provider 有机会兜。
 *
 * 返回给框架的清单字符串必须是用 malloc 出来的（框架那边按 free() 释放）。
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "ai_tools_provider.h"

/* 框架的工具 provider 接口（tool_registry_register_provider / _invalidate）。
 * -I<ai_agent>/src 只在开了 CONFIG_HELLO_APP_LLM_AI_AGENT 时才加进编译命令，
 * 所以这一条 include 和下面所有用到框架的部分都得跟着条件编译 —— 否则
 * 换个不启用 ai_agent 的配置，hello_app 会因为找不到头文件直接编不过。 */

#ifdef CONFIG_HELLO_APP_LLM_AI_AGENT
#  include "tools/tool_registry.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 日志。工具执行在 ai_agent 的任务里，printf 走的是同一条串口（平铺构建），
 * 所以这里的输出会和 ai_agent 的日志混在一起，加个前缀方便捞。 */

#define ai_tools_info(fmt, ...)  printf("[AI_TOOLS] " fmt "\n", ##__VA_ARGS__)

/* device_id 最长多少字节（够装 "living_room_light" 这类标识，多出来的截断
 * 反而会发出一台不存在的设备，不如直接打回去让模型重说） */
#define AI_TOOLS_DEVICE_MAX     64

/* command 最长多少字节。on / off 各 2 字节，留点余量只为判断"用户给的长"
 * 这种情况，多余的一律按不合法处理。 */
#define AI_TOOLS_COMMAND_MAX    16

/* 工具清单的 JSON。description 是写给模型看的，里面一律用中文标点 —— 这是
 * 手写的 JSON 字面量，一个 ASCII 引号或反斜杠就能把它拼坏。
 *
 * 清单里每个元素是框架要的形状：name + description + input_schema。 */

#define AI_TOOLS_SET_LIGHT_JSON                                          \
  "{\"name\":\"" AI_TOOL_SET_LIGHT "\","                                 \
  "\"description\":\"打开或关掉屋子里的灯。用户说开灯、关灯、把灯打开、"                       \
  "把灯关掉、太暗了、屋里黑之类的话时调用。device_id 一般不用传，"                        \
  "用户没说具体哪盏灯就用默认的那盏。\","                                        \
  "\"input_schema\":{\"type\":\"object\",\"properties\":{"               \
  "\"device_id\":{\"type\":\"string\",\"description\":\"灯的标识，例如 "     \
  AI_TOOLS_LIGHT_DEVICE_ID "；用户没说哪盏灯就不要传这个字段，"                 \
  "板子会自己用默认值。\"},"                                              \
  "\"command\":{\"type\":\"string\",\"description\":\"on 表示开灯，"      \
  "off 表示关灯，这两个之外的值一律无效。\","                                   \
  "\"enum\":[\"on\",\"off\"]}},"                                        \
  "\"required\":[\"command\"]}}"

#define AI_TOOLS_JSON_ARRAY  "[" AI_TOOLS_SET_LIGHT_JSON "]"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* 往哪张网络上下文上发设备命令。由 ai_tools_provider_set_network_ctx() 在
 * ai_companion 初始化阶段写一次，之后只读 —— 读它的可能是 ai_agent 的任务，
 * 所以约定"设完不再改"（见头文件）。 */

static ai_network_context_t *g_net_ctx = NULL;

/* 已经注册过没有。provider 表只有 4 个名额，而且它没有反注册接口，
 * 重复注册的后果是工具清单里出现两份同名工具（服务端可能直接 400）。
 * 只有真的能注册的那种配置才需要这个标志，否则它就是个没人看的静态变量。 */

#ifdef CONFIG_HELLO_APP_LLM_AI_AGENT
static bool g_registered = false;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**
 * @brief  在 JSON 串里找一个字符串字段的原始字节（不复制、不反转义）
 *
 * 写法照抄 mimo_voice.c 里那份最小的实现（那边不引 cJSON 的理由在这里同样
 * 成立）：先找 "key"，跳过冒号和空白，拿引号之间的原始字节，跳过 \" 。
 * 找不到 / 类型不对都返回 NULL。
 *
 * @param  json     被查的 JSON（NULL 当"没有"）
 * @param  key      字段名
 * @param  out_len  输出：字段值的字节数（不含引号）
 * @return 指向值的指针，NULL 表示没有这个字段
 */

static const char *ai_tools_json_find_string(const char *json, const char *key,
                                             size_t *out_len)
{
  char pat[32];
  const char *p;

  if (json == NULL || key == NULL || out_len == NULL)
    {
      return NULL;
    }

  if (strlen(key) + 3 > sizeof(pat))
    {
      return NULL;
    }

  snprintf(pat, sizeof(pat), "\"%s\"", key);

  p = strstr(json, pat);
  if (p == NULL)
    {
      return NULL;
    }

  p += strlen(pat);

  while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')
    {
      p++;
    }

  if (*p != ':')
    {
      return NULL;
    }

  p++;

  while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')
    {
      p++;
    }

  if (*p != '"')
    {
      return NULL;
    }

  p++;

  {
    const char *start = p;

    while (*p != '\0' && *p != '"')
      {
        if (*p == '\\' && p[1] != '\0')
          {
            p++;
          }

        p++;
      }

    if (*p != '"')
      {
        return NULL;
      }

    *out_len = (size_t)(p - start);
    return start;
  }
}

/**
 * @brief  把一个字段值拷进定长缓冲
 *
 * 只接受**纯 ASCII 记号**：带反斜杠（转义序列）或者长度越界的都当场判不合法。
 * 我们的参数本来就是 on / off / 设备标识这类记号，没有必要实现完整的反转义；
 * 反过来，装作支持转义却只做半套，会让 \" 这种输入悄悄变成一个引号，不如直接
 * 拒绝、让模型重新说一遍。
 *
 * @param  src      ai_tools_json_find_string() 给的原始字节
 * @param  len      字节数
 * @param  dst      输出缓冲（一定以 '\0' 结尾）
 * @param  dst_cap  输出缓冲大小
 * @return true 拷贝成功；false 值不合法或装不下
 */

static bool ai_tools_copy_token(const char *src, size_t len, char *dst,
                                size_t dst_cap)
{
  size_t i;

  if (src == NULL || dst == NULL || dst_cap == 0)
    {
      return false;
    }

  dst[0] = '\0';

  if (len == 0 || len >= dst_cap)
    {
      return false;
    }

  for (i = 0; i < len; i++)
    {
      unsigned char c = (unsigned char)src[i];

      if (c < 0x20 || c == '\\')
        {
          return false;
        }

      dst[i] = (char)c;
    }

  dst[len] = '\0';
  return true;
}

/**
 * @brief  框架的 get_tools 回调：交回我们的工具清单
 *
 * 每次重建清单都会调一次，所以这里每次都返回一份新的堆内存（框架负责 free）。
 * 清单是编译期常量，不用加锁。
 *
 * @return malloc 出来的 JSON 数组字符串；NULL 表示这次没有工具
 */

static char *ai_tools_provider_get_tools(void)
{
  size_t len = strlen(AI_TOOLS_JSON_ARRAY) + 1;
  char *out = malloc(len);

  if (out == NULL)
    {
      return NULL;
    }

  memcpy(out, AI_TOOLS_JSON_ARRAY, len);
  return out;
}

/**
 * @brief  框架的 execute 回调：执行一个工具调用
 *
 * @param  name         模型要调的工具名
 * @param  input_json   模型给的参数（JSON 对象串；可能为 NULL）
 * @param  output       工具结果（回给模型看的文本，必须留 '\0' 的位置）
 * @param  output_size  output 的大小
 * @return OK     - 名字是我们的（不管执行成功还是失败，结果都在 output 里）
 *         ERROR  - 不是我们的工具，让框架继续问下一个 provider
 */

static int ai_tools_provider_execute(const char *name, const char *input_json,
                                     char *output, size_t output_size)
{
  char device[AI_TOOLS_DEVICE_MAX];
  char command[AI_TOOLS_COMMAND_MAX];
  const char *p = NULL;
  size_t len = 0;
  int ret;

  if (name == NULL || output == NULL || output_size == 0)
    {
      return ERROR;
    }

  if (strcmp(name, AI_TOOL_SET_LIGHT) != 0)
    {
      /* 不是我们的工具：**必须**回 ERROR，否则框架会以为已经被处理掉了，
       * 排在后面的 provider（还有后加的）就永远轮不上。 */

      return ERROR;
    }

  output[0] = '\0';

  /* command 必填。缺了 / 值是 on/off 之外的，都把原因写成一句人话回给模型，
   * 让它自己跟老人解释或者再问一遍（返回 ERROR 会走成"未知工具"，模型只
   * 会看到一句英文报错）。 */

  p = ai_tools_json_find_string(input_json, "command", &len);

  if (!ai_tools_copy_token(p, len, command, sizeof(command)))
    {
      snprintf(output, output_size,
               "没执行：command 参数缺失或者不是合法的 on / off。"
               "请按 on 或 off 重新调用一次。");
      ai_tools_info("set_light: 参数不合法，打回");
      return OK;
    }

  if (strcmp(command, "on") != 0 && strcmp(command, "off") != 0)
    {
      snprintf(output, output_size,
               "没执行：command 只能是 on 或者 off，收到的是别的值。"
               "请换算成 on（开灯）或 off（关灯）重新调用。");
      ai_tools_info("set_light: command 值不合法，打回");
      return OK;
    }

  /* device_id 可选：模型没给（或者给得不合法）就用和 ai_companion_main.c 里
   * LIGHT_DEVICE_ID 同一个默认设备 —— 队友的灯模拟器只认那一台。 */

  snprintf(device, sizeof(device), "%s", AI_TOOLS_LIGHT_DEVICE_ID);

  p = ai_tools_json_find_string(input_json, "device_id", &len);
  if (p != NULL && !ai_tools_copy_token(p, len, device, sizeof(device)))
    {
      snprintf(device, sizeof(device), "%s", AI_TOOLS_LIGHT_DEVICE_ID);
    }

  if (g_net_ctx == NULL)
    {
      /* 网络模块还没起 / 已经拆了。这条命令不会发出去，但工具本身执行过了 ——
       * 老实告诉模型，别让它对老人说"灯打开了"。 */

      snprintf(output, output_size,
               "没执行：灯控还没接上网络模块（网络上下文没注册），"
               "这次命令没有发出去。");
      ai_tools_info("set_light: 网络上下文为空，命令没发");
      return OK;
    }

  /* 发到 zhi_ai/<client_id>/device_cmd。command 必须是**裸字符串** on/off：
   * 队友的灯模拟器收的是 network_comm.c 里 send_device_command() 拼的
   * {"device_id":..., "command":"on"}，这里再包一层 JSON 就没法解析了。 */

  ret = ai_network_send_device_command(g_net_ctx, device, command);

  if (ret < 0)
    {
      /* 负值 = 命令**没进发送队列**（-ENOTCONN 网络没连上、-ENOSPC 队列满、
       * -EMSGSIZE 载荷超长、-EINVAL 参数被拒），设备那边一点动静都没有。 */

      snprintf(output, output_size,
               "没执行：命令没能交给网络层（返回 %d，网络没连上、发送队列满或者"
               "载荷超长），这条命令不会发出去。", ret);
      ai_tools_info("set_light: device_cmd %s=%s 没交给网络层(%d)", device,
                    command, ret);
      return OK;
    }

  /* 走到这里只说明消息**进了发送队列**，真正发出去是 network_task 那条
   * socket 干的；设备收没收到、执行没执行都没有回执。所以不能回"已下发" ——
   * 模型会照着这句话跟老人说，而灯可能压根没动。 */

  snprintf(output, output_size,
           "已交给网络层发送：发给 %s 的灯控命令 %s。发出去而已，设备有没有"
           "执行这边看不到回执，别对老人说已经生效。", device, command);
  ai_tools_info("set_light: device_cmd %s=%s 已交给网络层", device, command);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void ai_tools_provider_set_network_ctx(ai_network_context_t *ctx)
{
  g_net_ctx = ctx;
  ai_tools_info("网络上下文已%s", (ctx != NULL) ? "注册" : "撤销");
}

int ai_tools_provider_init(void)
{
#ifdef CONFIG_HELLO_APP_LLM_AI_AGENT

  if (g_registered)
    {
      /* 幂等：重复注册会在工具清单里塞进两份同名工具。 */

      ai_tools_info("已经注册过，跳过");
      return OK;
    }

  tool_registry_register_provider(AI_TOOLS_PROVIDER_NAME,
                                  ai_tools_provider_get_tools,
                                  ai_tools_provider_execute);

  g_registered = true;

  /* 清单是带缓存的：注册完必须打脏，否则 ai_agent 那边一直用着旧清单，
   * 模型看不到新工具。下一次 ai_agent 收消息时（它每条消息都会重新取一次
   * 清单）set_light 就跟着一起下发了。 */

  tool_registry_invalidate();

  ai_tools_info("已注册 provider「%s」，工具 %s", AI_TOOLS_PROVIDER_NAME,
                AI_TOOL_SET_LIGHT);
  return OK;

#else

  /* 这个配置里 hello_app 没连 ai_agent，工具表都不存在 —— 明确报错，
   * 别让调用方以为注册成功了。 */

  ai_tools_info("本构建没开 CONFIG_HELLO_APP_LLM_AI_AGENT，工具无法注册");
  return -ENOSYS;

#endif
}
