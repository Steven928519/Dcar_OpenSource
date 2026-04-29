/**
 ******************************************************************************
 * @file    usart1_control.c
 * @brief   上位机串口控制实现 (USART1 + DMA+IDLE)
 *
 * 协议摘要:
 *   帧头(0xDF)
 *   目标地址 (U8)
 *   本机地址 (U8)
 *   类型 A (U8)
 *   类型 B (U8)
 *   数据长度 (U8) = 0x06
 *   数据区 6 字节:
 *     [0..1] S16 小端: X 方向速度 (右为正, mm/s)
 *     [2..3] S16 小端: Y 方向速度 (前为正, mm/s)
 *     [4..5] S16 小端: 旋转速度 (顺时针为正, mm/s)
 *   帧尾(0xFD)
 *   校验 (U8): 从帧头到帧尾(含)累加和取低 8 位
 *
 * 本模块职责:
 *   - 通过 HAL_UARTEx_ReceiveToIdle_DMA 启动 USART1 的 DMA+IDLE 接收
 *   - 在 HAL_UARTEx_RxEventCallback 中按协议解析数据
 *   - 将解析出的 vx/vy/w (mm/s) 填入 Uart1_ControlCmd_t 供上层查询
 *   - 收到合法指令时回传 0x43 作为应答
 *
 * 匀速位移 (0x64):
 *   数据区 14 字节:
 *     [0..3]   S32 小端: X 位移 (厘米 × 100)
 *     [4..7]   S32 小端: Y 位移 (厘米 × 100)
 *     [8..11]  S32 小端: Z 位移/航向 (厘米 × 100)
 *     [12..13] U16 小端: 最大合速度 (厘米/秒 × 100)
 *   单位转换: raw_value / 10.0 → mm 或 mm/s
 *   当前只处理 0x64 匀速平移
 *
 * 测试指令 (波特率 115200, 十六进制发送):
 *
 *   [0x67] 前进 300 mm/s:   DF 01 97 02 67 06 00 00 2C 01 00 00 FD 10
 *   [0x67] 后退 300 mm/s:   DF 01 97 02 67 06 00 00 D4 FE 00 00 FD B5
 *   [0x67] 右直行 300 mm/s:  DF 01 97 02 67 06 2C 01 00 00 00 00 FD 10
 *   [0x67] 左直行 300 mm/s:  DF 01 97 02 67 06 D4 FE 00 00 00 00 FD B5
 *   [0x67] 顺时针旋转 300:   DF 01 97 02 67 06 00 00 00 00 2C 01 FD 10
 *   [0x67] 停止:            DF 01 97 02 67 06 00 00 00 00 00 00 FD E3
 *
 *   [0x64] 匀速位移 X+20cm (200mm), 速度 50cm/s:
 *     DF 01 97 02 64 0E D0 07 00 00 00 00 00 00 00 00 00 00 88 13 FD ??
 *
 *   解析成功时单片机回传 0x43
 ******************************************************************************
 */

#include "usart1_control.h"
#include "usart.h"
#include <stdint.h>
#include <string.h>

/* USART1 DMA 接收缓冲区配置: 存放 HAL DMA+IDLE 收到的原始字节流 */
#define UART1_RX_BUF_SIZE 128
#define UART1_STREAM_BUF_SIZE 256
#define UART1_FRAME_WAIT_TIMEOUT_MS 100U

/* 协议常量 (当前实现: 匀速速度控制 0x67, 匀速位移 0x64) */
#define UART1_FRAME_HEAD 0xDF
#define UART1_FRAME_TAIL 0xFD

#define UART1_ADDR_MCU 0x01
#define UART1_ADDR_PC 0x97

#define UART1_TYPE_A_MOVE 0x02
#define UART1_TYPE_B_VELOCITY 0x67     /* 持续速度控制 */
#define UART1_TYPE_B_DISPLACEMENT 0x64 /* 定点匀速位移 */

#define UART1_DATA_LEN_VELOCITY 0x06
#define UART1_DATA_LEN_DISPLACEMENT 14

#define UART1_FRAME_LEN_VELOCITY                                               \
  (1U /* head */ + 1U /* dst */ + 1U /* src */ + 2U /* typeA/typeB */ +        \
   1U /* len */ + UART1_DATA_LEN_VELOCITY /* payload */ + 1U /* tail */ +      \
   1U /* checksum */)

#define UART1_FRAME_LEN_DISPLACEMENT                                           \
  (1U /* head */ + 1U /* dst */ + 1U /* src */ + 2U /* typeA/typeB */ +        \
   1U /* len */ + UART1_DATA_LEN_DISPLACEMENT /* payload */ + 1U /* tail */ +  \
   1U /* checksum */)

/* 速度缩放: 协议中 S16 一位对应多少 mm/s */
#define UART1_SPEED_SCALE_MMPS_PER_LSB 1.0f

/* 位移帧单位转换: 协议值 = 厘米 × 100, 目标单位 mm => raw / 100.0 × 10 = raw / 10.0 */
#define UART1_DISP_CM_TO_MM_FACTOR 10.0f

/* DMA 接收缓冲区 (只在本模块内部可见) */
static uint8_t s_uart1_rx_buf[UART1_RX_BUF_SIZE];

/* Keep bytes across DMA+IDLE callbacks so a wireless UART pause inside one
 * frame does not make the parser throw away the first half.
 */
static uint8_t s_uart1_stream_buf[UART1_STREAM_BUF_SIZE];
static uint16_t s_uart1_stream_len = 0U;
static uint32_t s_uart1_stream_last_tick = 0U;
static uint16_t s_uart1_last_rx_size = 0U;

typedef struct {
  uint32_t seq;
  uint16_t rx_size;
  uint16_t stream_len;
  uint16_t frame_len;
  uint8_t code;
  uint8_t hdr[6];
  uint8_t tail;
  uint8_t checksum_calc;
  uint8_t checksum_rx;
  int32_t x_raw;
  int32_t y_raw;
  int32_t z_raw;
  uint16_t speed_raw;
} Uart1_DebugInfo_t;

#define UART1_DBG64_SEEN 1U
#define UART1_DBG64_WAIT_MORE 2U
#define UART1_DBG64_BAD_TAIL 3U
#define UART1_DBG64_BAD_SUM 4U
#define UART1_DBG64_PARSE_OK 5U
#define UART1_DBG64_PARSE_FAIL 6U

static volatile Uart1_DebugInfo_t s_uart1_debug;
static volatile uint8_t s_uart1_debug_pending = 0U;

/* 最近一次解析成功的控制命令
 * 标记为 volatile 是为了防止编译器在中断与主循环之间优化掉读写
 */
static volatile Uart1_ControlCmd_t s_latest_cmd;

/* 将两个小端无符号字节组合成有符号 16 位整数 */
static int16_t BytesToS16LE(uint8_t low, uint8_t high) {
  return (int16_t)((uint16_t)low | ((uint16_t)high << 8));
}

/* 将两个小端无符号字节组合成无符号 16 位整数 */
static uint16_t BytesToU16LE(uint8_t low, uint8_t high) {
  return (uint16_t)((uint16_t)low | ((uint16_t)high << 8));
}

/* 将四个小端字节组合成有符号 32 位整数 */
static int32_t BytesToS32LE(const uint8_t *p) {
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                   ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

/* 尝试解析速度控制帧 (0x67) */
static uint8_t Uart1_ChecksumCalc(const uint8_t *frame, uint16_t frame_len) {
  uint8_t sum = 0U;

  if (frame == NULL || frame_len < 2U) {
    return 0U;
  }

  for (uint16_t i = 0U; i < (uint16_t)(frame_len - 1U); i++) {
    sum = (uint8_t)(sum + frame[i]);
  }

  return sum;
}

static void Uart1_Debug64Set(uint8_t code, uint16_t frame_len,
                             uint8_t checksum_calc) {
  s_uart1_debug.seq++;
  s_uart1_debug.code = code;
  s_uart1_debug.rx_size = s_uart1_last_rx_size;
  s_uart1_debug.stream_len = s_uart1_stream_len;
  s_uart1_debug.frame_len = frame_len;
  s_uart1_debug.checksum_calc = checksum_calc;

  for (uint8_t i = 0U; i < 6U; i++) {
    s_uart1_debug.hdr[i] =
        (i < s_uart1_stream_len) ? s_uart1_stream_buf[i] : 0U;
  }

  if (frame_len > 1U && s_uart1_stream_len >= frame_len) {
    s_uart1_debug.tail = s_uart1_stream_buf[frame_len - 2U];
    s_uart1_debug.checksum_rx = s_uart1_stream_buf[frame_len - 1U];
  } else {
    s_uart1_debug.tail = 0U;
    s_uart1_debug.checksum_rx = 0U;
  }

  if (s_uart1_stream_len >= (uint16_t)UART1_FRAME_LEN_DISPLACEMENT) {
    s_uart1_debug.x_raw = BytesToS32LE(&s_uart1_stream_buf[6]);
    s_uart1_debug.y_raw = BytesToS32LE(&s_uart1_stream_buf[10]);
    s_uart1_debug.z_raw = BytesToS32LE(&s_uart1_stream_buf[14]);
    s_uart1_debug.speed_raw =
        BytesToU16LE(s_uart1_stream_buf[18], s_uart1_stream_buf[19]);
  } else {
    s_uart1_debug.x_raw = 0;
    s_uart1_debug.y_raw = 0;
    s_uart1_debug.z_raw = 0;
    s_uart1_debug.speed_raw = 0U;
  }

  s_uart1_debug_pending = 1U;
}

static uint8_t ParseVelocityFrame(const uint8_t *buf, uint16_t size) {
  if (size != (uint16_t)UART1_FRAME_LEN_VELOCITY)
    return 0U;
  if (buf[0] != UART1_FRAME_HEAD || buf[12] != UART1_FRAME_TAIL)
    return 0U;
  if (buf[1] != UART1_ADDR_MCU || buf[2] != UART1_ADDR_PC)
    return 0U;
  if (buf[3] != UART1_TYPE_A_MOVE || buf[4] != UART1_TYPE_B_VELOCITY)
    return 0U;
  if (buf[5] != UART1_DATA_LEN_VELOCITY)
    return 0U;

  uint8_t sum = 0U;
  for (uint8_t i = 0; i <= 12; i++)
    sum = (uint8_t)(sum + buf[i]);
  if (sum != buf[13])
    return 0U;

  Uart1_ControlCmd_t cmd = {0};
  cmd.type = UART1_CMD_VELOCITY;
  cmd.vx_mmps =
      (float)BytesToS16LE(buf[6], buf[7]) * UART1_SPEED_SCALE_MMPS_PER_LSB;
  cmd.vy_mmps =
      (float)BytesToS16LE(buf[8], buf[9]) * UART1_SPEED_SCALE_MMPS_PER_LSB;
  cmd.w_mmps =
      (float)BytesToS16LE(buf[10], buf[11]) * UART1_SPEED_SCALE_MMPS_PER_LSB;
  cmd.valid = 1U;

  s_latest_cmd = cmd;
  return 1U;
}

/* 尝试解析位移控制帧 (0x64) */
static uint8_t ParseDisplacementFrame(const uint8_t *buf, uint16_t size) {
  if (size != (uint16_t)UART1_FRAME_LEN_DISPLACEMENT)
    return 0U;
  if (buf[0] != UART1_FRAME_HEAD || buf[20] != UART1_FRAME_TAIL)
    return 0U;
  if (buf[1] != UART1_ADDR_MCU || buf[2] != UART1_ADDR_PC)
    return 0U;
  if (buf[3] != UART1_TYPE_A_MOVE || buf[4] != UART1_TYPE_B_DISPLACEMENT)
    return 0U;
  if (buf[5] != UART1_DATA_LEN_DISPLACEMENT)
    return 0U;

  uint8_t sum = 0U;
  for (uint8_t i = 0; i <= 20; i++)
    sum = (uint8_t)(sum + buf[i]);
  if (sum != buf[21])
    return 0U;

  Uart1_ControlCmd_t cmd = {0};
  cmd.type = UART1_CMD_DISPLACEMENT;
  /*
   * 协议: F32 表示 厘米 × 100 (4字节小端整数), 目标单位 mm
   * 转换: (raw / 100.0) cm × 10 = raw / 10.0 mm
   * 例如: 0.2m = 20cm -> 协议值 2000 -> 2000/10 = 200mm ✓
   */
  cmd.target_x_mm = (float)BytesToS32LE(&buf[6]) / UART1_DISP_CM_TO_MM_FACTOR;
  cmd.target_y_mm = (float)BytesToS32LE(&buf[10]) / UART1_DISP_CM_TO_MM_FACTOR;
  cmd.target_z_mm = (float)BytesToS32LE(&buf[14]) / UART1_DISP_CM_TO_MM_FACTOR;

  /* F16 速度: 厘米/秒 × 100, 转 mm/s => raw / 10.0 */
  cmd.target_speed_mmps = (float)BytesToU16LE(buf[18], buf[19]) / UART1_DISP_CM_TO_MM_FACTOR;
  cmd.valid = 1U;

  s_latest_cmd = cmd;
  return 1U;
}

static uint8_t ParseRawDisplacementBlock(const uint8_t *data, uint16_t size) {
  if (data == NULL || size < (uint16_t)UART1_FRAME_LEN_DISPLACEMENT) {
    return 0U;
  }

  for (uint16_t i = 0U;
       i <= (uint16_t)(size - (uint16_t)UART1_FRAME_LEN_DISPLACEMENT); i++) {
    if (data[i] == UART1_FRAME_HEAD &&
        data[i + 4U] == UART1_TYPE_B_DISPLACEMENT &&
        data[i + 5U] == UART1_DATA_LEN_DISPLACEMENT &&
        ParseDisplacementFrame(&data[i],
                               (uint16_t)UART1_FRAME_LEN_DISPLACEMENT)) {
      return 1U;
    }
  }

  return 0U;
}

static void Uart1_StreamDrop(uint16_t count) {
  if (count >= s_uart1_stream_len) {
    s_uart1_stream_len = 0U;
    return;
  }

  memmove(s_uart1_stream_buf, &s_uart1_stream_buf[count],
          (size_t)(s_uart1_stream_len - count));
  s_uart1_stream_len = (uint16_t)(s_uart1_stream_len - count);
}

static void Uart1_StreamResetIfTimeout(uint32_t now_tick) {
  if (s_uart1_stream_len > 0U &&
      (uint32_t)(now_tick - s_uart1_stream_last_tick) >
          UART1_FRAME_WAIT_TIMEOUT_MS) {
    s_uart1_stream_len = 0U;
  }
}

static void Uart1_StreamPush(const uint8_t *data, uint16_t size) {
  if (data == NULL || size == 0U) {
    return;
  }

  if (size >= UART1_STREAM_BUF_SIZE) {
    data = &data[size - UART1_STREAM_BUF_SIZE];
    size = UART1_STREAM_BUF_SIZE;
    s_uart1_stream_len = 0U;
  } else if ((uint16_t)(s_uart1_stream_len + size) > UART1_STREAM_BUF_SIZE) {
    Uart1_StreamDrop((uint16_t)(s_uart1_stream_len + size -
                                UART1_STREAM_BUF_SIZE));
  }

  memcpy(&s_uart1_stream_buf[s_uart1_stream_len], data, size);
  s_uart1_stream_len = (uint16_t)(s_uart1_stream_len + size);
}

static uint8_t Uart1_StreamParse(void) {
  uint8_t ack_success = 0U;

  while (s_uart1_stream_len > 0U) {
    uint16_t head_index = 0U;
    while (head_index < s_uart1_stream_len &&
           s_uart1_stream_buf[head_index] != UART1_FRAME_HEAD) {
      head_index++;
    }

    if (head_index > 0U) {
      Uart1_StreamDrop(head_index);
    }

    if (s_uart1_stream_len < 6U) {
      break;
    }

    if (s_uart1_stream_buf[1] != UART1_ADDR_MCU ||
        s_uart1_stream_buf[2] != UART1_ADDR_PC ||
        s_uart1_stream_buf[3] != UART1_TYPE_A_MOVE) {
      Uart1_StreamDrop(1U);
      continue;
    }

    uint16_t frame_len = 0U;

    if (s_uart1_stream_buf[4] == UART1_TYPE_B_VELOCITY &&
        s_uart1_stream_buf[5] == UART1_DATA_LEN_VELOCITY) {
      frame_len = (uint16_t)UART1_FRAME_LEN_VELOCITY;
    } else if (s_uart1_stream_buf[4] == UART1_TYPE_B_DISPLACEMENT &&
               s_uart1_stream_buf[5] == UART1_DATA_LEN_DISPLACEMENT) {
      frame_len = (uint16_t)UART1_FRAME_LEN_DISPLACEMENT;
      Uart1_Debug64Set(UART1_DBG64_SEEN, frame_len, 0U);
    } else {
      Uart1_StreamDrop(1U);
      continue;
    }

    if (s_uart1_stream_len < frame_len) {
      if (frame_len == (uint16_t)UART1_FRAME_LEN_DISPLACEMENT) {
        Uart1_Debug64Set(UART1_DBG64_WAIT_MORE, frame_len, 0U);
      }
      break;
    }

    uint8_t checksum_calc = Uart1_ChecksumCalc(s_uart1_stream_buf, frame_len);
    if (s_uart1_stream_buf[frame_len - 2U] != UART1_FRAME_TAIL) {
      if (frame_len == (uint16_t)UART1_FRAME_LEN_DISPLACEMENT) {
        Uart1_Debug64Set(UART1_DBG64_BAD_TAIL, frame_len, checksum_calc);
      }
      Uart1_StreamDrop(1U);
      continue;
    }

    if (checksum_calc != s_uart1_stream_buf[frame_len - 1U]) {
      if (frame_len == (uint16_t)UART1_FRAME_LEN_DISPLACEMENT) {
        Uart1_Debug64Set(UART1_DBG64_BAD_SUM, frame_len, checksum_calc);
      }
      Uart1_StreamDrop(1U);
      continue;
    }

    if (frame_len == (uint16_t)UART1_FRAME_LEN_VELOCITY) {
      if (ParseVelocityFrame(s_uart1_stream_buf, frame_len)) {
        ack_success = 1U;
      }
    } else {
      if (ParseDisplacementFrame(s_uart1_stream_buf, frame_len)) {
        Uart1_Debug64Set(UART1_DBG64_PARSE_OK, frame_len, checksum_calc);
        ack_success = 1U;
      } else {
        Uart1_Debug64Set(UART1_DBG64_PARSE_FAIL, frame_len, checksum_calc);
      }
    }

    Uart1_StreamDrop(frame_len);
  }

  return ack_success;
}

void Uart1_Control_Init(void) {
  /* 初始化最近命令为无效状态 */
  s_latest_cmd.type = UART1_CMD_NONE;
  s_latest_cmd.vx_mmps = 0.0f;
  s_latest_cmd.vy_mmps = 0.0f;
  s_latest_cmd.w_mmps = 0.0f;
  s_latest_cmd.valid = 0U;
  s_uart1_stream_len = 0U;
  s_uart1_stream_last_tick = HAL_GetTick();
  s_uart1_last_rx_size = 0U;
  s_uart1_debug_pending = 0U;

  /* 启动 USART1 的 DMA+IDLE 接收
   * - huart1: 来自 usart.c 的 UART1 句柄
   * - s_uart1_rx_buf: DMA 接收缓冲区
   * - UART1_RX_BUF_SIZE: 最大接收长度
   */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_uart1_rx_buf, UART1_RX_BUF_SIZE);
  /* 关闭半传输中断，避免产生过多中断回调 */
  if (huart1.hdmarx != NULL) {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
}

void Uart1_Control_GetLatestCmd(Uart1_ControlCmd_t *out) {
  if (out == NULL) {
    return;
  }
  /* 简单的"临界区": 复制结构体时短暂关中断, 防止中断同时修改 s_latest_cmd */
  __disable_irq();
  *out = s_latest_cmd;
  __enable_irq();
}

void Uart1_Control_ClearCmd(void) {
  __disable_irq();
  s_latest_cmd.valid = 0U;
  s_latest_cmd.type = UART1_CMD_NONE;
  __enable_irq();
}

/* 应答标志：中断回调设置，主循环读取并发送 */
static volatile uint8_t s_ack_pending = 0;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart == NULL) {
    return;
  }
  if (huart->Instance != USART1) {
    return;
  }

  /* 尝试在本次数据块中寻找并解析控制帧
   * 允许前面有噪声或一次收到多帧：匹配到的帧都会更新 s_latest_cmd，最后一帧生效
   * 解析成功则设置应答标志，由主循环发送
   */
  uint32_t now_tick = HAL_GetTick();
  s_uart1_last_rx_size = Size;
  Uart1_StreamResetIfTimeout(now_tick);
  Uart1_StreamPush(s_uart1_rx_buf, Size);
  s_uart1_stream_last_tick = now_tick;
  uint8_t ack_success = ParseRawDisplacementBlock(s_uart1_rx_buf, Size);
  if (Uart1_StreamParse()) {
    ack_success = 1U;
  }
  
  /* 尝试在本次数据块中寻找并解析控制帧 */
  /* 寻找位移帧 (0x64) */
  /* 设置应答标志（主循环负责发送） */
  if (ack_success) {
    s_ack_pending = 1;
  }

  /* 重新启动 DMA+IDLE 接收 */
  /* 先停止 DMA，再重新启动，确保状态机正常 */
  HAL_UART_AbortReceive(&huart1);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_uart1_rx_buf, UART1_RX_BUF_SIZE);
  if (huart1.hdmarx != NULL) {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
}

/**
 * @brief  在中断外发送应答（非阻塞）
 * @note   应在主循环中调用
 */
void Uart1_Control_SendAck(void) {
  if (s_ack_pending) {
    const uint8_t ack_buf[] = {0x43U, '\r', '\n'};
    HAL_UART_Transmit(&huart1, ack_buf, sizeof(ack_buf), 10);
    s_ack_pending = 0;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart == NULL || huart->Instance != USART1) {
    return;
  }

  HAL_UART_AbortReceive(&huart1);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_uart1_rx_buf, UART1_RX_BUF_SIZE);
  if (huart1.hdmarx != NULL) {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
}
