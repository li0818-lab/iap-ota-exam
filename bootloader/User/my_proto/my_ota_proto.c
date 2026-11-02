/**
  ******************************************************************************
  * @file    my_ota_proto.c
  * @brief   OTA 串口自定义通信协议实现（打包 / 解包 / CRC16-Modbus）
  * @note    纯 C 实现，不依赖任何硬件，可同时用于：
  *           - STM32F407 + ST 标准外设库（Keil 工程直接加入本文件）
  *           - PC（gcc 编译，配合 my_ota_proto_test.c 做协议自测）
  *
  *          帧格式与返回值约定详见 my_ota_proto.h 文件头注释。
  ******************************************************************************
  */

#include "my_ota_proto.h"
#include <string.h>     /* memcpy / memset */

/* ==========================================================================
 * 一、内部小工具：16 位大端读写
 *    考题要求"所有 16 位字段统一大端，先发高字节"，
 *    所以这里不要用 *(uint16_t*)p 强转（那样会受 CPU 字节序影响，
 *    而且 Cortex-M4 要求 2 字节对齐，强转还可能触发硬件异常）。
 *    老老实实按字节拼，最稳、也最容易看懂。
 * ========================================================================== */

/**
 * @brief  把 16 位数值按"大端"写进缓冲区（buf[0]=高字节, buf[1]=低字节）
 */
static void Proto_WriteBe16(uint8_t *buf, uint16_t value)
{
    buf[0] = (uint8_t)((value >> 8) & 0xFFu);   /* 高字节先发 */
    buf[1] = (uint8_t)( value       & 0xFFu);   /* 低字节后发 */
}

/**
 * @brief  从缓冲区按"大端"读出一个 16 位数值
 */
static uint16_t Proto_ReadBe16(const uint8_t *buf)
{
    return (uint16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
}

/* ==========================================================================
 * 二、CRC16（Modbus RTU 标准）
 *
 *    参数：多项式 0xA001（0x8005 的反射形式）、初值 0xFFFF、
 *          输入反射、输出反射、结果不异或。
 *
 *    逐位算法（最简单、不用查表，速度对 115200 串口完全够用）：
 *      1. crc = 0xFFFF
 *      2. 每来一个字节：crc ^= byte
 *      3. 循环 8 次：
 *           若 crc 最低位为 1 -> crc = (crc >> 1) ^ 0xA001
 *           否则               -> crc = (crc >> 1)
 *      4. 全部字节处理完，crc 即为结果
 *
 *    自检：算法对 "123456789"（9 个 ASCII 字符）的结果必须是 0x4B37。
 * ========================================================================== */
uint16_t Proto_Crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFFu;     /* 初值 */
    uint16_t i;
    uint8_t  bit;

    if (buf == NULL)
    {
        return 0xFFFFu;         /* 防御：空指针时返回初值，不崩 */
    }

    for (i = 0u; i < len; i++)
    {
        crc ^= (uint16_t)buf[i];            /* 异或当前字节 */

        for (bit = 0u; bit < 8u; bit++)
        {
            if ((crc & 0x0001u) != 0u)
            {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            }
            else
            {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }

    return crc;
}

/* ==========================================================================
 * 三、打包 Pack
 *
 *    整帧布局（以数据长度 N 为例）：
 *      [0..1]   CMD      命令字（大端）
 *      [2..3]   LEN      总长度 = 10 + N（大端）
 *      [4..3+N] DATA     数据区（N 字节）
 *      [4+N..7+N] RSV    保留区 4 字节，填 0
 *      [8+N..9+N] CRC16  CRC（大端，高字节在前）
 *
 *    CRC 计算范围：[0 .. 7+N]（CMD 到保留区末尾），CRC 自身不算。
 * ========================================================================== */
int Proto_Pack(uint16_t cmd,
               const uint8_t *data,
               uint16_t data_len,
               uint8_t *out_buf,
               uint16_t out_cap,
               uint16_t *out_len)
{
    uint16_t total_len;
    uint16_t crc;

    /* ---------- 1. 参数检查 ---------- */
    if ((out_buf == NULL) || (out_len == NULL))
    {
        return PROTO_ERR_NULL;                  /* 4: 空指针 */
    }
    if ((data == NULL) && (data_len > 0u))
    {
        return PROTO_ERR_NULL;                  /* 4: 说有数据却没给地址 */
    }

    /* ---------- 2. 数据区超长检查 ---------- */
    if (data_len > PROTO_DATA_MAX)
    {
        return PROTO_ERR_DATA_LONG;             /* 3: 数据区超长 */
    }

    /* ---------- 3. 计算整帧长度，检查输出缓冲区 ---------- */
    total_len = (uint16_t)(PROTO_FRAME_OVERHEAD + data_len);   /* 10 + N */
    if (out_cap < total_len)
    {
        return PROTO_ERR_BUF;                   /* 5: 缓冲区放不下
                                                 *    （若要求只返回 0~3，
                                                 *      可改成 PROTO_ERR_LEN） */
    }

    /* ---------- 4. 逐字段写入（大端） ---------- */
    Proto_WriteBe16(&out_buf[0], cmd);          /* CMD   */
    Proto_WriteBe16(&out_buf[2], total_len);    /* LEN   */

    if (data_len > 0u)                          /* DATA  */
    {
        (void)memcpy(&out_buf[PROTO_DATA_OFFSET], data, (size_t)data_len);
    }

    /* RSV: 4 字节保留区，按当前约定填 0 */
    (void)memset(&out_buf[PROTO_DATA_OFFSET + data_len], 0, PROTO_RSV_SIZE);

    /* ---------- 5. 计算 CRC 并写入（大端） ----------
     * 计算范围 = 整帧去掉最后 2 字节 CRC：长度 total_len - 2 */
    crc = Proto_Crc16(out_buf, (uint16_t)(total_len - PROTO_CRC_SIZE));
    Proto_WriteBe16(&out_buf[total_len - PROTO_CRC_SIZE], crc);

    /* ---------- 6. 输出实际长度 ---------- */
    *out_len = total_len;

    return PROTO_OK;                            /* 0: 打包成功 */
}

/* ==========================================================================
 * 四、解包 Unpack
 *
 *    检查顺序就是考题的返回值顺序，见 my_ota_proto.h 的注释。
 *    注意：本协议没有"帧头 0xAA55"这种同步字，
 *          所以只能靠 LEN 字段 + CRC 双重校验来判断一帧是否合法。
 * ========================================================================== */
int Proto_Unpack(const uint8_t *in_buf,
                 uint16_t in_len,
                 proto_frame_t *frame,
                 uint16_t *used_len)
{
    uint16_t cmd;
    uint16_t total_len;
    uint16_t data_len;
    uint16_t crc_calc;
    uint16_t crc_recv;

    /* ---------- ① 指针检查 ---------- */
    if ((in_buf == NULL) || (frame == NULL))
    {
        return PROTO_ERR_NULL;                  /* 4: 空指针 */
    }

    /* ---------- ② 现有字节数连最小帧都不够 ---------- */
    if (in_len < PROTO_FRAME_MIN)               /* < 10 */
    {
        return PROTO_ERR_LEN;                   /* 1: 长度非法 */
    }

    /* ---------- ③ 读出 CMD 和 LEN（大端） ---------- */
    cmd       = Proto_ReadBe16(&in_buf[0]);
    total_len = Proto_ReadBe16(&in_buf[2]);

    if (total_len < PROTO_FRAME_MIN)            /* < 10 */
    {
        return PROTO_ERR_LEN;                   /* 1: 长度非法（LEN 字段不可能小于 10） */
    }

    /* ---------- ④ 算出数据区长度，检查是否超上限 ---------- */
    data_len = (uint16_t)(total_len - PROTO_FRAME_OVERHEAD);

    if (data_len > PROTO_DATA_MAX)
    {
        return PROTO_ERR_DATA_LONG;             /* 3: 数据区超长 */
    }

    /* ---------- ⑤ 帧还没收全 ---------- */
    if (total_len > in_len)
    {
        return PROTO_ERR_LEN;                   /* 1: 长度非法（数据不完整，
                                                 *    上层应继续收字节后再试） */
    }

    /* ---------- ⑥ 校验 CRC ----------
     * 计算范围：[0 .. total_len-3]，即 CMD 到保留区末尾 */
    crc_calc = Proto_Crc16(in_buf, (uint16_t)(total_len - PROTO_CRC_SIZE));
    crc_recv = Proto_ReadBe16(&in_buf[total_len - PROTO_CRC_SIZE]);

    if (crc_calc != crc_recv)
    {
        return PROTO_ERR_CRC;                   /* 2: CRC 校验失败 */
    }

    /* ---------- ⑦ 校验通过，填结构体 ---------- */
    frame->cmd       = cmd;
    frame->total_len = total_len;
    frame->data_len  = data_len;
    frame->crc       = crc_recv;

    /* 保留区拷贝出来（同时把旧内容清干净，避免上次残留） */
    (void)memcpy(frame->rsv, &in_buf[PROTO_DATA_OFFSET + data_len], PROTO_RSV_SIZE);

    /* 数据区零拷贝：直接指向输入缓冲区内部 */
    frame->data = &in_buf[PROTO_DATA_OFFSET];

    /* 告诉调用者这一帧占了多少字节 */
    if (used_len != NULL)
    {
        *used_len = total_len;
    }

    return PROTO_OK;                            /* 0: 解析成功 */
}

/* ==========================================================================
 * 五、小工具：判断保留区是否全 0
 * ========================================================================== */
int Proto_IsRsvAllZero(const proto_frame_t *frame)
{
    uint8_t i;

    if (frame == NULL)
    {
        return 0;
    }

    for (i = 0u; i < PROTO_RSV_SIZE; i++)
    {
        if (frame->rsv[i] != 0u)
        {
            return 0;
        }
    }
    return 1;
}

/* ==========================================================================
 * 六、串口"流式接收"该怎么用这个模块（下一阶段写收发时照这个思路做）
 *
 *    串口是一字节一字节来的，可能收到"半帧"，也可能一次收到"两帧"，
 *    还可能因为干扰多出几个乱七八糟的字节。推荐做法：
 *
 *      uint8_t  rx_buf[PROTO_FRAME_MAX];   // 接收缓冲
 *      uint16_t rx_len = 0;                // 当前缓冲里的有效字节数
 *
 *      // 每收到 1 个字节（串口中断里只做存放，不解析）：
 *      rx_buf[rx_len++] = new_byte;
 *
 *      // 在主循环 / 任务里尝试解析：
 *      while (rx_len >= PROTO_FRAME_MIN)
 *      {
 *          int ret = Proto_Unpack(rx_buf, rx_len, &frame, &used);
 *
 *          if (ret == PROTO_OK)                 // 0: 取到一帧
 *          {
 *              HandleFrame(&frame);             //   处理命令
 *              memmove(rx_buf, rx_buf + used, rx_len - used);  // 删掉已用字节
 *              rx_len -= used;
 *          }
 *          else if (ret == PROTO_ERR_LEN)       // 1: 可能没收全
 *          {
 *              break;                           //   继续等后面的字节
 *          }
 *          else                                 // 2/3/4: 这一帧是坏的
 *          {
 *              memmove(rx_buf, rx_buf + 1, rx_len - 1);   // 丢 1 个字节再试
 *              rx_len -= 1;                     // （没有帧头，只能逐字节滑动重试）
 *          }
 *      }
 *
 *    要点：
 *      - 中断里只搬数据，解析放主循环/任务，避免中断里算 CRC 耽误时间；
 *      - 因为协议没有帧头同步字，CRC 失败时只能"丢 1 字节再试"来重新对齐；
 *      - rx_buf 要留够 PROTO_FRAME_MAX 字节，防止半帧越界。
 * ========================================================================== */
