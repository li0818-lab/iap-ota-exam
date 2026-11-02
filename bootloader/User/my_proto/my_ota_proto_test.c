/**
  ******************************************************************************
  * @file    my_ota_proto_test.c
  * @brief   my_ota_proto 协议模块自测程序（学习用）
  * @note    同一条测试代码可以在两个地方跑：
  *
  *          【PC 上跑（推荐先这样学）】
  *              cd D:\all\shared
  *              gcc my_ota_proto.c my_ota_proto_test.c -DPROTO_TEST_PC -o proto_test.exe
  *              .\proto_test.exe
  *
  *          【STM32F407 板子上跑】
  *              1) 把 my_ota_proto.c / my_ota_proto_test.c / my_ota_proto.h
  *                 三个文件用 Keil "Add Existing Files" 加进工程；
  *              2) 在工程 main.c 里，串口初始化(printf 重定向)之后调用：
  *                     Proto_SelfTest();
  *              3) 打开串口助手 115200，复位板子，就能看到同一份测试报告。
  *                 （注意：不要定义 PROTO_TEST_PC 宏，否则会出现第二个 main）
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>


/* ======================= 测试小工具 ======================= */

static int s_pass = 0;   /* 通过计数 */
static int s_fail = 0;   /* 失败计数 */

/* 打印一段十六进制，例如： 00 01 00 0A 00 00 00 00 78 F1 */
static void DumpHex(const char *tag, const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    printf("%-30s (%u 字节):", tag, (unsigned)len);
    for (i = 0u; i < len; i++)
    {
        printf(" %02X", buf[i]);
    }
    printf("\r\n");
}

/* 通用判断：条件成立打印 PASS，否则 FAIL */
static void Check(const char *what, int cond)
{
    if (cond != 0)
    {
        printf("    [PASS] %s\r\n", what);
        s_pass++;
    }
    else
    {
        printf("    [FAIL] %s\r\n", what);
        s_fail++;
    }
}

/* 判断函数返回值：把"期望值 / 实际值"都打出来，方便学习 */
static void CheckRet(const char *what, int expect, int actual)
{
    if (expect == actual)
    {
        printf("    [PASS] %-40s 期望=%d 实际=%d\r\n", what, expect, actual);
        s_pass++;
    }
    else
    {
        printf("    [FAIL] %-40s 期望=%d 实际=%d\r\n", what, expect, actual);
        s_fail++;
    }
}

/* ======================= 自测主体 ======================= */

void Proto_SelfTest(void)
{
    /* 两个测试用例（用 static 放静态区，避免在 STM32 上占用过多栈空间） */
    static uint8_t  frame_empty[PROTO_FRAME_MAX];   /* 空数据帧 */
    uint16_t empty_len = 0u;
    static uint8_t  frame_normal[PROTO_FRAME_MAX];  /* 普通数据帧 */
    uint16_t normal_len = 0u;
    uint8_t  normal_data[4] = { 0x11u, 0x22u, 0x33u, 0x44u };

    uint8_t  stream[64];                         /* 两帧粘连的缓冲区 */
    uint16_t stream_len;
    uint16_t used = 0u;                          /* 解包消耗的字节数 */

    proto_frame_t frame;
    int ret;

    s_pass = 0;
    s_fail = 0;

    printf("\r\n================ my_ota_proto 协议模块自测开始 ================\r\n");

    /* ------------------------------------------------------------------
     * 1. CRC16 算法自检
     *    Modbus 标准校验值：ASCII "123456789" 的 CRC16 = 0x4B37
     *    这个值全世界通用，对不上就说明 CRC 算法写错了，后面全白搭。
     * ------------------------------------------------------------------ */
    printf("\r\n[1] CRC16/Modbus 算法自检\r\n");
    {
        const uint8_t  vec[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
        uint16_t crc = Proto_Crc16(vec, (uint16_t)sizeof(vec));

        printf("    CRC16(\"123456789\") = 0x%04X\r\n", (unsigned)crc);
        Check("CRC16 标准校验值应为 0x4B37", crc == 0x4B37u);
    }

    /* ------------------------------------------------------------------
     * 2. 打包：空数据帧（N = 0）
     *    帧长 = 10 + 0 = 10 字节
     * ------------------------------------------------------------------ */
    printf("\r\n[2] 打包：空数据帧 CMD=0x0001, N=0\r\n");
    ret = Proto_Pack(0x0001u, NULL, 0u, frame_empty, sizeof(frame_empty), &empty_len);
    CheckRet("Proto_Pack 返回值", PROTO_OK, ret);
    Check("整帧长度应为 10", empty_len == PROTO_FRAME_MIN);
    DumpHex("空数据帧 HEX", frame_empty, empty_len);
    Check("LEN 字段字节[2..3] 应为 00 0A",
          (frame_empty[2] == 0x00u) && (frame_empty[3] == 0x0Au));
    {
        /* 自己再算一遍 CRC，验证帧内 CRC 字段确实是对这 8 个字节算出来的 */
        uint16_t crc_calc  = Proto_Crc16(frame_empty, 8u);
        uint16_t crc_infrm = (uint16_t)(((uint16_t)frame_empty[8] << 8) | frame_empty[9]);
        printf("    自行验算 CRC = 0x%04X, 帧内 CRC = 0x%04X\r\n",
               (unsigned)crc_calc, (unsigned)crc_infrm);
        Check("帧内 CRC 与验算结果一致", crc_calc == crc_infrm);
    }
    {
        /* 回归测试：打包结果必须与下面这串"标准答案"逐字节一致
         * （这串字节就是本协议的"空数据帧"样例，写文档、和上位机联调都用它） */
        static const uint8_t expect_empty[10] =
        {
            0x00u, 0x01u,              /* CMD  = 0x0001      */
            0x00u, 0x0Au,              /* LEN  = 0x000A = 10 */
            0x00u, 0x00u, 0x00u, 0x00u,/* RSV  = 4 字节 0    */
            0xCAu, 0xC8u               /* CRC  = 0xCAC8（高字节先） */
        };
        Check("空数据帧与标准样例逐字节一致",
              memcmp(frame_empty, expect_empty, 10u) == 0);
    }

    /* ------------------------------------------------------------------
     * 3. 打包：普通数据帧（N = 4）
     *    CMD = 0x0002, DATA = 11 22 33 44
     *    帧长 = 10 + 4 = 14 字节
     * ------------------------------------------------------------------ */
    printf("\r\n[3] 打包：普通数据帧 CMD=0x0002, DATA=11 22 33 44\r\n");
    ret = Proto_Pack(0x0002u, normal_data, (uint16_t)sizeof(normal_data),
                     frame_normal, sizeof(frame_normal), &normal_len);
    CheckRet("Proto_Pack 返回值", PROTO_OK, ret);
    Check("整帧长度应为 14", normal_len == 14u);
    DumpHex("普通数据帧 HEX", frame_normal, normal_len);
    Check("LEN 字段字节[2..3] 应为 00 0E",
          (frame_normal[2] == 0x00u) && (frame_normal[3] == 0x0Eu));
    Check("DATA 区应为 11 22 33 44",
          (memcmp(&frame_normal[PROTO_DATA_OFFSET], normal_data, 4u) == 0) &&
          (frame_normal[8] == 0x00u) && (frame_normal[9] == 0x00u) &&
          (frame_normal[10] == 0x00u) && (frame_normal[11] == 0x00u));
    {
        /* 回归测试：普通数据帧的"标准答案"（14 字节） */
        static const uint8_t expect_normal[14] =
        {
            0x00u, 0x02u,              /* CMD  = 0x0002        */
            0x00u, 0x0Eu,              /* LEN  = 0x000E = 14   */
            0x11u, 0x22u, 0x33u, 0x44u,/* DATA = 4 字节         */
            0x00u, 0x00u, 0x00u, 0x00u,/* RSV  = 4 字节 0       */
            0xD4u, 0x14u               /* CRC  = 0xD414（高字节先） */
        };
        Check("普通数据帧与标准样例逐字节一致",
              memcmp(frame_normal, expect_normal, 14u) == 0);
    }

    /* ------------------------------------------------------------------
     * 4. 解包：空数据帧
     * ------------------------------------------------------------------ */
    printf("\r\n[4] 解包：空数据帧\r\n");
    (void)memset(&frame, 0, sizeof(frame));
    used = 0u;
    ret = Proto_Unpack(frame_empty, empty_len, &frame, &used);
    CheckRet("Proto_Unpack 返回值", PROTO_OK, ret);
    Check("CMD = 0x0001", frame.cmd == 0x0001u);
    Check("总长度 = 10", frame.total_len == 10u);
    Check("数据区长度 = 0", frame.data_len == 0u);
    Check("消耗字节数 = 10", used == 10u);
    Check("保留区全 0", Proto_IsRsvAllZero(&frame) == 1);
    printf("    解析结果: CMD=0x%04X LEN=%u N=%u CRC=0x%04X\r\n",
           (unsigned)frame.cmd, (unsigned)frame.total_len,
           (unsigned)frame.data_len, (unsigned)frame.crc);

    /* ------------------------------------------------------------------
     * 5. 解包：普通数据帧
     * ------------------------------------------------------------------ */
    printf("\r\n[5] 解包：普通数据帧\r\n");
    (void)memset(&frame, 0, sizeof(frame));
    ret = Proto_Unpack(frame_normal, normal_len, &frame, &used);
    CheckRet("Proto_Unpack 返回值", PROTO_OK, ret);
    Check("CMD = 0x0002", frame.cmd == 0x0002u);
    Check("总长度 = 14", frame.total_len == 14u);
    Check("数据区长度 = 4", frame.data_len == 4u);
    Check("数据区内容 = 11 22 33 44", memcmp(frame.data, normal_data, 4u) == 0);
    printf("    解析结果: CMD=0x%04X LEN=%u N=%u 数据=",
           (unsigned)frame.cmd, (unsigned)frame.total_len, (unsigned)frame.data_len);
    {
        uint16_t i;
        for (i = 0u; i < frame.data_len; i++)
        {
            printf("%02X ", frame.data[i]);
        }
        printf("\r\n");
    }

    /* ------------------------------------------------------------------
     * 6. 流式场景：两帧粘在一起，循环解包
     *    模拟串口一次收到了 10 + 14 = 24 个字节
     * ------------------------------------------------------------------ */
    printf("\r\n[6] 两帧粘连（模拟串口一次收到两帧）\r\n");
    (void)memcpy(&stream[0], frame_empty, empty_len);
    (void)memcpy(&stream[empty_len], frame_normal, normal_len);
    stream_len = (uint16_t)(empty_len + normal_len);
    DumpHex("stream 缓冲区", stream, stream_len);
    {
        uint16_t offset = 0u;
        int      cnt    = 0;

        while (offset < stream_len)
        {
            ret = Proto_Unpack(&stream[offset], (uint16_t)(stream_len - offset),
                               &frame, &used);
            if (ret != PROTO_OK)
            {
                break;
            }
            cnt++;
            printf("    第 %d 帧: CMD=0x%04X N=%u, 占用 %u 字节\r\n",
                   cnt, (unsigned)frame.cmd, (unsigned)frame.data_len, (unsigned)used);
            offset = (uint16_t)(offset + used);
        }
        CheckRet("应连续解析出 2 帧", 2, cnt);
        Check("两帧总消耗字节 = 24", offset == 24u);
    }

    /* ------------------------------------------------------------------
     * 7. 错误用例：长度非法（返回 1）
     * ------------------------------------------------------------------ */
    printf("\r\n[7] 错误用例：长度非法 (应返回 1)\r\n");
    {
        /* 7.1 缓冲区只有 5 个字节，连最小帧都不够 */
        ret = Proto_Unpack(frame_normal, 5u, &frame, &used);
        CheckRet("只有 5 字节 -> 1", PROTO_ERR_LEN, ret);
    }
    {
        /* 7.2 LEN 字段 = 0x0006（6 < 10，不可能合法） */
        uint8_t bad[10] = { 0x00u, 0x01u, 0x00u, 0x06u,
                            0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u };
        ret = Proto_Unpack(bad, (uint16_t)sizeof(bad), &frame, &used);
        CheckRet("LEN 字段=6 -> 1", PROTO_ERR_LEN, ret);
    }
    {
        /* 7.3 帧声称 0x0020 = 32 字节，但实际只给了 14 字节（没收全） */
        uint8_t bad[14];
        (void)memcpy(bad, frame_normal, 14u);
        bad[2] = 0x00u;
        bad[3] = 0x20u;
        ret = Proto_Unpack(bad, (uint16_t)sizeof(bad), &frame, &used);
        CheckRet("声明 32 字节只收 14 -> 1", PROTO_ERR_LEN, ret);
    }

    /* ------------------------------------------------------------------
     * 8. 错误用例：数据区超长（返回 3）
     *    LEN = 10 + 1025 = 1035 = 0x040B
     * ------------------------------------------------------------------ */
    printf("\r\n[8] 错误用例：数据区超长 (应返回 3)\r\n");
    {
        uint8_t bad[10] = { 0x00u, 0x01u, 0x04u, 0x0Bu,
                            0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u };
        ret = Proto_Unpack(bad, (uint16_t)sizeof(bad), &frame, &used);
        CheckRet("N=1025 > 1024 -> 3", PROTO_ERR_DATA_LONG, ret);
    }

    /* ------------------------------------------------------------------
     * 9. 错误用例：CRC 校验失败（返回 2）
     * ------------------------------------------------------------------ */
    printf("\r\n[9] 错误用例：CRC 校验失败 (应返回 2)\r\n");
    {
        uint8_t bad[14];
        (void)memcpy(bad, frame_normal, 14u);
        bad[4] ^= 0xFFu;              /* 故意把数据区第 1 个字节改掉 */
        DumpHex("被改坏的帧", bad, 14u);
        ret = Proto_Unpack(bad, (uint16_t)sizeof(bad), &frame, &used);
        CheckRet("数据被篡改 -> 2", PROTO_ERR_CRC, ret);
    }

    /* ------------------------------------------------------------------
     * 10. 打包侧的错误：缓冲区不足 / 数据区超长
     * ------------------------------------------------------------------ */
    printf("\r\n[10] 打包错误用例\r\n");
    {
        uint8_t small[8];
        uint16_t out_len = 0u;
        ret = Proto_Pack(0x0002u, normal_data, 4u, small, (uint16_t)sizeof(small), &out_len);
        CheckRet("输出缓冲区只有 8 字节 -> 5", PROTO_ERR_BUF, ret);
    }
    {
        static uint8_t big[PROTO_DATA_MAX + 1u];
        uint16_t out_len = 0u;
        ret = Proto_Pack(0x0003u, big, (uint16_t)sizeof(big),
                         frame_empty, sizeof(frame_empty), &out_len);
        CheckRet("数据区 1025 字节 -> 3", PROTO_ERR_DATA_LONG, ret);
    }

    /* ------------------------------------------------------------------
     * 11. 边界用例：最大数据区 1024 字节 打包 + 回环解包
     * ------------------------------------------------------------------ */
    printf("\r\n[11] 边界用例：数据区 1024 字节（最大）\r\n");
    {
        static uint8_t big[PROTO_DATA_MAX];
        static uint8_t big_frame[PROTO_FRAME_MAX];
        uint16_t out_len = 0u;
        uint16_t i;

        for (i = 0u; i < PROTO_DATA_MAX; i++)
        {
            big[i] = (uint8_t)(i & 0xFFu);          /* 0,1,2,...,255,0,1,... */
        }

        ret = Proto_Pack(0x0101u, big, PROTO_DATA_MAX, big_frame,
                         sizeof(big_frame), &out_len);
        CheckRet("打包 1024 字节数据", PROTO_OK, ret);
        Check("整帧长度 = 1034", out_len == (uint16_t)PROTO_FRAME_MAX);

        ret = Proto_Unpack(big_frame, out_len, &frame, &used);
        CheckRet("回环解包", PROTO_OK, ret);
        Check("解出 N = 1024", frame.data_len == PROTO_DATA_MAX);
        Check("数据内容一致", memcmp(frame.data, big, PROTO_DATA_MAX) == 0);
    }

    /* ------------------------------------------------------------------
     * 12. 测试汇总
     * ------------------------------------------------------------------ */
    printf("\r\n================ 自测结束：通过 %d 项，失败 %d 项 ================\r\n",
           s_pass, s_fail);
    printf("%s\r\n", (s_fail == 0) ? ">>> 全部通过，协议模块可以用于下一步。" 
                                  : ">>> 有失败项，请检查！");
}

/* ======================= PC 端入口 ======================= */
#ifdef PROTO_TEST_PC
int main(void)
{
    Proto_SelfTest();
    return (s_fail == 0) ? 0 : 1;
}
#endif
