/**
  ******************************************************************************
  * @file    xv7001bb.c
  * @brief   XV7001BB 陀螺仪驱动（4 线 SPI，MSB First，CPOL=0，CPHA=0）
  *          接线：PB13 = SCK，PB14 = MISO，PB15 = MOSI，PB12 = 片选 CS（软件控制）
  *          底层：SPI2（见 spi.c 的 Spi_Init / Spi_ReadWriteByte）
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "xv7001bb.h"

/* Private define ------------------------------------------------------------*/
/* 片选（CS）：软件控制 PB12（由本驱动层负责翻转，原因见 spi.c 中说明） */
#define XV7001_CS_LOW()         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define XV7001_CS_HIGH()        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)

/* SPI 命令位：bit7 = 1 读，bit7 = 0 写 */
#define XV7001_CMD_READ         0x80U
#define XV7001_CMD_WRITE_MASK   0x7FU

/* 寄存器地址 */
#define XV7001_REG_STATUS       0x04U   /* 状态寄存器（初始化时用于验证通信） */
#define XV7001_REG_TEMP         0x08U   /* TempRd：温度数据（16 位寄存器，连续读 2 字节） */
#define XV7001_REG_DATACC       0x0AU   /* DatAccOn：角速度数据（16 位寄存器，连续读 2 字节） */
#define XV7001_REG_OUTCTL1      0x0BU   /* OutCtl1：bit2 DataFormat（0 = 16 位，1 = 24 位） */
#define XV7001_REG_DSPRES       0x0DU   /* DspRes：数字滤波器复位（仅命令，无数据） */
#define XV7001_OUT_FMT24        0x04U   /* OutCtl1 的 DataFormat 位 */

/* Global variables ----------------------------------------------------------*/
/* 初始化时读回的状态寄存器值（调试用，可加入 Watch 观察） */
volatile uint8_t g_xv7001bbStatus = 0;
/* 写入 24 位设置后读回的 0x0B：bit2 = 1 表示 24 位输出已生效，否则退回 16 位（可加入 Watch） */
volatile uint8_t g_xv7001bbDataFormat = 0;

static uint8_t s_fmt24 = 0U;            /* 1 = 角速度按 24 位读取 */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  XV7001BB 初始化：初始化 SPI2，等待芯片上电稳定，读状态寄存器验证通信，切换到 24 位角速度输出
  * @param  None
  * @retval None
  */
void XV7001bb_Init(void)
{
	uint8_t outctl;

	Spi_Init();                             /* 初始化 SPI2（含 PB12 片选引脚，末尾 __HAL_SPI_ENABLE） */
	HAL_Delay(100);                         /* 等待芯片上电稳定（100ms） */

	/* 读状态寄存器（0x04）验证 SPI 通信，结果存入全局变量便于 Watch 观察 */
	g_xv7001bbStatus = XV7001bb_ReadReg(XV7001_REG_STATUS);

	/* 切换到 24 位输出：读-改-写 0x0B，然后复位数字滤波器，再读回确认
	 * （写不进去时自动退回 16 位读法，读出的值仍按 24 位等效返回，灵敏度系数不变） */
	outctl = XV7001bb_ReadReg(XV7001_REG_OUTCTL1);
	XV7001bb_WriteData(XV7001_REG_OUTCTL1, (uint8_t)(outctl | XV7001_OUT_FMT24));

	XV7001_CS_LOW();
	Spi_ReadWriteByte(XV7001_REG_DSPRES & XV7001_CMD_WRITE_MASK);
	XV7001_CS_HIGH();
	HAL_Delay(10);

	g_xv7001bbDataFormat = XV7001bb_ReadReg(XV7001_REG_OUTCTL1);
	s_fmt24 = ((g_xv7001bbDataFormat & XV7001_OUT_FMT24) != 0U) ? 1U : 0U;
}

/**
  * @brief  写数据
  *         时序：拉低片选(PB12) -> 发送 reg_addr & 0x7F -> 发送 data -> 拉高片选
  * @param  reg_addr 寄存器地址
  * @param  data     要写入的数据
  * @retval None
  */
void XV7001bb_WriteData(uint8_t reg_addr, uint8_t data)
{
	XV7001_CS_LOW();                                        /* 片选拉低，开始传输 */
	Spi_ReadWriteByte(reg_addr & XV7001_CMD_WRITE_MASK);    /* 写命令：bit7 = 0 */
	Spi_ReadWriteByte(data);                                /* 数据字节 */
	XV7001_CS_HIGH();                                       /* 片选拉高，结束传输 */
}

/**
  * @brief  读寄存器
  *         时序：拉低片选(PB12) -> 发送 reg_addr | 0x80 -> 读回 1 字节 -> 拉高片选
  * @param  reg_addr 寄存器地址
  * @retval 读到的寄存器值
  */
uint8_t XV7001bb_ReadReg(uint8_t reg_addr)
{
	uint8_t data;

	XV7001_CS_LOW();                                        /* 片选拉低，开始传输 */
	Spi_ReadWriteByte(reg_addr | XV7001_CMD_READ);          /* 读命令：bit7 = 1 */
	data = Spi_ReadWriteByte(0xFFU);                        /* 发 0xFF 产生时钟，读回 1 字节 */
	XV7001_CS_HIGH();                                       /* 片选拉高，结束传输 */

	return data;
}

/**
  * @brief  读温度（寄存器 0x08，TempRd）
  *         手册依据：Table 3.6 温度传感器
  *           - 12 位模式（默认，TsDataFormat[1:0] = 10）下 25 ℃ 对应 code = 400，
  *             温度系数 16 LSB/℃
  *           - 数据 = (高字节 << 8) | 低字节，取整个 16 位数据的高 12 位（忽略最低 4 位）
  *           - 12 位有符号数（最高位为 1 时视为负数）
  * @param  None
  * @retval 温度值（℃，浮点），公式：25.0 + (code - 400) / 16.0
  */
float XV7001bb_ReadTmp(void)
{
	uint8_t high_byte;
	uint8_t low_byte;
	uint16_t data;
	int16_t raw_temp;

	/* 读 0x08：同一 CS 低电平期间连续读 2 字节（第 1 字节为高字节） */
	XV7001_CS_LOW();                                        /* 片选拉低，开始传输 */
	Spi_ReadWriteByte(XV7001_REG_TEMP | XV7001_CMD_READ);   /* 读命令：0x08 | 0x80 */
	high_byte = Spi_ReadWriteByte(0xFFU);                   /* 第 1 字节：高字节 */
	low_byte  = Spi_ReadWriteByte(0xFFU);                   /* 第 2 字节：低字节 */
	XV7001_CS_HIGH();                                       /* 片选拉高，结束传输 */

	data     = ((uint16_t)high_byte << 8) | (uint16_t)low_byte;   /* 组合 16 位数据 */
	raw_temp = (int16_t)(data >> 4);                        /* 取高 12 位（忽略最低 4 位） */

	if (raw_temp & 0x0800)                                  /* 12 位最高位为 1 → 负数 */
	{
		raw_temp |= (int16_t)0xF000;                        /* 符号扩展到 16 位 */
	}

	return 25.0f + ((float)(raw_temp - 400) / 16.0f);
}

/**
  * @brief  读角速度原始值（寄存器 0x0A，DatAccOn）
  *         手册依据：5-5-1 节 Table 5.1、Table 3.5
  *           - 24 位模式：第 1 字节 D[23:16]，第 2 字节 D[15:8]，第 3 字节 D[7:0]；71680 LSB/(°/s)
  *           - 16 位模式（仅在 24 位设置写入失败时使用）：D[15:0]，左移 8 位后与 24 位同尺度
  *           - 几个字节必须在同一个 CS 低电平期间连续读出
  * @param  None
  * @retval 角速度原始值，按 24 位有符号数的尺度（71680 LSB/(°/s)）
  */
int32_t XV7001bb_ReadAngle(void)
{
	uint8_t b2;
	uint8_t b1;
	uint8_t b0 = 0U;
	uint32_t u;

	XV7001_CS_LOW();                                        /* 片选拉低，开始传输 */
	Spi_ReadWriteByte(XV7001_REG_DATACC | XV7001_CMD_READ); /* 读命令：0x0A | 0x80 */
	b2 = Spi_ReadWriteByte(0xFFU);                          /* 24 位：D[23:16]；16 位：D[15:8] */
	b1 = Spi_ReadWriteByte(0xFFU);                          /* 24 位：D[15:8]；16 位：D[7:0] */
	if (s_fmt24 != 0U)
	{
		b0 = Spi_ReadWriteByte(0xFFU);                      /* 24 位：D[7:0] */
	}
	XV7001_CS_HIGH();                                       /* 片选拉高，结束传输 */

	if (s_fmt24 != 0U)
	{
		u = ((uint32_t)b2 << 16) | ((uint32_t)b1 << 8) | (uint32_t)b0;
		return (u & 0x800000UL) ? ((int32_t)u - 0x1000000L) : (int32_t)u;   /* 24 位补码 */
	}

	return (int32_t)(int16_t)(((uint16_t)b2 << 8) | (uint16_t)b1) * 256L;   /* 16 位补码，换算到 24 位尺度 */
}
