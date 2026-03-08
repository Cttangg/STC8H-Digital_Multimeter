#include "oled.h"
#include <intrins.h>
#include "oledfont.h"

void I2C_Delay() {
    unsigned char i;
    for(i=0; i<100; i++) _nop_(); 
}

void I2C_Start() {
    OLED_SCL = 1; OLED_SDA = 1; I2C_Delay();
    OLED_SDA = 0; I2C_Delay();
    OLED_SCL = 0; I2C_Delay();
}

void I2C_Stop() {
    OLED_SDA = 0; I2C_Delay();
    OLED_SCL = 1; I2C_Delay();
    OLED_SDA = 1; I2C_Delay();
}

void Write_Byte(unsigned char dat) {
    unsigned char i;
    for(i=0; i<8; i++) {
        OLED_SDA = (dat & 0x80) ? 1 : 0;
        I2C_Delay();
        OLED_SCL = 1; I2C_Delay();
        OLED_SCL = 0; I2C_Delay();
        dat <<= 1;
    }
    // ACK
    OLED_SDA = 1; I2C_Delay();
    OLED_SCL = 1; I2C_Delay();
    OLED_SCL = 0; I2C_Delay();
}

void Write_Cmd(unsigned char cmd) {
    I2C_Start();
    Write_Byte(0x78); Write_Byte(0x00); Write_Byte(cmd);
    I2C_Stop();
}

void Write_Dat(unsigned char dat) {
    I2C_Start();
    Write_Byte(0x78); Write_Byte(0x40); Write_Byte(dat);
    I2C_Stop();
}

void OLED_SetPos(unsigned char x, unsigned char y) {
    Write_Cmd(0xb0 + y);
    Write_Cmd(((x & 0xf0) >> 4) | 0x10);
    Write_Cmd(x & 0x0f);
}

void OLED_Init() {
    unsigned int i;
    for(i=0; i<2000; i++) I2C_Delay();
    
    Write_Cmd(0xAE); // 关闭显示
    Write_Cmd(0x8D); Write_Cmd(0x14); // 必须：开启电荷泵
    
    // --- 屏幕翻转与显示修正指令 ---
    Write_Cmd(0xA1); // 段重映射 (0xA0/0xA1 切换试验)
    Write_Cmd(0xC8); // COM扫描方向 (0xC0/0xC8 切换试验)
    
    Write_Cmd(0xDA); Write_Cmd(0x12); // 关键：硬件引脚配置，不设置可能不亮
    Write_Cmd(0xD3); Write_Cmd(0x00); // 设置显示偏移
    Write_Cmd(0x40); // 设置起始行
    
    Write_Cmd(0xAF); // 开启显示
    OLED_Clear();    // 清屏
}

void OLED_Clear() {
    unsigned char i, j;
    for(i=0; i<8; i++) {
        OLED_SetPos(0, i);
        for(j=0; j<128; j++) Write_Dat(0x00);
    }
}

// 显示单个 8x16 字符
void OLED_ShowChar(unsigned char x, unsigned char y, char chr) {
    unsigned char i;
    unsigned char c = chr - ' '; // 计算 ASCII 偏移量

    // 1. 显示上半部分 (前 8 个字节)
    OLED_SetPos(x, y);
    for (i = 0; i < 8; i++) {
        // 注意：这里需确认你的 oledfont.h 中数组名是 F8X16 还是 F8x16
        Write_Dat(F8X16[c][i]); 
    }

    // 2. 显示下半部分 (后 8 个字节)
    // 8x16 字符高度占 16 像素，即 OLED 的 2 页 (Page)
    OLED_SetPos(x, y + 1);
    for (i = 8; i < 16; i++) {
        Write_Dat(F8X16[c][i]);
    }
}

// 显示字符串
void OLED_ShowString(unsigned char x, unsigned char y, char *s) {
    while (*s != '\0') {
        OLED_ShowChar(x, y, *s);
        x += 8;      // 每个字符宽度为 8 像素
        if (x > 120) { // 整行写满后换行
            x = 0; 
            y += 2;    // 高度为 16 像素，所以 y 坐标需下移 2 行
        }
        s++;
    }
}
// 计算 m^n 的函数，供显示数字使用
unsigned long OLED_Pow(unsigned char m, unsigned char n)
{
    unsigned long result = 1;	 
    while(n--) result *= m;    
    return result;
}

/**
  * @brief  OLED显示数字
  * @param  x: 起始横坐标 (0~127)
  * @param  y: 起始纵坐标 (0~7)
  * @param  num: 要显示的数字 (0~4294967295)
  * @param  len: 数字的位数
  * @param  size: 字体大小 (16/12)
  * @retval 无
  */

void OLED_ShowNumber(unsigned char x, unsigned char y, unsigned long num, unsigned char len, unsigned char size)
{         	
    unsigned char t, temp;
    unsigned char enshow = 0;						   
    for(t = 0; t < len; t++)
    {
        temp = (num / OLED_Pow(10, len - t - 1)) % 10;
        if(enshow == 0 && t < (len - 1))
        {
            if(temp == 0)
            {
                OLED_ShowChar(x + (size / 2) * t, y, ' '); // 高位为0时显示空格（消隐）
                continue;
            }
            else enshow = 1; 
        }
        OLED_ShowChar(x + (size / 2) * t, y, temp + '0'); 
    }
}