#include "stc8h.h"
#include "oled.h"
#include <intrins.h>
#include <stdio.h>
#include <math.h>

#define MAIN_Fosc 24000000L// 24MHz 主频

sbit BEEP = P1^0;   // 蜂鸣器连接在 P1.0
sbit KEY  = P3^2;   // 按键连接在 P3.2，使用外部中断 0 来捕获频率占空比
sbit PWM_OUT = P3^3;    // 自测信号输出，连接在 P3.3，配合定时器 0 产生固定频率的方波

// 电阻档位开关，连接在 P3.4-P3.7
sbit R_SW_200K = P3^4;
sbit R_SW_20K  = P3^5; 
sbit R_SW_2K   = P3^6; 
sbit R_SW_200  = P3^7;

// 定义模式与量程常量
#define MODE_VOLT 0     // 电压测量模式
#define MODE_RES  1     // 电阻测量模式
#define MODE_CONT 2     // 连通性测试模式
#define MODE_FREQ 3     // 频率占空比测量模式
#define MODE_DIODE 4    // 二极管测试模式
#define MODE_AC_VOLT 5    // 交流电压测量模式

// 定义电压量程
#define RANGE_20V     0   // 20V 档
#define RANGE_2000MV  1   // 2000mV 档


#define ADC_ZERO 2125   // ADC 零点偏移值，基于实测数据微调后得到的校准值
#define CAL_VOLT 0.01267f   // 20V 档位系数 (即 5V / 4095 * 1000，单位换算)，根据实测值微调后得到的校准系数
#define CAL_2000MV 66.11f    // 2000mV档位的系数 (即 20V档系数 * 1000，单位换算)

// 电阻档位的标准值，基于实测数据微调后得到的校准值
#define R_STD_200K 197000.0f
#define R_STD_20K  16900.0f
#define R_STD_2K   1900.0f
#define R_STD_200  190.0f

// 频率占空比测量相关变量
unsigned long Actual_Freq = 0;  // 实际频率，单位 Hz
unsigned int Actual_Duty = 0;   // 实际占空比，单位百分比 (0-100)
volatile unsigned long xdata T_High = 0, T_Period = 0;  // T_High: 高电平持续时间，T_Period: 周期时间
volatile unsigned long xdata T_Start = 0;   // T_Start: 上升沿时间戳
volatile unsigned long xdata Overflow_Cnt = 0; // 溢出累加器
unsigned int High_Overflow = 0; // 高电平期间的溢出数
unsigned char xdata auto_delay_cnt = 0; // 跳档消抖计数器
char xdata buf[32];               // 将 buf 移出 main 成为全局变量
static unsigned int xdata ac_samples[64]; // 将采样数组定义为静态或全局，不占用栈空间
// 基于主频的粗略延时函数，适用于简单的消抖和等待
void delay_ms(unsigned int ms) {
    unsigned int i;
    do {
        i = MAIN_Fosc / 13000;
        while (--i);
    } while (--ms);
}

// 基于主频的微秒级延时函数，适用于更精细的消抖和短暂等待
void delay_us(unsigned int us) {
    while (us--) {
        _nop_(); _nop_(); _nop_(); _nop_(); 
    }
}

unsigned char ScanKey() {
    if (KEY == 0) {
        unsigned int low_count = 0;
        unsigned int total_ticks = 0;
        
        // 第一步：初步消抖
        // 如果是 1kHz 的信号，低电平最多持续 1ms。
        // 我们等待 10ms，如果依然有大量低电平，说明可能是按键。
        delay_ms(10); 
        
        // 第二步：特征识别
        // 在 50ms 内采样，如果持续为低，则是按键；如果跳变，则是信号。
        for(total_ticks = 0; total_ticks < 50; total_ticks++) {
            if(KEY == 0) low_count++;
            delay_ms(1);
        }

        if (low_count > 40) { // 50ms 内有 80% 以上时间是低电平，判定为按键
            // 第三步：区分长短按
            unsigned int hold_time = 0;
            while (hold_time < 100) { // 检测是否达到 1s (50ms + 10ms*95)
                delay_ms(10);
                if (KEY == 1) break; // 信号变高，说明松手或只是信号
                hold_time++;
                
                if (hold_time >= 95) { // 达到长按阈值
                    BEEP = 1; delay_ms(50); BEEP = 0;
                    while(KEY == 0); // 彻底死等松手
                    T_Period = 0; // 清除按键抖动产生的错误频率数据
                    T_High = 0;
                    return 2; // 长按
                }
            }
            T_Period = 0; // 清除按键抖动产生的错误频率数据
            T_High = 0;
            return 1; // 短按
        }
    }
    return 0;
}

void Switch_Res_Range(unsigned char range) {
    // 1. 彻底切断所有量程开关，防止并联干扰
    // 将 P3.4, P3.5, P3.6, P3.7 全部设为高阻输入 (M1=1, M0=0)
    P3M1 |= 0xF0; 
    P3M0 &= ~0xF0; 
    
    // 确保引脚输出电平清零 (虽然高阻状态下电平不输出，但这可以防止切换过程中产生干扰)
    R_SW_200K = 0; 
    R_SW_20K  = 0; 
    R_SW_2K   = 0; 
    R_SW_200  = 0;

    // 2. 开启目标档位并设为推挽输出 (M1=0, M0=1) 充当 5V 电源
    switch(range) {
        case 0: // 200k 档 (P3.4)
            P3M1 &= ~0x10; P3M0 |= 0x10; 
            R_SW_200K = 1; 
            break; 
        case 1: // 20k 档 (P3.5)
            P3M1 &= ~0x20; P3M0 |= 0x20; 
            R_SW_20K  = 1; 
            break; 
        case 2: // 2k 档 (P3.6)
            P3M1 &= ~0x40; P3M0 |= 0x40; 
            R_SW_2K   = 1; 
            break; 
        case 3: // 200R 档 (P3.7)
            P3M1 &= ~0x80; P3M0 |= 0x80; 
            R_SW_200  = 1; 
            break; 
    }
    // 注：为了解决测量不准和 OPEN 问题，
    // 调用此函数后请务必执行 delay_ms(20) 以上再进行 ADC 采样
}


// ADC 初始化
void ADC_Init() {
    P_SW2 |= 0x80;
    ADCCFG = 0x2f;    
    ADC_CONTR = 0x80; 
    delay_ms(20);
}

// 获取 ADC
unsigned int Get_ADC(unsigned char ch) {
    ADC_CONTR = 0x80 | ch; 
    _nop_(); _nop_(); _nop_(); _nop_(); 
    //delay_ms(1);           
    delay_us(50); // 等待采样稳定，100us 的延时更合理
    ADC_CONTR |= 0x40;     
    while (!(ADC_CONTR & 0x20)); 
    ADC_CONTR &= ~0x20;    
    return ((unsigned int)ADC_RES << 8) | ADC_RESL;
}

// 1. 硬件捕获初始化：配置定时器和外部中断
void Hardware_Capture_Init() {
    P_SW2 |= 0x80;
    // P3.2 设置为高阻输入并开启上拉
    P3M1 |= 0x04; P3M0 &= ~0x04; 
    P3PU |= 0x04;

    AUXR |= 0x80;  // T0 设置为 1T 模式
    TMOD &= 0xF0;  // 模式 0: 16位自动重装
    TL0 = 0; TH0 = 0;
    TR0 = 1;       // 开启计时器
    ET0 = 1;       // 允许 T0 溢出中断
    IT0 = 0;       // INT0 设置为上升沿和下降沿均触发
    EX0 = 1;       // 允许外部中断
    EA = 1;        // 开启总中断
}

// 2. 定时器 0 中断：处理溢出基准
void Timer0_ISR(void) interrupt 1 {
    static unsigned char toggle_cnt = 0;
    
    Overflow_Cnt++; // 每次溢出(约2.73ms)记录一次
    
    // 产生自测信号：每 4 次溢出翻转一次，产生约 45.7Hz 信号
    if(++toggle_cnt >= 4) { 
        PWM_OUT = !PWM_OUT;
        toggle_cnt = 0;
    }
}

// 3. 外部中断 0：合成完整时间戳
void External_INT0_ISR(void) interrupt 0 {
    unsigned int low;
    unsigned long high;
    unsigned long now_tick;

    // 先读计数值，再读溢出次数
    low = ((unsigned int)TH0 << 8) | TL0;
    high = Overflow_Cnt;

    // 补偿逻辑：如果定时器正好溢出但 T0 中断还没来得及运行
    if ((TCON & 0x20) && (low < 0x8000)) high++; 
    
    now_tick = (high << 16) | low;

    if (P32) { // 上升沿
        T_Period = now_tick - T_Start;
        T_Start = now_tick;
    } else {   // 下降沿
        T_High = now_tick - T_Start;
    }
}

void Welcome(void) {
    // 第一行显示“数字万用表” (居中建议从 x=24 开始，每个字宽 16)
    OLED_ShowChinese(24, 0, 0); // 数
    OLED_ShowChinese(40, 0, 1); // 字
    OLED_ShowChinese(56, 0, 2); // 万
    OLED_ShowChinese(72, 0, 3); // 用
    OLED_ShowChinese(88, 0, 4); // 表
    
    // 第三行显示作者信息
    // 注意：因为汉字占了两行（0和1），这里从第 4 页开始显示
    OLED_ShowChinese(32, 4, 5); // 陈
    OLED_ShowChinese(48, 4, 6); // 皓
    OLED_ShowChinese(64, 4, 7); // 宇
    
    // 第四行显示学号 (使用你原有的 ShowString 函数)
    OLED_ShowString(24, 6, "25521033"); 
    
    delay_ms(2500); // 维持 2.5 秒
    OLED_Clear();
}

void main() {

    // --- 0. 变量定义  ---
    unsigned int adc_v, adc_r;  // 原始 ADC 值
    unsigned long v_sum;    // 电压测量的累加和，用于平均计算
    float xdata voltage, display_temp, res_val; // 电压和电阻的计算结果
    unsigned char i, keyAction; // 循环变量和按键状态
    unsigned char currentMode = 0;   // 当前大模式 (0-4)
    unsigned char currentRange = 0; // 电压模式下的量程
    unsigned char resAutoStep = 0;  // 电阻模式下的当前尝试档位
    //char xdata buf[20]; // 显示缓冲区
    bit isAutoRes = 1;           // 1: 自动量程, 0: 手动量程
    unsigned char auto_cnt = 0;  // 跳档消抖计数器
    unsigned long sum_raw = 0;
    float v_avg, v_now, sum_sq = 0;

    // --- 1. 硬件初始化 ---
    P_SW2 |= 0x80; 

    // P1 配置：蜂鸣器推挽，ADC高阻
    P1M1 = 0x3A; 
    P1M0 = 0x01; 
    BEEP = 0;
    //p1.4设置为高阻输入
    P1M1 |= 0x10;
    P1M0 &= ~0x10;

    // P3 配置：
    // 1. 立即将 P3.4-P3.7 (量程开关) 设为高阻输入，防止干扰电压测量
    P3M1 |= 0xF0; P3M0 &= ~0xF0; 
    
    // 2. 将 P3.3 (自测信号输出) 设为推挽输出
    P3M1 &= ~0x08; P3M0 |= 0x08;
    P3PU |= 0x04; // 0x04 对应 P3.2，开启上拉防止浮空

    // 3. 开启 P2 (OLED) 所在的准双向模式
    P2M1 &= ~0x30; P2M0 &= ~0x30;

    OLED_Init(); // OLED 初始化
    Welcome();    // 欢迎界面
    OLED_Clear();   // 清屏
    ADC_Init();  // ADC 初始化
    Hardware_Capture_Init();    // 包含频率占空比测量的初始化

    //调试：直接跳到对应模式
    //二极管正负测试
    //currentMode = MODE_DIODE;

    // --- 2. 主循环 ---
    while(1) {
        // --- A. 按键扫描 ---
        keyAction = ScanKey(); 
        if (keyAction == 2) { // 长按：切换大模式
            currentMode++;
            if (currentMode > 5) currentMode = 0;
            
            // 模式切换后的彻底初始化
            BEEP = 0;           
            T_Period = 0;       
            T_High = 0;
            resAutoStep = 0;    // 电阻默认回到第一档 (200k)
            isAutoRes = 1;      // 模式切换时，电阻档默认开启 [自动量程]
            
            OLED_Clear(); 
            // 针对新模式立即做一次硬件配置
            if(currentMode == MODE_RES) Switch_Res_Range(resAutoStep);
            else if(currentMode == MODE_CONT) Switch_Res_Range(3); 
        }
        else if (keyAction == 1) { 
            if (currentMode == MODE_VOLT) {
                currentRange = (currentRange + 1) % 2; 
            } 
            else if (currentMode == MODE_RES) {
                // 优化后的电阻档切换逻辑
                if (isAutoRes) {
                    // 1. 从自动切手动：固定进入 200K 档 (0)
                    isAutoRes = 0;
                    resAutoStep = 0; 
                } else {
                    // 2. 手动模式下循环
                    resAutoStep++;
                    if (resAutoStep > 3) {
                        // 3. 跑完 200R 后，回到自动模式
                        resAutoStep = 0;
                        isAutoRes = 1; 
                    }
                }
                // 切换硬件电平并清屏防止残影
                Switch_Res_Range(resAutoStep); 
                OLED_Clear(); 
            }
            // 其他模式的清屏处理
            if (currentMode != MODE_RES) OLED_Clear(); 
        }

        // 安全保护
        if (currentMode != MODE_CONT) {
            BEEP = 0; 
        }

        // --- B. 模式处理逻辑 ---
        switch (currentMode) {
            case MODE_VOLT:
                // 在测量电压前，必须确保电阻档的所有 IO 恢复为高阻输入，且输出为 0
                P3M1 |= 0xF0; P3M0 &= ~0xF0; 
                R_SW_200K = 0; R_SW_20K = 0; R_SW_2K = 0; R_SW_200 = 0;

                // 采集 10 次 ADC 取平均，显著降低抖动
                v_sum = 0;
                for(i=0; i<10; i++) { v_sum += Get_ADC(3); }
                adc_v = (unsigned int)(v_sum / 10);

                // --- 显示模式和量程 ---
                if(currentRange == RANGE_20V) {
                    OLED_ShowString(0, 0, "MODE: VOLT 20V  ");
                } else {
                    OLED_ShowString(0, 0, "MODE: VOLT 2000m");
                }

                // --- 修正零点偏移 ---
                // 使用实测值 2125 作为基准
                if (adc_v > 1850 && adc_v < 1860) { 
                    voltage = 0.0f; // 强制归零，防止抖动
                } else {
                    if (adc_v >= 1855) {
                        // 正电压计算
                        voltage = (float)((long)adc_v - 1855) * 0.01301f;
                    } else {
                        // 负电压计算
                        voltage = (float)((long)adc_v - 1855) * 0.00588f;
                    }
                }

                // --- 2000mV 档位转换 ---
                // 如果是 2000mV 档位，数值放大 1000 倍，单位换成 mV
                if (currentRange == RANGE_2000MV) {
                    display_temp = voltage * 1000.0f; 
                } else {
                    display_temp = voltage;
                }

                // --- 使用格式化输出 ---
                // sprintf 会自动处理正负号和对齐，末尾加空格清除残留字符
                if (currentRange == RANGE_2000MV) {
                    // 2000mV 档显示格式: -1500 mV
                    sprintf(buf, "VAL:%5.0f mV  ", display_temp); 
                } else {
                    // 20V 档显示格式: -12.34 V
                    // %c 处理符号，%5.2f 保证 2 位小数
                    sprintf(buf, "VAL:%c%2u.%02u V  ", 
                            (voltage >= 0 ? '+' : '-'), 
                            (unsigned int)fabs(display_temp), 
                            (unsigned int)(fabs(display_temp) * 100) % 100);
                }
                OLED_ShowString(0, 2, buf);

                // 调试行：实时监控原始 ADC
                sprintf(buf, "V_RAW:%4u      ", adc_v);
                OLED_ShowString(0, 6, buf);
                break;

            case MODE_RES:
                // --- 1. 硬件采样 ---
                Switch_Res_Range(resAutoStep);
                delay_ms(30); 
                adc_r = Get_ADC(1);

                // --- 2. 自动跳档逻辑 (修复 Bug 2: 计数器重置冲突) ---
                if (isAutoRes) {
                    unsigned int down_limit = 0, up_limit = 5000; // 初始化边界

                    // 设置当前档位的判定边界
                    if (resAutoStep == 0) { down_limit = 2580; up_limit = 4095; }
                    else if (resAutoStep == 1) { down_limit = 2200; up_limit = 2800; }
                    else if (resAutoStep == 2) { down_limit = 1000; up_limit = 3000; }
                    else if (resAutoStep == 3) { down_limit = 0;    up_limit = 2500; }

                    // 核心跳档判定：不在区间内才计数
                    if (resAutoStep < 3 && adc_r < down_limit) {
                        if (++auto_cnt > 3) { 
                            resAutoStep++; auto_cnt = 0; OLED_Clear(); break; 
                        }
                    } 
                    else if (resAutoStep > 0 && adc_r > up_limit) {
                        if (++auto_cnt > 3) { 
                            resAutoStep--; auto_cnt = 0; OLED_Clear(); break; 
                        }
                    }
                    else {
                        auto_cnt = 0; // 只有在合适的范围内，才清零计数器
                    }
                }

                // --- 3. 界面显示 (修复 Bug 1: 手动模式不显示档位) ---
                if (isAutoRes) {
                    OLED_ShowString(0, 0, "MODE: RES [AUTO]");
                    OLED_ShowString(0, 2, "                "); // 自动模式隐藏档位行
                } else {
                    OLED_ShowString(0, 0, "MODE: RES [MANU]");
                    // 确保手动模式下每一帧都更新档位显示
                    if(resAutoStep == 0)      sprintf(buf, "GEAR: 200K     ");
                    else if(resAutoStep == 1) sprintf(buf, "GEAR: 20K      ");
                    else if(resAutoStep == 2) sprintf(buf, "GEAR: 2K       ");
                    else                      sprintf(buf, "GEAR: 200R     ");
                    OLED_ShowString(0, 2, buf);
                }

                // --- 4. 核心数值计算 ---
                {
                    float adcf = (float)adc_r;
                    if (resAutoStep == 0) { // 200K
                        if (adc_r >= 2830) sprintf(buf, "VAL:  OPEN      ");
                        else {
                            res_val = (121.0f * (adcf - 2522.0f)) / (2840.0f - adcf);
                            sprintf(buf, "VAL: %7.2f K ", (res_val < 0) ? 0 : res_val);
                        }
                    } 
                    else if (resAutoStep == 1) { // 20K
                        if (adc_r >= 3740) sprintf(buf, "VAL:  OPEN      ");
                        else {
                            res_val = (45.0f * (adcf - 2020.0f)) / (3755.0f - adcf);
                            sprintf(buf, "VAL: %7.2f K ", (res_val < 0) ? 0 : res_val);
                        }
                    }
                    else if (resAutoStep == 2) { // 2K
                        if (adc_r >= 4040) sprintf(buf, "VAL:  OPEN      ");
                        else {
                            res_val = (2800.0f * (adcf - 720.0f)) / (4055.0f - adcf);
                            if (res_val < 0) res_val = 0;
                            if (res_val >= 1000.0f) sprintf(buf, "VAL: %7.2f K ", res_val / 1000.0f);
                            else sprintf(buf, "VAL: %7.1f R ", res_val);
                        }
                    }
                    else { // 200R
                        if (adc_r >= 4075) sprintf(buf, "VAL:  OPEN      ");
                        else {
                            res_val = (329.0f * (adcf - 65.0f)) / (4090.0f - adcf);
                            sprintf(buf, "VAL: %7.1f R ", (res_val < 0) ? 0 : res_val);
                        }
                    }
                }

                // --- 5. 刷新显示 ---
                OLED_ShowString(0, 4, buf);
                sprintf(buf, "R_RAW: %4u     ", adc_r); 
                OLED_ShowString(0, 6, buf);
                break;

            case MODE_CONT:
                OLED_ShowString(0, 0, "MODE: CONT     ");
                Switch_Res_Range(3); // 强制使用 200R 档位以获得 10Ω 附近最高精度
                delay_ms(20);        // 给电容充电/切换留足时间
                adc_r = Get_ADC(1);

                // --- 核心判定逻辑 ---
                // 根据实测参数：10欧姆对应 ADC 约为 184
                if (adc_r < 184) { 
                    OLED_ShowString(0, 4, "STATUS: SHORT ");
                    BEEP = 1; 
                } else {
                    OLED_ShowString(0, 4, "STATUS: OPEN  ");
                    BEEP = 0; 
                }

                // --- 可选：实时显示阻值，方便确认 ---
                /*
                {
                    float adcf = (float)adc_r;
                    if (adc_r >= 4075) {
                        sprintf(buf, "RES:  O.L      ");
                    } else {
                        // 使用你实测的 200R 拟合公式计算实时电阻
                        res_val = (329.0f * (adcf - 65.0f)) / (4090.0f - adcf);
                        if (res_val < 0) res_val = 0;
                        sprintf(buf, "RES: %5.1f R    ", res_val);
                    }
                    OLED_ShowString(0, 6, buf);
                }*/
                break;

            case MODE_FREQ:
                OLED_ShowString(0, 0, "FREQ / DUTY   ");
                
                // 计算逻辑 (临时关闭中断保护数据原子性)
                ET0 = 0; EX0 = 0;
                if (T_Period > 100) { // 简单滤波
                    Actual_Freq = MAIN_Fosc / T_Period;
                    Actual_Duty = (unsigned int)((T_High * 100) / T_Period);
                } else {
                    Actual_Freq = 0;
                    Actual_Duty = 0;
                }
                ET0 = 1; EX0 = 1;

                // --- 统一宽度对齐 ---
                // 使用 %7lu 保证频率占 7 位，使用 %7u 保证占空比也占 7 位
                // 这样 Hz 和 % 就会从第 8 个字符位置开始对齐
                sprintf(buf, "%7lu Hz  ", Actual_Freq);
                OLED_ShowString(0, 2, buf);
                
                sprintf(buf, "%7u %%   ", Actual_Duty);
                OLED_ShowString(0, 4, buf);
                break;
            
            case MODE_DIODE:
                OLED_ShowString(0, 0, "MODE: DIODE    ");
                
                // --- 1. 均值采样 (减少数据跳变) ---
                Switch_Res_Range(2); 
                delay_ms(30);
                {
                    unsigned long sum = 0;
                    unsigned char i;
                    for(i=0; i<8; i++) sum += Get_ADC(1); // 累加8次
                    adc_r = (unsigned int)(sum >> 3);    // 求平均值
                }
                voltage = 5.0f * (float)adc_r / 4055.0f;

                // --- 2. 后台特征探测 ---
                Switch_Res_Range(1); 
                delay_ms(10);
                {
                    unsigned int adc_check = Get_ADC(1);
                    
                    // --- 3. 逻辑判定 (加入噪声余量) ---
                    
                    // 空载判定：给天花板留出 10 个单位的噪声余量
                    // 2K档空载 4055 -> 阈值设为 4045
                    // 20K档空载 3755 -> 阈值设为 3745
                    if (adc_r >= 4045 && adc_check >= 3745) {
                        sprintf(buf, "Vf:            "); 
                    }
                    // 反接判定：只要不是绝对空载，且电压依然很高
                    else if (adc_r >= 3850) {
                        sprintf(buf, "Vf:     -      "); 
                    }
                    // 短路判定
                    else if (adc_r < 60) {
                        sprintf(buf, "Vf:  0.000 V   ");
                    }
                    // 正接导通
                    else {
                        sprintf(buf, "Vf:  %.3f V   ", voltage);
                    }
                }

                Switch_Res_Range(2); 
                OLED_ShowString(0, 2, buf);
                OLED_ShowString(0, 4, "RED:[+]  BLK:[-] "); 
                
                // 辅助调试：观察噪声水平
                // sprintf(buf, "RAW:%4u CHK:%4u", adc_r, adc_check);
                // OLED_ShowString(0, 6, buf);
                break;

            case MODE_AC_VOLT: // 交流 2000mV 测量 (使用 P3.2)
                // 强制 P3.4-P3.7 恢复高阻，确保 P3.2 不受电阻档供电干扰
                P3M1 |= 0xF0; P3M0 &= ~0xF0; 
                R_SW_200K = 0; R_SW_20K = 0; R_SW_2K = 0; R_SW_200 = 0;
                OLED_ShowString(0, 0, "MODE: AC 2000mV");
                {
                    sum_raw = 0;
                    sum_sq = 0;

                    // 1. 高速采样
                    // 注意：因为你的 Get_ADC 里有 delay_ms(1)，
                    // 64次采样会耗时 64ms 以上，这刚好能覆盖约 3 个交流周期(50Hz)
                    for(i = 0; i < 64; i++) {
                        ac_samples[i] = Get_ADC(1); // 改为 P3.2 所在的通道 1
                        sum_raw += ac_samples[i];
                    }

                    // 计算直流偏置 (DC Offset)
                    v_avg = (float)sum_raw / 64.0f;

                    // 2. 计算真有效值 (RMS)
                    for(i = 0; i < 64; i++) {
                        // 将原始 ADC 值减去偏移，并换算为电压 (mV)
                        v_now = ((float)ac_samples[i] - v_avg) * (5000.0f / 4095.0f);
                        sum_sq += v_now * v_now;
                    }
                    
                    // 公式：$V_{rms} = \sqrt{\frac{\sum V_n^2}{N}}$
                    voltage = (float)sqrt(sum_sq / 64.0f);
                }

                // 3. 结果显示
                if (voltage > 2200.0f) {
                    sprintf(buf, "VAL:  O.L      ");
                } else {
                    // 滤除极小的感应噪声
                    if (voltage < 5.0f) voltage = 0.0f; 
                    sprintf(buf, "VAL:%6.1f mV ", voltage); 
                }
                OLED_ShowString(0, 2, buf);

                // 4. 同时监测频率 (P3.2 复用)
                sprintf(buf, "FRQ:%7lu Hz ", Actual_Freq);
                OLED_ShowString(0, 4, buf);
                
                // 辅助观察：显示当前的直流偏置中点
                sprintf(buf, "DC_OFF: %4.0f  ", v_avg);
                OLED_ShowString(0, 6, buf);
                break;
        }
    }
}