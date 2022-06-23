/******************************************************************************
* Copyright (C) 2016, Huada Semiconductor Co.,Ltd All rights reserved.
*
* This software is owned and published by:
* Huada Semiconductor Co.,Ltd ("HDSC").
*
* BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
* BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
*
* This software contains source code for use with HDSC
* components. This software is licensed by HDSC to be adapted only
* for use in systems utilizing HDSC components. HDSC shall not be
* responsible for misuse or illegal use of this software for devices not
* supported herein. HDSC is providing this software "AS IS" and will
* not be responsible for issues arising from incorrect user implementation
* of the software.
*
* Disclaimer:
* HDSC MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE,
* REGARDING THE SOFTWARE (INCLUDING ANY ACOOMPANYING WRITTEN MATERIALS),
* ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED USE, INCLUDING,
* WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY, THE IMPLIED
* WARRANTY OF FITNESS FOR A PARTICULAR PURPOSE OR USE, AND THE IMPLIED
* WARRANTY OF NONINFRINGEMENT.
* HDSC SHALL HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT,
* NEGLIGENCE OR OTHERWISE) FOR ANY DAMAGES WHATSOEVER (INCLUDING, WITHOUT
* LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION,
* LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY LOSS) ARISING FROM USE OR
* INABILITY TO USE THE SOFTWARE, INCLUDING, WITHOUT LIMITATION, ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA,
* SAVINGS OR PROFITS,
* EVEN IF Disclaimer HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* YOU ASSUME ALL RESPONSIBILITIES FOR SELECTION OF THE SOFTWARE TO ACHIEVE YOUR
* INTENDED RESULTS, AND FOR THE INSTALLATION OF, USE OF, AND RESULTS OBTAINED
* FROM, THE SOFTWARE.
*
* This software may be replicated in part or whole for the licensed use,
* with the restriction that this Disclaimer and Copyright notice must be
* included with each copy of this software, whether used in part or whole,
* at all times.
*/
/******************************************************************************/
/** \file main.c
 **
 ** A detailed description is available at
 ** @link Sample Group Some description @endlink
 **
 **   - 2016-02-16  1.0  XYZ First version for Device Driver Library of Module.
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "gpio.h"
#include "clk.h"
#include "wdt.h"
#include "interrupts_hc32l110.h"
#include "bt.h"
#include "pca.h"
#include "adt.h"
#include    <math.h>
//adt gpio interrupt
/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

#define     T_PCLK                                              0.25                                                                            //PCLK=4MHz, 0.25us
#define     T_BEACON_CLK                                    (T_PCLK*256)                                                                    //PCLK 256??, AdTIM6??
#define     BEACON_DUTY                                     0.8                                                                                     //flash????                                             
#define     DEBUG                                                   1
#define     BEACON_ADT_PERIOD_COUNT(t)      (uint16_t)(1000000.0*(t)/T_BEACON_CLK+0.5)
#define     BEACON_ADT_DUTY_COUNT(cnt)      (uint16_t)((cnt)*BEACON_DUTY+0.5)
uint16_t    beacon_flash_count;             //光的周期
uint16_t    beacon_flash_duty_count;    //光的占空比
uint16_t  voiceOnActPeriod;//TM4的周期
uint16_t  stap = 0; //特殊序列标志
uint16_t  flag = 0; //平波的标志

uint16_t  u16InitCntData;//BT0的周期
uint32_t  u32InitCntData;//BT1的周期
uint32_t  u32ArrData; //同步信号的周期
uint16_t  My_Bt0_flag = 1; //BT0的中断标志
uint16_t  My_Bt0_1_flag = 0; //连续锯齿波的标志
uint16_t  My_Bt1_flag = 1; //BT1的中断标志位
uint16_t  My_Bt2_flag = 0; //同步标志
uint16_t  My_slave_flag = 0; //从机检测主机是否丢失标志
uint16_t  doublet = 1; // token == 24
void My_InitBt0(uint16_t  u16InitCntData);   
void My_InitBt1(uint32_t  u32InitCntData);
void My_InitBt2(uint32_t  u32ArrData);
void My_Tim0_IRQ(void);
void My_Tim1_IRQ(void);
void My_Tim2_IRQ(void);
void My_AdTim4_IRQ(void);
unsigned char key1, key2, key3, key4, key5, key6, key7, ledin, token, tokenOld = 0xFF;
unsigned int voiceOnActCompare ;//TM4比较输出
unsigned int bgnFrq ;
unsigned int finFrq ;
unsigned int bgnFrqTm ;//方波频率执行的时间
unsigned int finFrqTm ;
unsigned int PeriodTm;//每个频率的时间
unsigned int PeriodTm_OFF;//有间断的频率时间
unsigned int discretFrqFlag = 1;  //是否方波
unsigned int bgnPeriod;//声音开始的周期
unsigned int finPeriod;//声音结束的周期
unsigned int signal;
unsigned int damaster = 1; //主
unsigned int slave = 0; //从

void key_check(void);

void My_InitAdTIM6(uint16_t beacon_flash_count, uint16_t beacon_flash_duty_count);
void My_SynAdTIM6(void);
void My_InitClk(void);

void My_InitAdTIM4(uint16_t  voiceOnActPeriod);
void My_SounderPhaseEndWaiting(void);
void Gpio_IRQHandler(uint8_t u8Param)
{
    if (3 == u8Param)
    {
        if (Gpio_GetIrqStat(3, 4))
        {
            Gpio_ClearIrq(3, 4);
            Bt_Stop(TIM1);
            Bt_Stop(TIM0);
            Adt_StopCount(AdTIM4);
            Adt_StopCount(AdTIM6);
            My_Bt1_flag = 1;
            discretFrqFlag = 1;
            My_Bt0_flag = 1;
            stap = 0;
            doublet = 1;
            My_SynAdTIM6();
        }

    }
}




int32_t main(void)
{

    My_InitClk();
    
    Gpio_InitIOExt(3, 6, GpioDirOut, TRUE, FALSE, TRUE, FALSE);
    My_Gpio_SetFunc(3, 6, 2u);

    Gpio_InitIO(0, 1, GpioDirOut);
    Gpio_InitIOExt(0, 1, GpioDirOut, TRUE, FALSE, TRUE, FALSE);
    Gpio_InitIOExt(3, 3, GpioDirIn, TRUE, FALSE, TRUE, FALSE);
    Gpio_InitIOExt(3, 2, GpioDirIn, TRUE, FALSE, TRUE, FALSE);
    Gpio_InitIOExt(2, 6, GpioDirIn, TRUE, FALSE, TRUE, FALSE);
    Gpio_InitIOExt(2, 5, GpioDirIn, TRUE, FALSE, TRUE, FALSE);
    Gpio_InitIOExt(2, 4, GpioDirIn, TRUE, FALSE, TRUE, FALSE);
    Gpio_InitIOExt(2, 3, GpioDirIn, TRUE, FALSE, TRUE, FALSE);
    Gpio_InitIOExt(1, 4, GpioDirIn, TRUE, FALSE, TRUE, FALSE);

    Gpio_SetFunc_TIM4_CHB_P15();
    key_check();
    token = key1 + key2 + key3 + key4 + key5;
    if (token != tokenOld)
    {
        switch (token)
        {
        case 0: //970Hz
            bgnFrq = 970;
            finFrq = 970;

            break;
        case 1: //800Hz 0.5s; 970Hz 0.5s

            bgnFrq = 800;
            finFrq = 970;
            bgnFrqTm = 0xFFF85EDF;
            finFrqTm = 0xFFF85EDF;
            discretFrqFlag = 1;
            break;
        case 2: //1s 800~970Hz

            bgnFrq = 800;
            finFrq = 970;
            bgnPeriod = 1250;
            finPeriod = 1031;
            PeriodTm  = 0xEE29;
            break;
        case 3: //970Hz 1s ON; 1s OFF

            bgnFrq = 970;
            finFrq = 970;
            bgnFrqTm = 0xFFF0BDBF;
            finFrqTm = 0xFFF0BDBF;
            discretFrqFlag = 1;
            break;
        case 4: //630Hz 0.5s; 970Hz 0.5s

            bgnFrq = 630;
            finFrq = 970;
            bgnFrqTm = 0xFFF85EDF;
            finFrqTm = 0xFFF85EDF;
            discretFrqFlag = 1;
            break;
        case 5: //440Hz 0.4s; 554Hz 0.1s

            bgnFrq = 440;
            finFrq = 554;
            bgnFrqTm = 0xFFF9E57F;
            finFrqTm = 0xFFFE795F;
            discretFrqFlag = 1;
            break;
        case 6: //500~1200; 3.5s ON; 0.5s OFF

            bgnFrq = 500;
            finFrq = 1200;
            bgnPeriod = 2000;
            finPeriod = 833;
            PeriodTm  = 0xF448;
            PeriodTm_OFF = 0xFE53;
            break;
        case 7: //420Hz 0.625s ON; 0.625s OFF

            bgnFrq = 420;
            finFrq = 420;
            bgnFrqTm = 0xFFF67697;
            finFrqTm = 0xFFF67697;
            discretFrqFlag = 1;
            break;
        case 8: //1000~2500; 0.5s ON/0.5s OFF x3; 1.5s OFF

            bgnFrq = 1000;
            finFrq = 2500;
            bgnPeriod = 1000;
            finPeriod = 400;
            PeriodTm  = 0xFCBF;
            break;
        case 9: //440Hz 2s; 550Hz 2s

            bgnFrq = 440;
            finFrq = 550;
            bgnFrqTm = 0xFFE17B7F;
            finFrqTm = 0xFFE17B7F;
            discretFrqFlag = 1;
            break;
        case 10: //970Hz 0.5s ON/0.5s OFF; 1.5s OFF

            bgnFrq = 970;
            finFrq = 970;
            bgnFrqTm = 0xFFF85EDF;
            finFrqTm = 0xFFF85EDF;
            break;
        case 11: //2850Hz 0.5s ON; 1.5s OFF

            bgnFrq = 2850;
            finFrq = 2850;
            bgnFrqTm = 0xFFF85EDF;
            finFrqTm = 0xFFF85EDF;
            break;
        case 12: //1200~500Hz 1s

            bgnFrq = 1200;
            finFrq = 500;
            bgnPeriod = 833;
            finPeriod = 2000;
            PeriodTm  = 0xFCA6;
            break;
        case 13: //400Hz

            bgnFrq = 400;
            finFrq = 400;

            break;
        case 14: //550Hz 0.7s; 1000Hz 0.33s

            bgnFrq = 550;
            finFrq = 1000;
            bgnFrqTm = 0xFFF5519F;
            finFrqTm = 0xFFFAF6EF;
            discretFrqFlag = 1;
            break;
        case 15: //1500~2700Hz 0.333s

            bgnFrq = 1500;
            finFrq = 2700;
            bgnPeriod = 667;
            finPeriod = 370;
            PeriodTm = 0xFB9E;
            break;
        case 16: //750Hz

            bgnFrq = 750;
            finFrq = 750;

            break;
        case 17: //2400Hz

            bgnFrq = 2400;
            finFrq = 2400;

            break;
        case 18: //660Hz

            bgnFrq = 660;
            finFrq = 660;

            break;
        case 19: //660Hz 1.8s ON; 1.8s OFF

            bgnFrq = 660;
            finFrq = 660;
            bgnFrqTm = 0xFFE488BF;
            finFrqTm = 0xFFE488BF;
            discretFrqFlag = 1;
            break;
        case 20: //660Hz 0.15 NO; 0.15 OFF

            bgnFrq = 660;
            finFrq = 660;
            bgnFrqTm = 0xFFFDB60F;
            finFrqTm = 0xFFFDB60F;
            discretFrqFlag = 1;
            break;
        case 21: //510Hz 0.25s; 610Hz 0.25s

            bgnFrq = 510;
            finFrq = 610;
            bgnFrqTm = 0xFFFC2F6F;
            finFrqTm = 0xFFFC2F6F;
            discretFrqFlag = 1;
            break;
        case 22: //800Hz 0.5s; 1000Hz 0.5s

            bgnFrq = 800;
            finFrq = 1000;
            bgnFrqTm = 0xFFF85EDF;
            finFrqTm = 0xFFF85EDF;
            discretFrqFlag = 1;
            break;
        case 23: //250~1200Hz 12Hz 83ms

            bgnFrq = 250;
            finFrq = 1200;
            bgnPeriod = 4000;
            finPeriod = 840;
            PeriodTm = 0xFDF2;
            break;
        case 24: //500~1200Hz 0.33Hz/3s

            bgnFrq = 500;
            finFrq = 1200;
            bgnPeriod = 2000;
            finPeriod = 833;
            PeriodTm = 0xF94E;

            break;
        case 25: //2400~2900Hz 9Hz/111ms

            bgnFrq = 2400;
            finFrq = 2900;
            bgnPeriod = 417;
            finPeriod = 345;
            PeriodTm = 0xF9F9;
            break;
        case 26: //2400~2900Hz 3Hz/333ms

            bgnFrq = 2400;
            finFrq = 2900;
            bgnPeriod = 417;
            finPeriod = 345;
            PeriodTm = 0xEDEE;
            break;
        case 27: //500~1200Hz 0.5s ON; 1.5s OFF

            bgnFrq = 500;
            finFrq = 1200;
            bgnPeriod = 2000;
            finPeriod = 833;
            PeriodTm = 0xFE53;
            break;
        case 28: //800~970Hz 9Hz/111ms

            bgnFrq = 800;
            finFrq = 970;
            bgnPeriod = 1250;
            finPeriod = 1031;
            PeriodTm = 0xFE04;
            break;
        case 29: //800~970Hz 3Hz/333ms

            bgnFrq = 800;
            finFrq = 970;
            bgnPeriod = 1250;
            finPeriod = 1031;
            PeriodTm  = 0xFA0F;
            break;
        case 30: //800Hz 0.25s ON; 1s OFF

            bgnFrq = 800;
            finFrq = 800;
            bgnFrqTm = 0xFFFC2F6F;
            finFrqTm = 0xFFF0BDBF;
            discretFrqFlag = 1;
            break;
        case 31: //500~1200Hz 3.75s ON; 0.25s OFF

            bgnFrq = 500;
            finFrq = 1200;
            bgnPeriod = 2000;
            finPeriod = 833;
            PeriodTm = 0xF372;
            PeriodTm_OFF = 0xFF29;
            break;
        default:
            break;
        }

        tokenOld = token;
    }

    if (key7)
    {
        beacon_flash_count =   BEACON_ADT_PERIOD_COUNT(1);
    }
    else
    {
        beacon_flash_count =   BEACON_ADT_PERIOD_COUNT(2);
    }
    beacon_flash_duty_count   =   BEACON_ADT_DUTY_COUNT(beacon_flash_count);
	
	//P35初始化必须放这（吊毛说的）
    Gpio_InitIOExt(3, 5, GpioDirOut, FALSE, TRUE, FALSE, FALSE);
    Gpio_SetIO(3, 5, FALSE);
    Gpio_InitIOExt(3, 4, GpioDirIn, TRUE, FALSE, FALSE, 0);
    //初始化计数定时器，初始化发声定时器
    unsigned int i;
    for (i = 0; i < 0x28B0AA; i++);
    Gpio_DisableIrq(3, 4,  GpioIrqFalling);
    while (1)
    {

        Gpio_SetIO(0, 1, TRUE);
        delay1ms(200);

        Gpio_GetIO(3, 4);
//主从判定，先不开3.4引脚的中断
        if (Gpio_GetIO(3, 4) == TRUE)
        {
            Gpio_SetIO(3, 5, TRUE);
            Gpio_DisableIrq(3, 4,  GpioIrqFalling);
            signal = damaster; //主机
            u32ArrData  = 0xFF85EDFF;
        }
        else
        {
            signal = slave; //从机，开中断
            Gpio_ClearIrq(3, 4);
            Gpio_EnableIrq(3, 4,  GpioIrqFalling);
            EnableNvic(PORT3_IRQn, 0, TRUE);
            u32ArrData    = 0xFF6BD1ED;
        }
        
        My_InitBt2(u32ArrData);//同步定时器       
        My_InitBt0(u16InitCntData);// 每Hz 计数
        My_InitBt1(u16InitCntData);// 每Hz 计数
        My_InitAdTIM4(voiceOnActPeriod);//pwm   VOICE
        My_InitAdTIM6(beacon_flash_count, beacon_flash_duty_count); //PWM LIGHT
        Gpio_SetIO(0, 1, FALSE);
        if (signal == damaster)
        {
            My_Bt1_flag = 1;
            My_Bt0_flag = 1;
            Bt_Run(TIM2);
            if (token == 0 || token == 13 || token == 16 || token == 17 || token == 18) //平波
            {
                while (1)
                {
                    if (My_Bt1_flag == 1)
                    {
                        Adt_StopCount(AdTIM4);
                        voiceOnActPeriod = ((1000000 / bgnFrq) + 0.5);
                        Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                        Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                        Adt_StartCount(AdTIM4);
                        Adt_StartCount(AdTIM6);
                        My_Bt1_flag = 0;
                    }

                    if (My_Bt2_flag == 1)
                    {
                        My_Bt2_flag = 0;
                        Gpio_SetIO(3, 5, FALSE);
                        My_SynAdTIM6();
                        delay100us(1);
                        if (Gpio_GetIO(3, 4) == FALSE)
                        {
                            Adt_StopCount(AdTIM4);
                            Adt_StopCount(AdTIM6);
                            break;
                        }
                        Gpio_SetIO(3, 5, TRUE);
                        My_Bt1_flag = 1;
                        Bt_Cnt32Set(TIM2, u32ArrData);
                        Bt_Run(TIM2);
                    }
                }
            }
            else if (token == 1 || token == 4 || token == 5 || token == 9 || token == 14 || token == 21 || token == 22) //方波
            {
                while (1)
                {
                    if (My_Bt1_flag == 1)
                    {
                        My_Bt1_flag = 0;
                        Adt_StartCount(AdTIM6);
                        if (discretFrqFlag == 1)
                        {
                            u32InitCntData = bgnFrqTm;
                            Bt_Cnt32Set(TIM1, u32InitCntData);
                            voiceOnActPeriod = ((1000000 / bgnFrq) + 0.5);
                            Bt_Run(TIM1);
                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                            Adt_StartCount(AdTIM4);

                        }
                        else
                        {
                            u32InitCntData = finFrqTm;
                            Bt_Cnt32Set(TIM1, u32InitCntData);
                            voiceOnActPeriod = ((1000000 / finFrq) + 0.5);
                            Bt_Run(TIM1);
                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                            Adt_StartCount(AdTIM4);
                        }
                    }
                    if (My_Bt2_flag == 1)
                    {
                        My_Bt2_flag = 0;
                        Gpio_SetIO(3, 5, FALSE);
                        My_SynAdTIM6();
                        delay100us(2);
                        if (Gpio_GetIO(3, 4) == FALSE)
                        {
                            break;
                        }
                        Gpio_SetIO(3, 5, TRUE);
                        My_Bt1_flag = 1;
                        discretFrqFlag = 1;
                        Bt_Cnt32Set(TIM2, u32ArrData);
                        Bt_Run(TIM2);
                    }
                }
            }
            else if (token == 2 || token == 12 || token == 15  || token == 25 || token == 26 || token == 28 || token == 29)//连续锯齿波
            {
                while (1)
                {
                    if (My_Bt0_flag == 1) //每个波形执行完的中断标志
                    {
                        if (finFrq > bgnFrq)
                        {
                            voiceOnActPeriod = bgnPeriod;
                            u16InitCntData = PeriodTm;
                            Bt_ARRSet(TIM0, u16InitCntData);
                            Bt_Cnt16Set(TIM0, u16InitCntData);
                            Bt_Run(TIM0);
                            Adt_StopCount(AdTIM4);
                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                            Adt_StartCount(AdTIM4);
                            Adt_StartCount(AdTIM6);
                        }
                        else
                        {
                            voiceOnActPeriod = bgnPeriod;
                            u16InitCntData = PeriodTm;
                            Bt_ARRSet(TIM0, u16InitCntData);
                            Bt_Cnt16Set(TIM0, u16InitCntData);
                            Bt_Run(TIM0);
                            Adt_StopCount(AdTIM4);
                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                            Adt_StartCount(AdTIM4);
                            Adt_StartCount(AdTIM6);
                        }
                        My_Bt0_flag = 0;
                    }
                    if (My_Bt0_1_flag == 1) //每个频率执行完进一次中断
                    {
                        if (finFrq > bgnFrq)
                        {
                            voiceOnActPeriod--;

                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);

                            if (voiceOnActPeriod <= finPeriod)
                            {
                                My_Bt0_flag = 1;
                            }
                        }
                        else
                        {
                            voiceOnActPeriod++;

                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);

                            if (voiceOnActPeriod >= finPeriod)
                            {
                                My_Bt0_flag = 1;
                            }
                        }
                        My_Bt0_1_flag = 0;
                    }
                    if (My_Bt2_flag == 1)
                    {
                        My_Bt2_flag = 0;
                        Gpio_SetIO(3, 5, FALSE);
                        My_SynAdTIM6();
                        delay100us(2);
                        if (Gpio_GetIO(3, 4) == FALSE)
                        {
                            break;
                        }
                        Gpio_SetIO(3, 5, TRUE);
                        My_Bt0_flag = 1;
                        Bt_Cnt32Set(TIM2, u32ArrData);
                        Bt_Run(TIM2);
                    }
                }
            }
            else if (token == 24)
            {
                while (1)
                {
                    if (My_Bt0_flag == 1) //每个波形执行完的中断标志
                    {
                        if (doublet == 1)
                        {
                            voiceOnActPeriod = bgnPeriod;
                            u16InitCntData = PeriodTm;
                            Bt_ARRSet(TIM0, u16InitCntData);
                            Bt_Cnt16Set(TIM0, u16InitCntData);
                            Bt_Run(TIM0);
                            Adt_StopCount(AdTIM4);
                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                            Adt_StartCount(AdTIM4);
                            Adt_StartCount(AdTIM6);
                        }
                        else
                        {
                            voiceOnActPeriod = finPeriod;
                            u16InitCntData = PeriodTm;
                            Bt_ARRSet(TIM0, u16InitCntData);
                            Bt_Cnt16Set(TIM0, u16InitCntData);
                            Bt_Run(TIM0);
                            Adt_StopCount(AdTIM4);
                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                            Adt_StartCount(AdTIM4);
                            Adt_StartCount(AdTIM6);
                        }
                        My_Bt0_flag = 0;
                    }
                    if (My_Bt0_1_flag == 1) //每个频率执行完进一次中断
                    {
                        if (doublet == 1)
                        {
                            voiceOnActPeriod--;

                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);

                            if (voiceOnActPeriod <= finPeriod)
                            {
                                My_Bt0_flag = 1;
                                doublet = 0;
                            }
                        }
                        else
                        {
                            voiceOnActPeriod++;

                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);

                            if (voiceOnActPeriod >= bgnPeriod)
                            {
                                My_Bt0_flag = 1;
                                doublet = 1;
                            }
                        }
                        My_Bt0_1_flag = 0;
                    }
                    if (My_Bt2_flag == 1)
                    {
                        My_Bt2_flag = 0;
                        Gpio_SetIO(3, 5, FALSE);
                        My_SynAdTIM6();
                        delay100us(2);
                        if (Gpio_GetIO(3, 4) == FALSE)
                        {
                            break;
                        }
                        Gpio_SetIO(3, 5, TRUE);
                        My_Bt0_flag = 1;
                        doublet = 1;
                        Bt_Cnt32Set(TIM2, u32ArrData);
                        Bt_Run(TIM2);
                    }
                }
            }
            else if (token == 23)
            {
                while (1)
                {
                    if (My_Bt0_flag == 1) //每个波形执行完的中断标志
                    {
                        voiceOnActPeriod = bgnPeriod;
                        u16InitCntData = PeriodTm;
                        Bt_ARRSet(TIM0, u16InitCntData);
                        Bt_Cnt16Set(TIM0, u16InitCntData);
                        Bt_Run(TIM0);
                        Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                        Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                        Adt_StartCount(AdTIM4);
                        Adt_StartCount(AdTIM6);
                        My_Bt0_flag = 0;
                    }
                    if (My_Bt0_1_flag == 1) //每个频率执行完进一次中断
                    {

                        voiceOnActPeriod = voiceOnActPeriod - 20;
                        Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                        Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                        if (voiceOnActPeriod <= finPeriod)
                        {
                            My_Bt0_flag = 1;
                        }
                        My_Bt0_1_flag = 0;
                    }
                    if (My_Bt2_flag == 1)
                    {
                        My_Bt2_flag = 0;
                        Gpio_SetIO(3, 5, FALSE);
                        My_SynAdTIM6();
                        delay100us(2);
                        if (Gpio_GetIO(3, 4) == FALSE)
                        {
                            break;
                        }
                        Gpio_SetIO(3, 5, TRUE);
                        My_Bt0_flag = 1;
                        Bt_Cnt32Set(TIM2, u32ArrData);
                        Bt_Run(TIM2);
                    }
                }
            }
            else if (token == 3 || token == 7 || token == 19 || token == 20 || token == 30) //间断性平波
            {
                while (1)
                {
                    if (My_Bt1_flag == 1)
                    {
                        My_Bt1_flag = 0;
                        if (discretFrqFlag == 1)
                        {
                            u32InitCntData = bgnFrqTm;
                            Bt_Cnt32Set(TIM1, u32InitCntData);
                            voiceOnActPeriod = ((1000000 / bgnFrq) + 0.5);
                            Bt_Run(TIM1);
                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                            Adt_StartCount(AdTIM4);
                            Adt_StartCount(AdTIM6);
                        }
                        else
                        {
                            u32InitCntData = finFrqTm;
                            Bt_Cnt32Set(TIM1, u32InitCntData);
                            Bt_Run(TIM1);
                            Adt_StopCount(AdTIM4);
                            Adt_StartCount(AdTIM6);
                        }
                    }
                    if (My_Bt2_flag == 1)
                    {
                        My_Bt2_flag = 0;
                        Gpio_SetIO(3, 5, FALSE);
                        My_SynAdTIM6();
                        delay100us(2);
                        if (Gpio_GetIO(3, 4) == FALSE)
                        {
                            break;
                        }
                        Gpio_SetIO(3, 5, TRUE);
                        My_Bt1_flag = 1;
                        discretFrqFlag = 1;
                        Bt_Cnt32Set(TIM2, u32ArrData);
                        Bt_Run(TIM2);
                    }
                }
            }
            else if (token == 10 || token == 11) //特殊序列平波
            {
                while (1)
                {
                    if (My_Bt1_flag == 1)
                    {
                        My_Bt1_flag = 0;
                        if (stap == 0 || stap == 2 || stap == 4)
                        {
                            u32InitCntData = bgnFrqTm;
                            Bt_Cnt32Set(TIM1, u32InitCntData);
                            voiceOnActPeriod = ((1000000 / bgnFrq) + 0.5);
                            Bt_Run(TIM1);
                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                            Adt_StartCount(AdTIM4);
                            Adt_StartCount(AdTIM6);
                        }
                        else if (stap == 1 || stap == 3 || stap == 5)
                        {
                            u32InitCntData = finFrqTm;
                            Bt_Cnt32Set(TIM1, u32InitCntData);
                            Bt_Run(TIM1);
                            Adt_StopCount(AdTIM4);

                        }
                        else
                        {
                            u32InitCntData = finFrqTm;
                            Bt_Cnt32Set(TIM1, u32InitCntData);
                            Bt_Run(TIM1);
                            Adt_StopCount(AdTIM4);
                        }
                    }
                    if (My_Bt2_flag == 1)
                    {
                        My_Bt2_flag = 0;
                        Gpio_SetIO(3, 5, FALSE);
                        My_SynAdTIM6();
                        delay100us(2);
                        if (Gpio_GetIO(3, 4) == FALSE)
                        {
                            break;
                        }
                        Gpio_SetIO(3, 5, TRUE);
                        My_Bt1_flag = 1;
                        stap = 0;
                        Bt_Cnt32Set(TIM2, u32ArrData);
                        Bt_Run(TIM2);
                    }
                }
            }
            else if (token == 8 || token == 27) //特殊序列平波
            {
                while (1)
                {
                    if (My_Bt0_flag == 1) //每个波形执行完的中断标志
                    {
                        voiceOnActPeriod = bgnPeriod;
                        u16InitCntData = PeriodTm;
                        Bt_ARRSet(TIM0, u16InitCntData);
                        Bt_Cnt16Set(TIM0, u16InitCntData);
                        Bt_Run(TIM0);
                        Adt_StartCount(AdTIM6);

                        My_Bt0_flag = 0;
                    }
                    if (My_Bt0_1_flag == 1) //每个频率执行完进一次中断
                    {

                        voiceOnActPeriod--;

                        if (stap == 0 || stap == 2 || stap == 4)
                        {

                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                            Adt_StartCount(AdTIM4);
                        }
                        else
                        {
                            Adt_StopCount(AdTIM4);
                        }


                        if (voiceOnActPeriod <= finPeriod)
                        {
                            My_Bt0_flag = 1;
                            stap++;
                            if (stap == 8)
                            {
                                stap = 0;
                            }
                        }
                        My_Bt0_1_flag = 0;
                    }
                    if (My_Bt2_flag == 1)
                    {
                        My_Bt2_flag = 0;
                        Gpio_SetIO(3, 5, FALSE);
                        My_SynAdTIM6();
                        delay100us(2);
                        if (Gpio_GetIO(3, 4) == FALSE)
                        {
                            break;
                        }
                        Gpio_SetIO(3, 5, TRUE);
                        My_Bt0_flag = 1;
                        stap = 0;
                        Bt_Cnt32Set(TIM2, u32ArrData);
                        Bt_Run(TIM2);
                    }
                }
            }
            else
            {
                while (1)
                {
                    if (My_Bt0_flag == 1) //每个波形执行完的中断标志
                    {

                        voiceOnActPeriod = bgnPeriod;
                        if (stap == 0)
                        {
                            u16InitCntData = PeriodTm;
                        }
                        else
                        {
                            u16InitCntData = PeriodTm_OFF;
                        }
                        Bt_ARRSet(TIM0, u16InitCntData);
                        Bt_Cnt16Set(TIM0, u16InitCntData);
                        Bt_Run(TIM0);
                        Adt_StartCount(AdTIM6);
                        My_Bt0_flag = 0;
                    }

                    if (My_Bt0_1_flag == 1) //每个频率执行完进一次中断
                    {

                        voiceOnActPeriod--;
                        if (stap == 0)
                        {

                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                            Adt_StartCount(AdTIM4);
                        }
                        else
                        {
                            Adt_StopCount(AdTIM4);
                        }

                        if (voiceOnActPeriod <= finPeriod) //满足条件说明一个波形执行完了
                        {
                            Adt_StopCount(AdTIM4);
                            My_Bt0_flag = 1;
                            stap = 1 - stap;
                        }
                        My_Bt0_1_flag = 0;
                    }

                    if (My_Bt2_flag == 1)
                    {
                        My_Bt2_flag = 0;
                        Gpio_SetIO(3, 5, FALSE);
                        My_SynAdTIM6();
                        delay100us(2);
                        if (Gpio_GetIO(3, 4) == FALSE)
                        {
                            break;
                        }
                        Gpio_SetIO(3, 5, TRUE);
                        My_Bt0_flag = 1;
                        stap = 0;
                        Bt_Cnt32Set(TIM2, u32ArrData);
                        Bt_Run(TIM2);
                    }
                }
            }

        }
        else
        {
            My_Bt1_flag = 1;
            My_Bt0_flag = 1;
            Bt_Run(TIM2);
            if (token == 0 || token == 13 || token == 16 || token == 17 || token == 18) //平波
            {
                while (1)
                {
                    if (My_Bt1_flag == 1)
                    {
                        voiceOnActPeriod = ((1000000 / bgnFrq) + 0.5);
                        Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                        Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                        Adt_StartCount(AdTIM4);
                        Adt_StartCount(AdTIM6);
                        My_Bt1_flag = 0;
                    }
                    if (My_slave_flag == 1)
                    {
                        My_slave_flag = 0;
                        if (Gpio_GetIO(3, 4) == TRUE)
                        {

                            Adt_StopCount(AdTIM4);
                            Adt_StopCount(AdTIM6);
                            Gpio_DisableIrq(3, 4,  GpioIrqFalling);
                            break;
                        }
                        Bt_Cnt32Set(TIM2, u32ArrData);
                        Bt_Run(TIM2);
                    }

                }

            }
            else if (token == 1 || token == 4 || token == 5 || token == 9 || token == 14 || token == 21 || token == 22) //方波
            {
                while (1)
                {
                    if (My_Bt1_flag == 1)
                    {
                        My_Bt1_flag = 0;
                        Adt_StartCount(AdTIM6);
                        if (discretFrqFlag == 1)
                        {
                            u32InitCntData = bgnFrqTm;
                            Bt_Cnt32Set(TIM1, u32InitCntData);
                            voiceOnActPeriod = ((1000000 / bgnFrq) + 0.5);
                            Bt_Run(TIM1);
                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                            Adt_StartCount(AdTIM4);

                        }
                        else
                        {
                            u32InitCntData = finFrqTm;
                            Bt_Cnt32Set(TIM1, u32InitCntData);
                            voiceOnActPeriod = ((1000000 / finFrq) + 0.5);
                            Bt_Run(TIM1);
                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                            Adt_StartCount(AdTIM4);
                        }
                    }
                    if (My_slave_flag == 1)
                    {
                        My_slave_flag = 0;

                        if (Gpio_GetIO(3, 4) == TRUE)
                        {
                            Bt_Stop(TIM1);
                            Adt_StopCount(AdTIM4);
                            Adt_StopCount(AdTIM6);
                            Gpio_DisableIrq(3, 4,  GpioIrqFalling);
                            break;
                        }
                        Bt_Cnt32Set(TIM2, u32ArrData);
                        Bt_Run(TIM2);
                    }
                }
            }

            else if (token == 2 || token == 12 || token == 15  || token == 25 || token == 26 || token == 28 || token == 29)//连续锯齿波
            {
                while (1)
                {
                    if (My_Bt0_flag == 1) //每个波形执行完的中断标志
                    {
                        if (finFrq > bgnFrq)
                        {
                            voiceOnActPeriod = bgnPeriod;
                            u16InitCntData = PeriodTm;
                            Bt_ARRSet(TIM0, u16InitCntData);
                            Bt_Cnt16Set(TIM0, u16InitCntData);
                            Bt_Run(TIM0);
                            Adt_StopCount(AdTIM4);
                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                            Adt_StartCount(AdTIM4);
                            Adt_StartCount(AdTIM6);
                        }
                        else
                        {
                            voiceOnActPeriod = bgnPeriod;
                            u16InitCntData = PeriodTm;
                            Bt_ARRSet(TIM0, u16InitCntData);
                            Bt_Cnt16Set(TIM0, u16InitCntData);
                            Bt_Run(TIM0);
                            Adt_StopCount(AdTIM4);
                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                            Adt_StartCount(AdTIM4);
                            Adt_StartCount(AdTIM6);
                        }
                        My_Bt0_flag = 0;
                    }
                    if (My_Bt0_1_flag == 1) //每个频率执行完进一次中断
                    {
                        if (finFrq > bgnFrq)
                        {
                            voiceOnActPeriod--;

                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);

                            if (voiceOnActPeriod <= finPeriod)
                            {
                                My_Bt0_flag = 1;
                            }
                        }
                        else
                        {
                            voiceOnActPeriod++;

                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);

                            if (voiceOnActPeriod >= finPeriod)
                            {
                                My_Bt0_flag = 1;
                            }
                        }
                        My_Bt0_1_flag = 0;
                    }
                    if (My_slave_flag == 1)
                    {
                        My_slave_flag = 0;
                        if (Gpio_GetIO(3, 4) == TRUE)
                        {
                            Bt_Stop(TIM1);
                            Bt_Stop(TIM0);
                            Adt_StopCount(AdTIM4);
                            Adt_StopCount(AdTIM6);
                            Gpio_DisableIrq(3, 4,  GpioIrqFalling);
                            break;
                        }
                        Bt_Cnt32Set(TIM2, u32ArrData);
                        Bt_Run(TIM2);
                    }
                }
            }

            else if (token == 24)
            {
                while (1)
                {
                    if (My_Bt0_flag == 1) //每个波形执行完的中断标志
                    {
                        if (doublet == 1)
                        {
                            voiceOnActPeriod = bgnPeriod;
                            u16InitCntData = PeriodTm;
                            Bt_ARRSet(TIM0, u16InitCntData);
                            Bt_Cnt16Set(TIM0, u16InitCntData);
                            Bt_Run(TIM0);
                            Adt_StopCount(AdTIM4);
                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                            Adt_StartCount(AdTIM4);
                            Adt_StartCount(AdTIM6);
                        }
                        else
                        {
                            voiceOnActPeriod = finPeriod;
                            u16InitCntData = PeriodTm;
                            Bt_ARRSet(TIM0, u16InitCntData);
                            Bt_Cnt16Set(TIM0, u16InitCntData);
                            Bt_Run(TIM0);
                            Adt_StopCount(AdTIM4);
                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                            Adt_StartCount(AdTIM4);
                            Adt_StartCount(AdTIM6);
                        }
                        My_Bt0_flag = 0;
                    }
                    if (My_Bt0_1_flag == 1) //每个频率执行完进一次中断
                    {
                        if (doublet == 1)
                        {
                            voiceOnActPeriod--;

                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);

                            if (voiceOnActPeriod <= finPeriod)
                            {
                                My_Bt0_flag = 1;
                                doublet = 0;
                            }
                        }
                        else
                        {
                            voiceOnActPeriod++;

                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);

                            if (voiceOnActPeriod >= bgnPeriod)
                            {
                                My_Bt0_flag = 1;
                                doublet = 1;
                            }
                        }
                        My_Bt0_1_flag = 0;
                    }
                    if (My_slave_flag == 1)
                    {
                        My_slave_flag = 0;
                        if (Gpio_GetIO(3, 4) == TRUE)
                        {
                            Bt_Stop(TIM1);
                            Bt_Stop(TIM0);
                            Adt_StopCount(AdTIM4);
                            Adt_StopCount(AdTIM6);
                            Gpio_DisableIrq(3, 4,  GpioIrqFalling);
                            break;
                        }
                        Bt_Cnt32Set(TIM2, u32ArrData);
                        Bt_Run(TIM2);
                    }

                }
            }
            else if (token == 23)
            {
                while (1)
                {
                    if (My_Bt0_flag == 1) //每个波形执行完的中断标志
                    {
                        voiceOnActPeriod = bgnPeriod;
                        u16InitCntData = PeriodTm;
                        Bt_ARRSet(TIM0, u16InitCntData);
                        Bt_Cnt16Set(TIM0, u16InitCntData);
                        Bt_Run(TIM0);
                        Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                        Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                        Adt_StartCount(AdTIM4);
                        Adt_StartCount(AdTIM6);
                        My_Bt0_flag = 0;
                    }
                    if (My_Bt0_1_flag == 1) //每个频率执行完进一次中断
                    {
                        voiceOnActPeriod = voiceOnActPeriod - 20;
                        Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                        Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                        if (voiceOnActPeriod <= finPeriod)
                        {
                            My_Bt0_flag = 1;
                        }
                        My_Bt0_1_flag = 0;
                    }
                    if (My_slave_flag == 1)
                    {
                        My_slave_flag = 0;
                        if (Gpio_GetIO(3, 4) == TRUE)
                        {
                            Bt_Stop(TIM1);
                            Bt_Stop(TIM0);
                            Adt_StopCount(AdTIM4);
                            Adt_StopCount(AdTIM6);
                            Gpio_DisableIrq(3, 4,  GpioIrqFalling);
                            break;
                        }
                        Bt_Cnt32Set(TIM2, u32ArrData);
                        Bt_Run(TIM2);
                    }
                }
            }
            else if (token == 3 || token == 7 || token == 19 || token == 20 || token == 30) //间断性平波
            {
                while (1)
                {
                    if (My_Bt1_flag == 1)
                    {
                        My_Bt1_flag = 0;
                        if (discretFrqFlag == 1)
                        {
                            u32InitCntData = bgnFrqTm;
                            Bt_Cnt32Set(TIM1, u32InitCntData);
                            voiceOnActPeriod = ((1000000 / bgnFrq) + 0.5);
                            Bt_Run(TIM1);
                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                            Adt_StartCount(AdTIM4);
                            Adt_StartCount(AdTIM6);
                        }
                        else
                        {
                            u32InitCntData = finFrqTm;
                            Bt_Cnt32Set(TIM1, u32InitCntData);
                            Bt_Run(TIM1);
                            Adt_StopCount(AdTIM4);
                            Adt_StartCount(AdTIM6);
                        }
                    }
                    if (My_slave_flag == 1)
                    {
                        My_slave_flag = 0;
                        if (Gpio_GetIO(3, 4) == TRUE)
                        {
                            Bt_Stop(TIM1);
                            Bt_Stop(TIM0);
                            Adt_StopCount(AdTIM4);
                            Adt_StopCount(AdTIM6);
                            Gpio_DisableIrq(3, 4,  GpioIrqFalling);
                            break;
                        }
                        Bt_Cnt32Set(TIM2, u32ArrData);
                        Bt_Run(TIM2);
                    }
                }
            }
            else if (token == 10 || token == 11) //特殊序列平波
            {
                while (1)
                {

                    if (My_Bt1_flag == 1)
                    {
                        My_Bt1_flag = 0;
                        if (stap == 0 || stap == 2 || stap == 4)
                        {
                            u32InitCntData = bgnFrqTm;
                            Bt_Cnt32Set(TIM1, u32InitCntData);
                            voiceOnActPeriod = ((1000000 / bgnFrq) + 0.5);
                            Bt_Run(TIM1);
                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                            Adt_StartCount(AdTIM4);
                            Adt_StartCount(AdTIM6);
                        }
                        else if (stap == 1 || stap == 3 || stap == 5)
                        {
                            u32InitCntData = finFrqTm;
                            Bt_Cnt32Set(TIM1, u32InitCntData);
                            Bt_Run(TIM1);
                            Adt_StopCount(AdTIM4);

                        }
                        else
                        {
                            u32InitCntData = finFrqTm;
                            Bt_Cnt32Set(TIM1, u32InitCntData);
                            Bt_Run(TIM1);
                            Adt_StopCount(AdTIM4);
                        }
                    }
                    if (My_slave_flag == 1)
                    {
                        My_slave_flag = 0;
                        if (Gpio_GetIO(3, 4) == TRUE)
                        {
                            Bt_Stop(TIM1);
                            Bt_Stop(TIM0);
                            Adt_StopCount(AdTIM4);
                            Adt_StopCount(AdTIM6);
                            Gpio_DisableIrq(3, 4,  GpioIrqFalling);
                            break;
                        }
                        Bt_Cnt32Set(TIM2, u32ArrData);
                        Bt_Run(TIM2);
                    }
                }
            }
            else if (token == 8 || token == 27) //特殊序列平波
            {
                while (1)
                {
                    if (My_Bt0_flag == 1) //每个波形执行完的中断标志
                    {
                        voiceOnActPeriod = bgnPeriod;
                        u16InitCntData = PeriodTm;
                        Bt_ARRSet(TIM0, u16InitCntData);
                        Bt_Cnt16Set(TIM0, u16InitCntData);
                        Bt_Run(TIM0);
                        Adt_StartCount(AdTIM6);

                        My_Bt0_flag = 0;
                    }
                    if (My_Bt0_1_flag == 1) //每个频率执行完进一次中断
                    {

                        voiceOnActPeriod--;
                        if (stap == 0 || stap == 2 || stap == 4)
                        {

                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                            Adt_StartCount(AdTIM4);
                        }

                        else
                        {
                            Adt_StopCount(AdTIM4);
                        }

                        if (voiceOnActPeriod <= finPeriod)
                        {
                            My_Bt0_flag = 1;
                            stap++;
                            if (stap == 8)
                            {
                                stap = 0;
                            }
                        }
                        My_Bt0_1_flag = 0;
                    }
                    if (My_slave_flag == 1)
                    {
                        My_slave_flag = 0;
                        if (Gpio_GetIO(3, 4) == TRUE)
                        {
                            Bt_Stop(TIM1);
                            Bt_Stop(TIM0);
                            Adt_StopCount(AdTIM4);
                            Adt_StopCount(AdTIM6);
                            Gpio_DisableIrq(3, 4,  GpioIrqFalling);
                            break;
                        }
                        Bt_Cnt32Set(TIM2, u32ArrData);
                        Bt_Run(TIM2);
                    }
                }
            }
            else
            {
                while (1)
                {
                    if (My_Bt0_flag == 1) //每个波形执行完的中断标志
                    {

                        voiceOnActPeriod = bgnPeriod;
                        if (stap == 0)
                        {
                            u16InitCntData = PeriodTm;
                        }
                        else
                        {
                            u16InitCntData = PeriodTm_OFF;
                        }
                        Bt_ARRSet(TIM0, u16InitCntData);
                        Bt_Cnt16Set(TIM0, u16InitCntData);
                        Bt_Run(TIM0);

                        Adt_StartCount(AdTIM6);

                        My_Bt0_flag = 0;
                    }

                    if (My_Bt0_1_flag == 1) //每个频率执行完进一次中断
                    {

                        voiceOnActPeriod--;
                        if (stap == 0)
                        {

                            Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
                            Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActPeriod / 2);
                            Adt_StartCount(AdTIM4);
                        }
                        else
                        {
                            Adt_StopCount(AdTIM4);
                        }

                        if (voiceOnActPeriod <= finPeriod) //满足条件说明一个波形执行完了
                        {
                            My_Bt0_flag = 1;
                            stap = 1 - stap;
                        }
                        My_Bt0_1_flag = 0;
                    }

                    if (My_slave_flag == 1)
                    {
                        My_slave_flag = 0;
                        if (Gpio_GetIO(3, 4) == TRUE)
                        {
                            Bt_Stop(TIM1);
                            Bt_Stop(TIM0);
                            Adt_StopCount(AdTIM4);
                            Adt_StopCount(AdTIM6);
                            Gpio_DisableIrq(3, 4,  GpioIrqFalling);
                            break;
                        }
                        Bt_Cnt32Set(TIM2, u32ArrData);
                        Bt_Run(TIM2);
                    }
                }
            }

        }
    }
}

void My_InitAdTIM6(uint16_t beacon_flash_count, uint16_t beacon_flash_duty_count)
{
    stc_adt_CHxX_port_cfg_t pstcAdtCHxXCfg;
    stc_adt_basecnt_cfg_t stcAdtBaseCntCfg;
    stcAdtBaseCntCfg.enCntMode = AdtSawtoothMode;
    stcAdtBaseCntCfg.enCntDir = AdtCntUp;
    stcAdtBaseCntCfg.enCntClkDiv = AdtClkPClk0Div256;
    Adt_Init(AdTIM6, &stcAdtBaseCntCfg);
    //Adt_SetCount(AdTIM6,0);
    Adt_SetPeriod(AdTIM6, beacon_flash_count);
    Adt_SetCompareValue(AdTIM6, AdtCompareA, beacon_flash_duty_count);
    Adt_ClearIrqFlag(AdTIM6, AdtOVFIrq);
    EnableNvic(TIM6_IRQn, 3u, TRUE);
    pstcAdtCHxXCfg.enCap    =   AdtCHxCompareOutput;
    pstcAdtCHxXCfg.bOutEn =   TRUE;
    pstcAdtCHxXCfg.enPerc =   AdtCHxPeriodHigh;//AdtCHxPeriodHigh;
    pstcAdtCHxXCfg.enCmpc =   AdtCHxCompareInv;
    pstcAdtCHxXCfg.enStaOut =   AdtCHxPortOutHigh;//AdtCHxPortOutHigh;
    pstcAdtCHxXCfg.enStpOut =   AdtCHxPortOutHigh;//AdtCHxPortOutHigh;
    pstcAdtCHxXCfg.enStaStp   =   AdtCHxStateSelSS;
    Adt_CHxXPortConfig(AdTIM6, AdtCHxA, &pstcAdtCHxXCfg);

}

void My_SynAdTIM6(void)
{
    stc_adt_basecnt_cfg_t stcAdtBaseCntCfg;
    stc_adt_CHxX_port_cfg_t pstcAdtCHxXCfg;

    Adt_StopCount(AdTIM6);

    stcAdtBaseCntCfg.enCntMode = AdtSawtoothMode;                           //???A??
    stcAdtBaseCntCfg.enCntDir = AdtCntUp;                                           //????
    stcAdtBaseCntCfg.enCntClkDiv = AdtClkPClk0Div256;                   //Pclk/256

    Adt_DeInit(AdTIM6);
    Adt_Init(AdTIM6, &stcAdtBaseCntCfg);                                              //ADT????????????

    Adt_SetCount(AdTIM6, 0);                                                                      //?0??
//      Adt_SetPeriodBuf(AdTIM6,beacon_flash_count);                        //????????

//      Adt_SetCompareValue(AdTIM6,AdtCompareD,beacon_flash_duty_count);        //???????D

//      Adt_EnableValueBuf(AdTIM6,AdtCHxB,TRUE);                                    //???????D->B??

    Adt_ClearIrqFlag(AdTIM6, AdtOVFIrq);                                            //AdTIM6???????,?????0

    EnableNvic(TIM6_IRQn, 3u, TRUE);                                                    //?????3

    pstcAdtCHxXCfg.enCap    =   AdtCHxCompareOutput;                        //????
    pstcAdtCHxXCfg.bOutEn =   TRUE;                                                       //????
    pstcAdtCHxXCfg.enPerc =   AdtCHxPeriodHigh;                               //????????
    pstcAdtCHxXCfg.enCmpc =   AdtCHxCompareInv;                               //?????????
    pstcAdtCHxXCfg.enStaOut =   AdtCHxPortOutHigh;                          //??????????
    pstcAdtCHxXCfg.enStpOut =   AdtCHxPortOutHigh;                          //??????????
    pstcAdtCHxXCfg.enStaStp   =   AdtCHxStateSelSS;                           //?????????????

    Adt_CHxXPortConfig(AdTIM6, AdtCHxA, &pstcAdtCHxXCfg);


    Adt_StartCount(AdTIM6);
}
void My_InitClk(void)
{
    stc_clk_config_t stcCfg;


    DDL_ZERO_STRUCT(stcCfg);                                                    //clear stcCfg
    stcCfg.enClkSrc = ClkRCH;                                                   // RCH  = 4M
    stcCfg.enHClkDiv = ClkDiv1;                                             // HCLK = 4M
    stcCfg.enPClkDiv = ClkDiv1;                                             // PCLK = 4M

    Clk_Init(&stcCfg);

    Clk_SwitchTo(ClkRCH);

    Clk_SetPeripheralGate(ClkPeripheralGpio, TRUE);     //GPIO CLK enable
    Clk_SetPeripheralGate(ClkPeripheralAdt, TRUE);      //ADT CLK enable
    Clk_SetPeripheralGate(ClkPeripheralWdt, TRUE);      //WDT CLK enable
    Clk_SetPeripheralGate(ClkPeripheralBt, TRUE);           //BT CLK enable
}
void key_check(void)
{
    key1 = !Gpio_GetIO(3, 3) * 16;
    key2 = !Gpio_GetIO(3, 2) * 8;
    key3 = !Gpio_GetIO(2, 6) * 4;
    key4 = !Gpio_GetIO(2, 5) * 2;
    key5 = !Gpio_GetIO(2, 4) * 1;
    key6 = !Gpio_GetIO(2, 3) * 1;
    key7 = !Gpio_GetIO(1, 4) * 1;
    ledin = Gpio_GetIO(3, 4) * 1;
}
void My_InitBt0(uint16_t  u16InitCntData)
{
    stc_bt_config_t   stcConfig;
    en_result_t       enResult = Error;
    //uint16_t          u16ArrData = 0x0000;

    EnableNvic(TIM0_IRQn, 3, TRUE);
    Bt_EnableIrq(TIM0);

    stcConfig.pfnTim0Cb = My_Tim0_IRQ;

    stcConfig.enGateP = BtPositive;
    stcConfig.enGate  = BtGateDisable;
    stcConfig.enPRS   = BtPCLKDiv4;
    stcConfig.enTog   = BtTogDisable;
    stcConfig.enCT    = BtTimer;
    stcConfig.enMD    = BtMode2;
    if (Ok != Bt_Init(TIM0, &stcConfig))
    {
        enResult = Error;
    }
    Bt_ARRSet(TIM0, u16InitCntData);
    Bt_Cnt16Set(TIM0, u16InitCntData);
    //Bt_Run(TIM0);

}
void My_Tim0_IRQ(void)
{
    if (TRUE == Bt_GetIntFlag(TIM0))
    {
        My_SounderPhaseEndWaiting();
        Bt_ClearIntFlag(TIM0);

        My_Bt0_1_flag = 1;


    }
}
void My_InitAdTIM4(uint16_t  voiceOnActPeriod)
{
    stc_adt_basecnt_cfg_t stcAdtBaseCntCfg;
    stc_adt_CHxX_port_cfg_t stcAdtTIM4BCfg;
    Adt_StopCount(AdTIM4);
    Adt_ClearCount(AdTIM4);

    stcAdtBaseCntCfg.enCntMode = AdtSawtoothMode;
    stcAdtBaseCntCfg.enCntDir = AdtCntUp;
    stcAdtBaseCntCfg.enCntClkDiv = AdtClkPClk0Div4;
    Adt_Init(AdTIM4, &stcAdtBaseCntCfg);
    Adt_ClearIrqFlag(AdTIM4, AdtOVFIrq);                                            //AdTIM4???????,?????0
    Adt_ConfigIrq(AdTIM4, AdtOVFIrq, TRUE, My_AdTim4_IRQ);          //AdTIM4????????
    stcAdtTIM4BCfg.enCap = AdtCHxCompareOutput;
    stcAdtTIM4BCfg.bOutEn = TRUE;
    stcAdtTIM4BCfg.enPerc = AdtCHxPeriodInv;
    stcAdtTIM4BCfg.enCmpc = AdtCHxCompareLow;
    stcAdtTIM4BCfg.enStaStp = AdtCHxStateSelSS;
    stcAdtTIM4BCfg.enStaOut = AdtCHxPortOutHigh;
    stcAdtTIM4BCfg.enStpOut = AdtCHxPortOutLow;
    Adt_CHxXPortConfig(AdTIM4, AdtCHxB, &stcAdtTIM4BCfg);
    Adt_SetPeriod(AdTIM4, voiceOnActPeriod);
    Adt_SetCompareValue(AdTIM4, AdtCompareB, voiceOnActCompare);
}

void My_InitBt1(uint32_t  u32InitCntData)
{
    stc_bt_config_t   stcConfig;
    en_result_t       enResult = Error;


    EnableNvic(TIM1_IRQn, 3, TRUE);
    Bt_EnableIrq(TIM1);
    Bt_ClearIntFlag(TIM1);
    stcConfig.pfnTim1Cb = My_Tim1_IRQ;

    stcConfig.enGateP = BtPositive;
    stcConfig.enGate  = BtGateDisable;
    stcConfig.enPRS   = BtPCLKDiv4;
    stcConfig.enTog   = BtTogDisable;
    stcConfig.enCT    = BtTimer;
    stcConfig.enMD    = BtMode1;
    if (Ok != Bt_Init(TIM1, &stcConfig))
    {
        enResult = Error;
    }

    Bt_Cnt32Set(TIM1, u32InitCntData);
    //Bt_Run(TIM1);

}
void My_Tim1_IRQ(void)
{
    if (TRUE == Bt_GetIntFlag(TIM1))
    {
        My_SounderPhaseEndWaiting();
        Bt_ClearIntFlag(TIM1);
        My_Bt1_flag = 1;
        discretFrqFlag = 1 - discretFrqFlag;
        stap++;
        Bt_Stop(TIM1);
        if (stap == 8)
        {
            stap = 0;
        }


    }
}
void My_InitBt2(uint32_t u32ArrData)
{
    stc_bt_config_t   stcConfig;
    en_result_t       enResult = Error;


    EnableNvic(TIM2_IRQn, 2, TRUE);
    Bt_EnableIrq(TIM2);
    Bt_ClearIntFlag(TIM2);
    stcConfig.pfnTim2Cb = My_Tim2_IRQ;

    stcConfig.enGateP = BtPositive;
    stcConfig.enGate  = BtGateDisable;
    stcConfig.enPRS   = BtPCLKDiv4;
    stcConfig.enTog   = BtTogDisable;
    stcConfig.enCT    = BtTimer;
    stcConfig.enMD    = BtMode1;
    if (Ok != Bt_Init(TIM2, &stcConfig))
    {
        enResult = Error;
    }

    Bt_Cnt32Set(TIM2, u32ArrData);
    //Bt_Run(TIM1);

}
void My_Tim2_IRQ(void)
{
    if (TRUE == Bt_GetIntFlag(TIM2))
    {
        Bt_ClearIntFlag(TIM2);
        Bt_Stop(TIM2);
        if (signal == damaster)
        {
            My_Bt2_flag = 1;
            Bt_Stop(TIM1);
            Bt_Stop(TIM0);
            //Adt_StopCount(AdTIM4);
            Adt_StopCount(AdTIM6);
        }
        else
        {
            My_slave_flag = 1;
        }
    }
}
void My_SounderPhaseEndWaiting(void)
{
    boolean_t   adtim4_run_status, adtim4_ovff_flag  =   FALSE;

    adtim4_run_status   =   Adt_Start_Status(AdTIM4);

    if (1 == adtim4_run_status)
    {
        while (1 != adtim4_ovff_flag)
            adtim4_ovff_flag    =   Adt_GetOvffIntFlag(AdTIM4);
    }
}

void My_AdTim4_IRQ(void)
{
    Adt_ClearIrqFlag(AdTIM4, AdtOVFIrq);

//      goto LOOP;
}


