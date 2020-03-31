﻿/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.C
* Author             : WCH
* Version            : V1.1
* Date               : 2017/07/05
* Description        : CH554 触摸按键中断和查询方式进行采集并报告当前采样通道按键状态，包含初始化和按键采样等演示函数
*******************************************************************************/
#include <stdint.h>
#include <stdio.h>

#include <ch554.h>
#include <debug.h>
#include <touchkey.h>

void main()
{
    uint8_t i;
    CfgFsys( );                                                                //CH554时钟选择配置 - CH554 clock selection configuration
    mDelaymS(5);                                                               //修改主频建议稍加延时等待芯片供电稳定 - Modify the clock frequency
    mInitSTDIO( );                                                             //串口0初始化 - Serial port 0 initialization
    UART1Setup();

    printf("start ...\n");

    P1_DIR_PU &= 0x0C;                                                         //所有触摸通道设置为浮空输入，用不到的通道可以不设置 - All touch channels are set as floating input, the unused channels can be left unset
    TouchKeyQueryCyl2ms();                                                     //TouchKey查询周期2ms - TouchKey query cycle 2ms
    GetTouchKeyFree();                                                         //获取采样基准值 - Get the sampling reference value
	
#if DE_PRINTF
    for(i=KEY_FIRST;i<(KEY_LAST+1);i++)                                        //打印采样基准值 - Print the sampling reference value
    {
        printf("Channel %02x base sample %04x\n",(uint16_t)i,KeyFree[i]);
    }
#endif

#if INTERRUPT_TouchKey
    EA = 1;
    while(1)
    {
        if(KeyBuf)                                                               //key_buf非0，表示检测到按键按下 - key_buf is non-zero, indicating that a key press was detected
        {
            printf("INT TouchKey Channel %02x \n",(uint16_t)KeyBuf);                 //打印当前按键状态通道 - Print the current key state channel
            KeyBuf	= 0;                                                           //清除按键按下标志 - Clear the key press flag
            mDelaymS(100);                                                         //延时无意义，模拟单片机做按键处理 - delay is meaningless, imitate single chip computer to do key processing
        }
        mDelaymS(100);                                                           //延时无意义，模拟单片机干其他事
    }
#else
    while(1)
    {
        TouchKeyChannelQuery();                                                  //查询触摸按键状态 - Query touch key status
        if(KeyBuf)                                                               //key_buf非0，表示检测到按键按下 - key_buf is non-zero, indicating that a key press was detected
        {
            printf("Query TouchKey Channel %02x \n",(uint16_t)KeyBuf);              //打印当前按键状态通道 - Print the current key state channel
            KeyBuf = 0;                                                           //清除按键按下标志 - Clear the key press flag
            mDelaymS(20);                                                         //延时无意义，模拟单片机做按键处理 - delay is meaningless, imitate single chip computer to do other things
        }
        //       mDelaymS(100);                                                           //延时无意义，模拟单片机干其他事
    }
#endif
}
