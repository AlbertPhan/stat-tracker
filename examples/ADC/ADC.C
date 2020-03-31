/* ********************************* (C) COPYRIGHT ******** ********************
* File Name: ADC.C
* Author: WCH
* Version: V1.0
* Date: 2017/01/20
* Description: CH554 ADC sampling clock setting, ADC channel setting function, voltage comparison mode setting
************************************************** *****************************/

#include <ch554.h>
#include <debug.h>
#include "ADC.H"
#include "stdio.h"

#define ADC_INTERRUPT 0

/************************************************ ******************************
* Function Name: ADCInit (uint8_t div)
* Description: ADC sampling clock setting, module is on, interrupt is on
* Input: uint8_t div clock setting
                   1 Slow 384 Fosc
                   0 Fast 96 Foscs
* Output: None
* Return: None
************************************************** *****************************/
void ADCInit (uint8_t div)
{
    ADC_CFG & = ~ bADC_CLK | div; // fast or slow ADC
    ADC_CFG | = bADC_EN; // ADC power enable
#if ADC_INTERRUPT
    ADC_IF = 0; // clear interrupt
    IE_ADC = 1; // Enable ADC interrupt
#endif
}

/************************************************ ******************************
* Function Name: ADC_ChannelSelect (uint8_t ch)
* Description: ADC sampling is enabled
* Input: uint8_t ch uses channels
* Output: None
* Return: SUCCESS
                   FAIL
************************************************** *****************************/
uint8_t ADC_ChannelSelect (uint8_t ch)
{
    if (ch == 0) {ADC_CHAN1 = 0; ADC_CHAN0 = 0; P1_DIR_PU & = ~ bAIN0;} // AIN0
    else if (ch == 1) {ADC_CHAN1 = 0; ADC_CHAN0 = 1; P1_DIR_PU & = ~ bAIN1;} // AIN1
    else if (ch == 2) {ADC_CHAN1 = 1; ADC_CHAN0 = 0; P1_DIR_PU & = ~ bAIN2;} // AIN2
    else if (ch == 3) {ADC_CHAN1 = 1; ADC_CHAN0 = 1; P3_DIR_PU & = ~ bAIN3;} // AIN3
    else return FAIL;
    return SUCCESS;
}

/************************************************ ******************************
* Function Name: VoltageCMPModeInit ()
* Description: Voltage comparator mode initialization
* Input: uint8_t fo forward port 0 \ 1 \ 2 \ 3 (non inverting input)
                   uint8_t
                 re reverse port 1 \ 3 (inverting input)
* Output: None
* Return: SUCCESS
                   FAIL
************************************************** *****************************/
uint8_t VoltageCMPModeInit (uint8_t fo, uint8_t re)
{
    ADC_CFG | = bCMP_EN; // level comparison power supply enable
    if (re == 1) {
      if (fo == 0) {ADC_CHAN1 = 0; ADC_CHAN0 = 0; CMP_CHAN = 0;} // AIN0 and AIN1
      else if (fo == 2) {ADC_CHAN1 = 1; ADC_CHAN0 = 0; CMP_CHAN = 0;} // AIN2 and AIN1
      else if (fo == 3) {ADC_CHAN1 = 1; ADC_CHAN0 = 1; CMP_CHAN = 0;} // AIN3 and AIN1
      else return FAIL;
    }
    else if (re == 3) {
      if (fo == 0) {ADC_CHAN1 = 0; ADC_CHAN0 = 0; CMP_CHAN = 0;} // AIN0 and AIN1
      else if (fo == 1) {ADC_CHAN1 = 0; ADC_CHAN0 = 1; CMP_CHAN = 0;} // AIN1 and AIN1
      else if (fo == 2) {ADC_CHAN1 = 1; ADC_CHAN0 = 0; CMP_CHAN = 0;} // AIN2 and AIN1
      else return FAIL;
    }
    else return FAIL;
#if ADC_INTERRUPT
    CMP_IF = 0; // clear interrupt
    IE_ADC = 1; // Enable ADC interrupt
#endif

     return SUCCESS;
}

#if ADC_INTERRUPT
/************************************************ ******************************
* Function Name: ADCInterrupt (void)
* Description: ADC interrupt service routine
************************************************** *****************************/
void ADCInterrupt (void) interrupt INT_NO_ADC using 1 // ADC interrupt service routine, using register group 1
{
    if (ADC_IF == 1) // ADC completed interrupt
    {
// UserData = ADC_DATA; // Remove ADC sampling data
      ADC_IF = 0; // Clear ADC interrupt flag
    }
    if (CMP_IF == 1) // Voltage comparison completed interrupt
    {
// UserData = ADC_CTRL & 0x80 >> 7); // Save the comparator result
      CMP_IF = 0; // Clear the comparator completion interrupt
    }
}
#endif