
#define ADC_INTERRUPT 1

/************************************************ ******************************
* Function Name: ADCClkSet (uint8_t div)
* Description: ADC sampling clock setting, module is on, interrupt is on
* Input: uint8_t div clock setting
                   0 Slow 384 Fosc
                   1 fast 96 Fosc
* Output: None
* Return: None
************************************************** *****************************/
void ADCInit (uint8_t div);

/************************************************ ******************************
* Function Name: ADC_ChannelSelect (uint8_t ch)
* Description: ADC sampling channel setting
* Input: uint8_t ch uses channels 0-3
* Output: None
* Return: SUCCESS
                   FAIL channel setting is out of range
************************************************** *****************************/
uint8_t ADC_ChannelSelect (uint8_t ch);

/************************************************ ******************************
* Function Name: VoltageCMPModeInit ()
* Description: Voltage comparator mode initialization
* Input: uint8_t fo forward port 0 \ 1 \ 2 \ 3
                   uint8_t re reverse port 1 \ 3
* Output: None
* Return: SUCCESS
                   FAIL
************************************************** *****************************/
uint8_t VoltageCMPModeInit (uint8_t fo, uint8_t re);
