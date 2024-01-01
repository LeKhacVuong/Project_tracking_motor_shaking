#include "main.h"
#include "board.h"

int32_t axist_balance_value[3] = {2000, 2000, 1950};
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

uint16_t sensor_value[3];
uint16_t vol_value[3];
float axit_value[3];
uint32_t g_sys_time = 0;



void adc_value_to_vol(uint16_t *_adcValue, uint16_t *_volValue, uint8_t _numberValue);
void adc_value_to_3_axist(uint16_t* _adcValue, float* _axistValue, uint8_t _numberValue);

int main(void)
{
	board_init();
  while (1)
  {
  }
}


void SysTick_Handler(void)
{
  HAL_IncTick();
  g_sys_time++;
  adc_value_to_3_axist(sensor_value,axit_value,3);

}


void adc_value_to_vol(uint16_t *_adcValue, uint16_t *_volValue, uint8_t _numberValue){
	for(int i = 0; i < _numberValue; i ++){
		_volValue[i] = 3300*_adcValue[i]/4095;
	}
}

void adc_value_to_3_axist(uint16_t* _adcValue, float* _axistValue, uint8_t _numberValue){
	for(int i = 0; i < _numberValue; i ++){
		_axistValue[i] = (float)((int)_adcValue[i] - axist_balance_value[i])/400;
	}
}



