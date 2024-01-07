#include "main.h"
#include "board.h"
#include "math.h"

#define SCAN_ADC_RATE 10 //ms

uint32_t three_axist_motor_lengh[3] = {10000, 10000, 10000};   //mm
int32_t axist_balance_value[3] = {2000, 2000, 1950};        //adc value
float new_axist_value[3] = {0,};
float old_axist_value[3] = {0,};
float shaking_value[3] = {0,};

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

uint16_t sensor_value[3];
uint16_t vol_value[3];
float axit_value[3];
uint32_t g_sys_time = 0;

void adc_value_to_3_axist(uint16_t* _adcValue, float* _axistValue, uint8_t _numberValue);
void caculate_shaking_value(float* _old_axist_value, float* _new_axist_value, float* _shaking_value);

int main(void)
{
	board_init();
  while (1)
  {
	  ///TODO: Thích hiển thị dữ liệu gì thì làm trong này
  }
}


void SysTick_Handler(void)
{
  HAL_IncTick();
  g_sys_time++;
  if(g_sys_time % SCAN_ADC_RATE == 0){
	  adc_value_to_3_axist(sensor_value, axit_value,3);
	  caculate_shaking_value(old_axist_value, new_axist_value, shaking_value);
  }
}



void adc_value_to_3_axist(uint16_t* _adcValue, float* _axistValue, uint8_t _numberValue){
	for(int i = 0; i < _numberValue; i ++){
		_axistValue[i] = (float)((int)_adcValue[i] - axist_balance_value[i])/400*90;
		old_axist_value[i] = new_axist_value[i];
		new_axist_value[i] = _axistValue[i];
	}
}

void caculate_shaking_value(float* _old_axist_value, float* _new_axist_value, float* _shaking_value){
	for(int i = 0; i < 3; i++){
		_shaking_value[i] = three_axist_motor_lengh[i]*(1 - cos(_new_axist_value[i]-old_axist_value[i])) / SCAN_ADC_RATE ;
	}
}


