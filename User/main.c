#include "main.h"
#include "board.h"
#include "math.h"

#define SCAN_ADC_RATE 100 //ms
#define PI 3.1416

uint32_t three_axist_motor_lengh[3] = {1000, 1000, 1000};   //mm
int32_t axist_balance_value[3] = {2000, 2000, 1950};        //adc value
double new_axist_value[3] = {0,};
double old_axist_value[3] = {0,};
double shaking_value[3] = {0,};

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

uint16_t sensor_value[3];
uint16_t vol_value[3];
double axit_value[3];
uint32_t g_sys_time = 0;

uint8_t tx_buff[256] = {0,};

void adc_value_to_3_axist(uint16_t* _adcValue, double* _axistValue, uint8_t _numberValue);
void caculate_shaking_value(double* _old_axist_value, double* _new_axist_value, double* _shaking_value);
void build_tx_data(uint8_t* _tx_buff, double* _shaking_value);

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
	  adc_value_to_3_axist(sensor_value, axit_value, 3);
	  caculate_shaking_value(old_axist_value, new_axist_value, shaking_value);
	  build_tx_data(tx_buff, shaking_value);
	  HAL_UART_Transmit(&rs485_com.uart_module, tx_buff, 26, 100);
  }
}



void adc_value_to_3_axist(uint16_t* _adcValue, double* _axistValue, uint8_t _numberValue){
	for(int i = 0; i < _numberValue; i ++){
		_axistValue[i] = (float)((int)_adcValue[i] - axist_balance_value[i])/400*90;
		old_axist_value[i] = new_axist_value[i];
		new_axist_value[i] = _axistValue[i];
	}
}

void caculate_shaking_value(double* _old_axist_value, double* _new_axist_value, double* _shaking_value){
	for(int i = 0; i < 3; i++){

		double new_angle;
		double old_angle;

		if(i == 2){
			 new_angle = (_new_axist_value[i] - 90)*PI/ 180;
		 	 old_angle = (old_axist_value[i] - 90)*PI/ 180;
		}else{
			 new_angle = _new_axist_value[i]*PI/ 180;
			 old_angle = old_axist_value[i]*PI/ 180;
		}



		if((_new_axist_value[i]*old_axist_value[i]) >= 0){

			_shaking_value[i] = three_axist_motor_lengh[i]*fabs(cos(new_angle) - cos(old_angle)) / SCAN_ADC_RATE ;
		}else{
			_shaking_value[i] = three_axist_motor_lengh[i]*(2 - cos(old_angle) - cos(new_angle)) / SCAN_ADC_RATE ;
		}

	}
}

void build_tx_data(uint8_t* _tx_buff, double* _shaking_value){
	_tx_buff[0] = 'X';
	_tx_buff[1] = ':';
	lkv_lcd_float_to_string(&_shaking_value[0], &_tx_buff[2]);
	_tx_buff[7] = '\n';

	_tx_buff[8] = 'Y';
	_tx_buff[9] = ':';
	lkv_lcd_float_to_string(&_shaking_value[1], &_tx_buff[10]);
	_tx_buff[15] = '\n';

	_tx_buff[16] = 'Z';
	_tx_buff[17] = ':';
	lkv_lcd_float_to_string(&_shaking_value[2], &_tx_buff[18]);
	_tx_buff[23] = '\n';
	_tx_buff[24] = '\n';
	_tx_buff[25] = '\0';

}


void USART1_IRQHandler(void){
	HAL_UART_IRQHandler(&rs485_com.uart_module);
	HAL_UART_Receive_IT(&rs485_com.uart_module, &rs485_com.rx_data, 1);
}

