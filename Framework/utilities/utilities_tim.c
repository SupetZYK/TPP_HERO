#include "utilities_tim.h"

#include "peripheral_define.h"
#include "tim.h"

uint64_t timeMicros = 0;
uint64_t fw_getTimeMicros(void){
	return timeMicros + __HAL_TIM_GET_COUNTER(&USER_TIM);
}

void fw_userTimeEnable(void){
	HAL_TIM_Base_Start_IT(&USER_TIM);
	HAL_TIM_PWM_Start(&FRICTION_TIM, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&FRICTION_TIM, TIM_CHANNEL_2);
	
	HAL_TIM_PWM_Start(&BULLET_TIM, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&BULLET_TIM, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&BULLET_TIM, TIM_CHANNEL_3);
	//__HAL_TIM_ENABLE(&USER_TIM);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &USER_TIM){
		timeMicros += 0xFFFF;
	}
}
