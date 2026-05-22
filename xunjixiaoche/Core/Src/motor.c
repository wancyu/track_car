#include "motor.h"

#define PWM_MAX 7200
#define PWM_MIN -7200

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
int abs(int p)
{
	if(p>0)
		return p;
	else
		return -p;
}
void Load(int left_pwm, int right_pwm)
{
	// --- 左电机控制 (moto1 对应左轮) ---
	if(left_pwm >= 0) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	}
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, abs(left_pwm)); // Channel 1 控制左

	// --- 右电机控制 (moto2 对应右轮) ---
	if(right_pwm >= 0) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	}
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, abs(right_pwm)); // Channel 2 控制右
}

void Limit(int *motoA,int *motoB)
{
	if(*motoA>PWM_MAX)*motoA=PWM_MAX;
	if(*motoA<PWM_MIN)*motoA=PWM_MIN;
	if(*motoB>PWM_MAX)*motoB=PWM_MAX;
	if(*motoB<PWM_MIN)*motoB=PWM_MIN;
}