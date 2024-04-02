/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct gesture   
{
	float pulse; //脉冲数，相当于位移
	float speed; //电机旋转速度
	float voltage; //电压，模拟量
	float angle; //倒立摆角度
	float angle_last;
};
struct pidstruct  //pid参数
{
	double kp,ki,kd;
	double p,i,d;
	double thisde,lastde;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t rx_buf1[1] = {0};
uint8_t rx_buf3[1] = {0};
uint16_t ADC_Value=0;
float angle_out=0;
float speed_out=0;
float position_out=0;
float angle_target=5.3;//5.3
float speed_target=0;
float position_target=0;

int mode=0;//0
int mission=0;//0

int pwm_out=0;
float cal_temp=0;
uint8_t Mission6_FLAG=1;

struct gesture gest;//当前姿态
struct pidstruct pidspeed;//速度环//逆时针为正
struct pidstruct pidangle;//角度环
struct pidstruct pidposition;//角度环
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DataGet1(uint8_t data)//调试
{
	
}

uint8_t reverse=0;
void DataGet3(uint8_t data)//蓝牙或串口屏
{
	
	if(data=='0')
	{
		mode=0;
		Mission6_FLAG=1;
		mission=0;
		pidspeed.p=0;
	  pidspeed.i=0;
	  pidspeed.d=0;
	  pidspeed.lastde=0;
	  pidspeed.thisde=0;
	}
	if(data=='1') mission=1;
	if(data=='2') mission=2;
	if(data=='3') mission=3;
	if(data=='4') mission=4;
	if(data=='5') mission=5;
	if(data=='6') mission=6;
	if(data=='s') mode=1;
//	if(data=='4') mode=4;
//	if(data=='r') reverse=!reverse;
	
}
void Wheel(int pwm)//轮子调速
{
	int direct;
	
	if(pwm>1000)
		pwm=1000;
	if(pwm<-1000)
		pwm=-1000;
	if(pwm>0)
		direct=1;
	if(pwm<0)
		direct=0;
	switch(direct){
	case 1:
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,1000-pwm);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,1000);
		break;
	case 0 :
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,1000);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,1000+pwm);
		break;
	}	
}
struct pidstruct initPID(float kp,float ki,float kd)//pid初始化
{
	struct pidstruct e;
	e.kp=kp;
	e.ki=ki;
	e.kd=kd;
	e.p=0;
	e.i=0;
	e.d=0;
	e.lastde=0;
	e.thisde=0;
	return e;
}
float PID(struct pidstruct *e,float err,float outlow,float outhigh,float a,float int_k)//计算输出
{
	float out;
	e->thisde = err*(1-a) + e->lastde*a;
	e->p = e->kp * e->thisde;
	e->i = e->i + e->ki * e->thisde;
	e->i *= int_k;
	e->d = e->kd * (e->thisde - e->lastde);
	e->lastde = e->thisde;
	out = e->p + e->i + e->d;
	if(e->i > outhigh) e->i=outhigh;
	if(e->i < outlow) e->i=outlow;
	if(out>outhigh) out=outhigh;
	if(out<outlow) out=outlow;
	return out;
}
void Read_Encoder(void)//读取计数器的值
{
		static int catch1[3];
    catch1[1]=__HAL_TIM_GET_COUNTER(&htim1);
    catch1[2]=catch1[1]-catch1[0];
	  catch1[0]=catch1[1];
    gest.pulse=0.025f*(32768-catch1[1]);
    gest.speed=-catch1[2]*0.025f;
}
void ADC_Cal()
{
	gest.angle_last = gest.angle;
	gest.voltage=ADC_Value;
	gest.angle=(ADC_Value-1470)*0.087912f-180;//左正右负
	if(gest.angle<-180)
		gest.angle=gest.angle+360;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  //串口中断
{
	if(huart == &huart1)
	{
		DataGet1(rx_buf1[0]);
		HAL_UART_AbortReceive_IT(&huart1);
		HAL_UART_Receive_IT(&huart1, rx_buf1, 1);
	}
	if(huart == &huart3)
	{
		DataGet3(rx_buf3[0]);
		HAL_UART_AbortReceive_IT(&huart3);
		HAL_UART_Receive_IT(&huart3, rx_buf3, 1);
	}
}

float int_k=1;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//定时器中段
{
	static int k=0,k1=0,delay_count=0,delay_flag=1,current_x=0;
	k1++;
	if(k1>=10)
	  Read_Encoder(),k1=0;	
//	if(k1>=50&&(cal_temp>0.5||cal_temp<-0.5))
//		k1=0,pidspeed.i/=1.5;		//1.58
//	else if(k1>=50&&(cal_temp>0.3||cal_temp<-0.3))
//		k1=0,pidspeed.i/=1.3;		//1.2
//	else if(k1>=50&&cal_temp<0.2&&cal_temp>-0.2)
//		k1=0,pidspeed.i/=1.1;
//	else if(k1>=50&&cal_temp<0.1&&cal_temp>-0.1)
//		k1=0;
	
	if(mode==0)//停止运动
	{
		k=0;
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,1000);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,1000); 
	}
	else if(mode==1)//倒立
	{
		if(gest.angle<(angle_target+30) && gest.angle>(angle_target-30))
		{
			k++;
			if(k==10)
			{
				//Read_Encoder();
				speed_out =PID(&pidspeed,speed_target-gest.speed,-15,15,0.7,int_k);	//0.94
				k=0;
			}
			angle_out = PID(&pidangle,angle_target-speed_out-gest.angle,-900,900,0,1);
			Wheel(angle_out);
		}
		else
		{
			k=0;
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,1000);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,1000); 
		}	
	}
	else if(mode==2)//起摆
	{
		if(gest.angle>165 && gest.angle<180 && gest.angle_last>165 && gest.angle_last<180)
		{
			if(gest.angle>gest.angle_last)
				Wheel(-200);
			else if(gest.angle<gest.angle_last)
				Wheel(200);
		}
		else if(gest.angle<-165 && gest.angle>-180 && gest.angle_last<-165 && gest.angle_last>-180)
		{
			if(gest.angle>gest.angle_last)
				Wheel(-200);
			else if(gest.angle<gest.angle_last)
				Wheel(200);
		}
	}
	else if(mode==3)//起摆
	{
		if(gest.angle>165 && gest.angle<180 && gest.angle_last>165 && gest.angle_last<180)
		{
			if(gest.angle>gest.angle_last)
				Wheel(-140);
			else if(gest.angle<gest.angle_last)
				Wheel(140);
		}
		else if(gest.angle<-165 && gest.angle>-180 && gest.angle_last<-165 && gest.angle_last>-180)
		{
			if(gest.angle>gest.angle_last)
				Wheel(-140);
			else if(gest.angle<gest.angle_last)
				Wheel(140);
		}
	}
	if(mission==0)
	{
		delay_count=0,delay_flag=1;
	}
  else if(mission==1)
	{
		if(delay_flag==1)
		{
			Wheel(150);
			delay_count++;
			if(delay_count==500)
				delay_count=0,delay_flag=0,mode=2;
		}
		if(gest.angle>-90 && gest.angle<90)
			mode=0,mission=0;
	}
	else if(mission==2)
	{
		if(delay_flag==1)
		{
			Wheel(150);
			delay_count++;
			if(delay_count==500)
				delay_count=0,delay_flag=0,mode=2;
		}
		if(gest.angle>-10 && gest.angle<10)
			mode=0,mission=0;
	}
	else if(mission==3)
	{
		mode=1;
		mission=0;
	}
	else if(mission==4)
	{
		if(delay_flag==1)
		{
			Wheel(150);
			delay_count++;
			if(delay_count==300)
				delay_count=0,delay_flag=0,mode=3;
		}
		if(gest.angle<(angle_target+15) && gest.angle>(angle_target-15))//20
			mode=1;
		if(gest.angle>(angle_target+90) || gest.angle<(angle_target-90))
			mode=3;
	}
	else if(mission==5)
	{
		if(gest.angle<(angle_target+15) && gest.angle>(angle_target-15))
			mode=1;
		if(gest.angle>(angle_target+90) || gest.angle<(angle_target-90))
			mode=3;
	}
	else if(mission==6)
	{
		if(Mission6_FLAG)
		{
			mode=1,speed_target=0.2;
			current_x=gest.pulse+39,Mission6_FLAG=0;
		}
		if(gest.pulse>=current_x)
		{
			speed_target=0;	
			mission=0,Mission6_FLAG=1;
		}
		
	}
}
	
//	if(mode==0)
//	{
//		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,1000);
//		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,1000); 
//		return;
//	}
//	else if(mode==1)
//		pwm_out=100;
//	else if(mode==2)
//		pwm_out=200;
//	else if(mode==3)
//		pwm_out=300;
//	else if(mode==4)
//		pwm_out=400;
//	if(reverse) 
//		pwm_out=-pwm_out;
//	Wheel(pwm_out);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc1);    //AD校准
	
	HAL_UART_Receive_IT(&huart1,rx_buf1,1);
	HAL_UART_Receive_IT(&huart3,rx_buf3,1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
	
//	pidspeed=initPID(6.2,0.5,3.4);	//0.008   0.0035		//6 0.5 3.2(is able to keep for 20min)
	int_k=0.95;
	pidspeed=initPID(0,3.2,0);			//1.2
	//pidspeed=initPID(0,0,0);
	pidangle=initPID(115*0.7*0.7,0,2200*0.7*0.7);			//115*0.6=69	2200*0.6=1320
	__HAL_TIM_SET_COUNTER(&htim1,32768);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		 //Wheel(200);
		 HAL_ADC_Start(&hadc1);     //启动ADC转换
     HAL_ADC_PollForConversion(&hadc1, 50);   //等待转换完成，50为最大等待时间，单位为ms
     if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
		 {
			 ADC_Value = HAL_ADC_GetValue(&hadc1);   //获取AD值
			 ADC_Cal();
		 }
//		 if(gest.speed<0.1&& gest.angle>-0.1)
//			pidspeed.i=0;
		 printf("%.2f",speed_target);
		 //printf("1");
     //HAL_Delay(200);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1 ,(uint8_t*)&ch,1,10);
	return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
