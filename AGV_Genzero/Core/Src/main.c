/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "fonts.h"
#include "test.h"

#include "pn532_stm32f1.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc3;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

// #### LINE SENSOR VARIABLES ####

#define line_sensor_front_total_channel 10
#define line_sensor_back_total_channel 10

#define line_sensor_front_trigger_threshhold 400
#define line_sensor_back_trigger_threshhold 400

int _line_read_value;

volatile uint16_t line_sensor_front_values_dma[line_sensor_front_total_channel];
volatile uint16_t line_sensor_back_values_dma[line_sensor_back_total_channel];

volatile uint16_t line_sensor_front_values_calibrated[line_sensor_front_total_channel];
volatile uint16_t line_sensor_back_values_calibrated[line_sensor_back_total_channel];

volatile uint8_t line_sensor_front_on_line_left_number, line_sensor_front_on_line_middle_number, line_sensor_front_on_line_right_number;
volatile uint8_t line_sensor_front_on_line_total_number;

volatile uint8_t line_sensor_back_on_line_left_number, line_sensor_back_on_line_middle_number, line_sensor_back_on_line_right_number;
volatile uint8_t line_sensor_back_on_line_total_number;


uint16_t line_sensor_front_max_sensor_vales[line_sensor_front_total_channel] = {3000, 3004, 2980, 2995, 3000, 3003, 3008, 2985, 2999, 3000};
uint16_t line_sensor_front_min_sensor_vales[line_sensor_back_total_channel] = {1830, 1400, 1100, 880, 2200, 1150, 1040, 1000, 1350, 2100};

uint16_t line_sensor_back_max_sensor_vales[line_sensor_front_total_channel] = {3000, 3004, 2980, 2995, 3000, 3003, 3008, 2985, 2999, 3000};
uint16_t line_sensor_back_min_sensor_vales[line_sensor_back_total_channel] = {1830, 1400, 1100, 880, 2200, 1150, 1040, 1000, 1350, 2100};


const int line_sensor_front_channel_number = sizeof(line_sensor_front_values_dma)/sizeof(line_sensor_front_values_dma[0]);
const int line_sensor_back_channel_number = sizeof(line_sensor_back_values_dma)/sizeof(line_sensor_back_values_dma[0]);

volatile uint16_t line_sensor_front_read_line_value;
volatile uint16_t line_sensor_back_read_line_value;



// #### END LINE SENSOR VARIABLES ####

// -o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-

// #### AGV ORIENTATION VARIABLES ####

uint16_t agv_orientation = 0xFFFF;


// #### END AGV ORIENTATION VARIABLES ####

// -o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-

// #### DiSPLAY DATA AND BUTTON VARIABLES ####

uint8_t Tx_Data_Uart1[5];
uint8_t Rx_Data_Uart1[5];



// #### END DiSPLAY DATA AND BUTTON VARIABLES ####

// -o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-

// #### PID and MOTOR SPEED VARIABLES ####

volatile float P, I, D;

float Kp = 0.4;
float Ki = 0.010;
float Kd = 0.4;

volatile float pid_last_error;
volatile float pid_error;
volatile float pid_motor_speed_change;

volatile uint16_t pid_motor_speed_A;
volatile uint16_t pid_motor_speed_B;
uint16_t pid_motor_base_speed = 200;



uint8_t agv_turn_count = 0;

uint8_t debug_oled[20];
// #### END PID and MOTOR SPEED VARIABLES ####

// -o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-

// #### Task Completion Variables ####

uint16_t On_Task = 0xF00F;  // not on task = 0xF00F || on task = 0xF11F

uint8_t Station = 1;

uint8_t Current_Station = 1;
uint8_t Target_Station = 1;

uint8_t on_task_decisions[5] = {'R', 'F', 'L', 'A', 'A'};

uint16_t pid_motor_orientation;

// #### END Task Completion Variables ####

uint8_t uid[MIFARE_UID_MAX_LENGTH];
int32_t uid_len = 0;
uint8_t uid_version[10];
PN532 pn532;
uint8_t Station_id[5];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void Line_Sensor_Calculation(volatile uint16_t *sensor_values,
						 	  volatile uint16_t *sensor_calibrated_values,
							      	   uint16_t *sensor_max_values,
									   uint16_t *sensor_min_values,

							  volatile uint8_t  *sensor_middle_on_line_number,

							  volatile uint8_t  *sensor_total_on_line_number,
									   uint16_t  sensor_threshhold,
									   uint8_t   sensor_numbers,
							  volatile uint16_t *line_position){
	uint8_t i, on_line = 0;
	uint32_t avg = 0;
	uint32_t sum = 0;
	uint16_t value;
	uint8_t on_sensor_total_number = 0;

	uint8_t middle_on_line = 0;


	for(i = 0; i < sensor_numbers; i++){
		uint16_t calmin, calmax;
		uint16_t denominator;
		calmax = sensor_max_values[i];
		calmin = sensor_min_values[i];

		denominator = calmax - calmin;

		int x = 0;
		if(denominator != 0){
			x = (((signed long)sensor_values[i]) - calmin) * 1000/denominator;
		}
		if(x <0){
			x = 0;
		}
		if(x>1000){
			x = 1000;
		}
		value = (1000-x);
		sensor_calibrated_values[i] = value;

		// start read line number section
		if(value > 400){
			on_line = 1;
		}
		if(value > 200){
			avg += (long)(value)*(i*1000);
			sum += value;
		}
		// end read line number section

		// start on line sensor calculation
		if(value > sensor_threshhold){
			on_sensor_total_number++;
			if(i >= 2 && i <= 7){
				middle_on_line++;
			}
		}
		// end on line sensor calculation
	}

	// start read line number section
	if(!on_line){
		if(_line_read_value < (sensor_numbers - 1) * 1000/2){
			_line_read_value = 0;
		}
		else{
			_line_read_value = (sensor_numbers - 1)*1000;
		}
	}
	else{
		_line_read_value = avg/sum;
	}
	*line_position = _line_read_value;
	// end read line number section

	// start on line sensor calculation

	*sensor_middle_on_line_number = middle_on_line;

	*sensor_total_on_line_number = on_sensor_total_number;
	// end on line sensor calculation
	// 0 - 1 - 2 - 3 - 4 - 5 - 6 - 7 - 8 - 9

	// 2 - 7 --> mid

}

void PID_control(volatile uint16_t *line_position,
				          uint16_t *motor_orientation){

	pid_error = 4500 - *line_position;

	P = pid_error;
	//I = error + I;
	I = 0;
	D = pid_error - pid_last_error;
	pid_last_error = pid_error;

	pid_motor_speed_change = P*Kp + I*Ki + D*Kd;

	pid_motor_speed_A = pid_motor_base_speed - pid_motor_speed_change;
	pid_motor_speed_B = pid_motor_base_speed + pid_motor_speed_change;

	if(pid_motor_speed_A > 255){
		pid_motor_speed_A = 255;
	}
	if(pid_motor_speed_A < 0){
		pid_motor_speed_A = 0;
	}
	if(pid_motor_speed_B > 255){
		pid_motor_speed_B = 255;
	}
	if(pid_motor_speed_B < 0){
		pid_motor_speed_B = 0;
	}

}

void PID_Forward_Rotation(uint16_t enableA, uint16_t enableB){

	//LEFT
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);

	//RIGHT
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);

	//Right
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, enableA);

	//Left
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, enableB);
}

void PID_Motor_Turn_Left(uint16_t _speed){
	//LEFT
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);

	//RIGHT
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);

	//Right
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, _speed);

	//Left
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, _speed);
}

void PID_Motor_Turn_Right(uint16_t _speed){
	//LEFT
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);

	//RIGHT
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);

	//Right
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, _speed);

	//Left
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, _speed);
}


void PID_Motor_All_Break(){
	//LEFT
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);

	//RIGHT
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);

	//Right
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);

	//Left
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}

void AGV_waiting(){
	if(Target_Station == Current_Station){
		On_Task = 0xF00F;
		if(Target_Station != Station){
			Target_Station = Station;
			if(Target_Station == 2){
				on_task_decisions[0] = 'R';
				on_task_decisions[1] = 'L';
				on_task_decisions[2] = 'A';
				on_task_decisions[3] = 'A';
				on_task_decisions[4] = 'A';
			}
			if(Target_Station == 3){
				on_task_decisions[0] = 'R';
				on_task_decisions[1] = 'F';
				on_task_decisions[2] = 'L';
				on_task_decisions[3] = 'A';
				on_task_decisions[4] = 'A';
			}
		}
	}
	if(Target_Station != Current_Station){
		On_Task = 0xF11F;
	}
}

void Task_Completion(){

}

void AGV_Turn_Detection_Completion(volatile uint16_t *sensor_calibrated_values,
								   volatile uint8_t  *sensor_middle_on_line_number,
								   	   	   	uint8_t  *decision_array){
#define white_detection_thresh_hold 350
#define black_detection_thresh_hold 500
#define sensor_mid_on_line_thresh_hold 1
#define first_timer_buffer 1500
#define second_timer_buffer 800
#define skip_turn_timer_buffer 100

	uint8_t _turn_decide = 0;

	if((sensor_calibrated_values[8] < white_detection_thresh_hold) && (sensor_calibrated_values[9] < white_detection_thresh_hold) &&
	   (sensor_calibrated_values[0] > black_detection_thresh_hold) && (sensor_calibrated_values[1] > black_detection_thresh_hold)){
		if(*sensor_middle_on_line_number >= 1){
			PID_Forward_Rotation(150, 150);
			HAL_Delay(first_timer_buffer);
			PID_Motor_All_Break();


			agv_turn_count += 1;
			if(decision_array[agv_turn_count - 1] == 'L'){
				PID_Motor_Turn_Left(200);
				HAL_Delay(second_timer_buffer);

				_turn_decide = 'L';
			}
			if(decision_array[agv_turn_count - 1] == 'F'){
				PID_Forward_Rotation(255, 255);
				HAL_Delay(skip_turn_timer_buffer);
				_turn_decide = 0;
			}

		}

	}
	if((sensor_calibrated_values[8] > black_detection_thresh_hold) && (sensor_calibrated_values[9] > black_detection_thresh_hold) &&
	   (sensor_calibrated_values[0] < white_detection_thresh_hold) && (sensor_calibrated_values[1] < white_detection_thresh_hold)){
		if(*sensor_middle_on_line_number >= 1){
			PID_Forward_Rotation(150, 150);
			HAL_Delay(first_timer_buffer);
			PID_Motor_All_Break();

			agv_turn_count += 1;
			if(decision_array[agv_turn_count - 1] == 'R'){
				PID_Motor_Turn_Right(200);
				HAL_Delay(second_timer_buffer);

				_turn_decide = 'R';
			}
			if(decision_array[agv_turn_count - 1] == 'F'){
				PID_Forward_Rotation(255, 255);
				HAL_Delay(skip_turn_timer_buffer);
				_turn_decide = 0;
			}
		}

	}

	if((sensor_calibrated_values[8] > black_detection_thresh_hold) && (sensor_calibrated_values[9] > black_detection_thresh_hold) &&
	   (sensor_calibrated_values[0] > black_detection_thresh_hold) && (sensor_calibrated_values[1] > black_detection_thresh_hold)){
		if(*sensor_middle_on_line_number >= 1){
			PID_Forward_Rotation(150, 150);
			HAL_Delay(first_timer_buffer);
			PID_Motor_All_Break();

			agv_turn_count += 1;
			if(decision_array[agv_turn_count - 1] == 'R'){
				PID_Motor_Turn_Right(200);
				HAL_Delay(second_timer_buffer);

				_turn_decide = 'R';
			}
			if(decision_array[agv_turn_count - 1] == 'L'){
				PID_Motor_Turn_Left(200);
				HAL_Delay(second_timer_buffer);

				_turn_decide = 'L';
			}
			if(decision_array[agv_turn_count - 1] == 'F'){
				PID_Forward_Rotation(255, 255);
				HAL_Delay(skip_turn_timer_buffer);
				_turn_decide = 0;
			}
		}

	}

	if(_turn_decide == 'L'){
		if(sensor_calibrated_values[5] < white_detection_thresh_hold){
			PID_Motor_Turn_Left(200);
		}
		if(sensor_calibrated_values[5] > black_detection_thresh_hold){
			PID_Motor_All_Break();
			_turn_decide = 0;

		}

	}
	if(_turn_decide == 'R'){
		if(sensor_calibrated_values[4] < white_detection_thresh_hold){
			PID_Motor_Turn_Right(200);
		}
		if(sensor_calibrated_values[4] > black_detection_thresh_hold){
			PID_Motor_All_Break();
			_turn_decide = 0;
		}
	}

}

void AGV_Orientation(		  uint16_t *_orientation_state,
					 volatile uint16_t *_front_line_sensor_array,
					 volatile uint16_t *_back_line_sensor_array
					 ){


}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	Tx_Data_Uart1[0] = 5;
	Tx_Data_Uart1[1] = 4;
	Tx_Data_Uart1[2] = 3;
	Tx_Data_Uart1[3] = 2;
	Tx_Data_Uart1[4] = 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(Rx_Data_Uart1[0] == 0x56){
		HAL_GPIO_TogglePin(UART_STATUS_GPIO_Port, UART_STATUS_Pin);

		if(Rx_Data_Uart1[1] == 0xF1){
			Station = 1;
		}
		if(Rx_Data_Uart1[1] == 0xF2){
			Station = 2;
		}
		if(Rx_Data_Uart1[1] == 0xF3){
			Station = 3;
		}
		if(Rx_Data_Uart1[1] == 0xF4){
			Station = 4;
		}
		if(Rx_Data_Uart1[1] == 0xF5){
			Station = 5;
		}

	}

}

void Startup_RFID(){

	PN532_SPI_Init(&pn532);
	//HAL_GPIO_WritePin(RFID_SSF_GPIO_Port, RFID_SSF_Pin, 0);

	if(PN532_GetFirmwareVersion(&pn532, uid_version) != PN532_STATUS_OK)
	{
		while(1){

		}
	}

	PN532_SamConfiguration(&pn532);

}
void AGV_RFID_Detection(){
	uid_len = PN532_ReadPassiveTarget(&pn532, uid, PN532_MIFARE_ISO14443A, 100);

	if(uid_len >= 4){
		Station_id[0] = uid[0];
		Station_id[1] = uid[1];
		Station_id[2] = uid[2];
		Station_id[3] = uid[3];
		Station_id[4] = uid[4];
	}
}

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */


  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  HAL_ADC_Start_DMA(&hadc1, (uint16_t*) line_sensor_front_values_dma, line_sensor_front_channel_number);
  HAL_ADC_Start_DMA(&hadc3, (uint16_t*) line_sensor_back_values_dma, line_sensor_back_channel_number);

  HAL_UART_Transmit_DMA(&huart1, Tx_Data_Uart1, sizeof (Tx_Data_Uart1));
  HAL_UART_Receive_DMA (&huart1, Rx_Data_Uart1, sizeof (Rx_Data_Uart1));

  SSD1306_Init (); // initialize the display


  //HAL_GPIO_WritePin(RFID_BSS_GPIO_Port, RFID_BSS_Pin, RESET);
  Startup_RFID();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HAL_GPIO_WritePin(RFID_BSS_GPIO_Port, RFID_BSS_Pin, RESET);
//	  HAL_GPIO_WritePin(RFID_SS_GPIO_Port, RFID_SS_Pin, RESET);

	  AGV_RFID_Detection();
//	  if(On_Task == 0xF11F){
//		  Line_Sensor_Calculation(line_sensor_front_values_dma,
//				  	  	  	  	  line_sensor_front_values_calibrated,
//								  line_sensor_front_max_sensor_vales,
//								  line_sensor_front_min_sensor_vales,
//
//								 &line_sensor_front_on_line_middle_number,
//
//								 &line_sensor_front_on_line_total_number,
//								  line_sensor_front_trigger_threshhold,
//								  line_sensor_front_total_channel,
//								 &line_sensor_front_read_line_value);
//
//		  PID_control(&line_sensor_front_read_line_value, &pid_motor_orientation);
//
//		  PID_Forward_Rotation(pid_motor_speed_A, pid_motor_speed_B);
//
//		  AGV_Turn_Detection_Completion(line_sensor_front_values_calibrated,
//				  	  	  	  	  	   &line_sensor_front_on_line_middle_number,
//									    on_task_decisions);
//	  }
//	  else if(On_Task == 0xF00F){
//		  AGV_waiting();
//	  }


	  //AGV_Orientation(&agv_orientation, main_line_sensor_front_values_calibrated, main_line_sensor_back_values_calibrated);

	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);


//	  sprintf(debug_oled, "lc: %d", agv_turn_count);
//	  SSD1306_GotoXY (10, 30);
//	  SSD1306_Puts (debug_oled, &Font_11x18, 1);
//	  SSD1306_UpdateScreen(); // update screen

	  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 10;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 10;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_10;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA2_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RFID_IRQ_Pin|IN3_Pin|IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RFID_SS_GPIO_Port, RFID_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RFID_Reset_Pin|RFID_BSS_Pin|UART_STATUS_Pin|INTER_STATUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_8|IN1_Pin|IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RFID_IRQ_Pin IN3_Pin IN4_Pin */
  GPIO_InitStruct.Pin = RFID_IRQ_Pin|IN3_Pin|IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RFID_SS_Pin RFID_Reset_Pin RFID_BSS_Pin UART_STATUS_Pin
                           INTER_STATUS_Pin */
  GPIO_InitStruct.Pin = RFID_SS_Pin|RFID_Reset_Pin|RFID_BSS_Pin|UART_STATUS_Pin
                          |INTER_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN2_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
