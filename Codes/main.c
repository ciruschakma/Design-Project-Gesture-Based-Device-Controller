/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body combining IR gesture sensor,
  *                   touch sensor toggle, backup push button override, and
  *                   persistent gesture command functionality.
  *                   The sensor commands persist until a new gesture updates them,
  *                   and the backup push buttons act as momentary overrides.
  ******************************************************************************
  * @attention
  *
  * This software is provided without USART debugging functionality.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
// USART is no longer needed, so we remove its inclusion:
// #include "usart.h"
#include "gpio.h"

#include "DEV_Config.h"
#include "PAJ7620U2.h"

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
volatile uint8_t ledState = 0;  // 0: process off, 1: process on
/* USER CODE END PTD */

/* Private variables ---------------------------------------------------------*/
unsigned short Gesture_Data;
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
// Remove USART retargeting code since we no longer use printf

/*
   Backup Push Button Initialization:
   This function initializes two push button inputs (PA1 and PA4) as momentary backup override inputs.
   They are configured with an internal pull-up (so they read HIGH when not pressed).
*/
static void OverrideButtons_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable clock for GPIOA if not already enabled */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Configure PA1 as input with pull-up (for IR_UD backup override) */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure PA4 as input with pull-up (for IR_LR backup override) */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  uint8_t i;

  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  // USART is not required so comment out its initialization:
  // MX_USART2_UART_Init();
  MX_I2C1_Init();

  /*
     Initialize the backup push button inputs.
  */
  OverrideButtons_Init();

  /* USER CODE BEGIN 2 */
  // USART debug messages removed.
  // Initialize IR Gesture Sensor
  if (!PAJ7620U2_init())
  { 
    // Error indicator could be implemented using an LED blink or similar method.
    Error_Handler();
  }
  // Sensor initialization
  DEV_I2C_WriteByte(PAJ_BANK_SELECT, 0);  // Select Bank 0
  for (i = 0; i < Gesture_Array_SIZE; i++)
  {
    DEV_I2C_WriteByte(Init_Gesture_Array[i][0], Init_Gesture_Array[i][1]);
  }
  /* USER CODE END 2 */

  /*
     Persistent sensor variables for IR_UD (up/down) and IR_LR (left/right).
     They hold the last valid command.
     Default values are initially set (RESET).
  */
  static uint32_t sensor_UD = GPIO_PIN_RESET;
  static uint32_t sensor_LR = GPIO_PIN_RESET;

  /* Infinite loop */
  while (1)
  {
    if(ledState == 1)
    {
      /* Read gesture sensor and update persistent sensor command variables.
         The sensor may provide a valid command only momentarily,
         so update the persistent variables only when a valid gesture is detected.
      */
      Gesture_Data = DEV_I2C_ReadWord(PAJ_INT_FLAG1);
      if(Gesture_Data)
      {
        switch (Gesture_Data)
        {
          case PAJ_UP:
            sensor_UD = GPIO_PIN_SET;
            break;
          case PAJ_DOWN:
            sensor_UD = GPIO_PIN_RESET;
            break;
          case PAJ_LEFT:
            sensor_LR = GPIO_PIN_SET;
            break;
          case PAJ_RIGHT:
            sensor_LR = GPIO_PIN_RESET;
            break;
          // Only the four gestures are handled. Any other value is simply ignored.
          default:
            break;
        }
        DEV_Delay_ms(50);  // Debounce delay for gesture processing
      }

      /* Determine final output values.
         Start with the persistent sensor command.
      */
      uint32_t final_UD_state = sensor_UD;
      uint32_t final_LR_state = sensor_LR;

      /*
         Backup override integration:
         If a backup push button is pressed (active low), override the sensor command.
      */
      if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET)
      {
        final_UD_state = GPIO_PIN_SET;
      }
      if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET)
      {
        final_LR_state = GPIO_PIN_SET;
      }

      // Write the integrated final state to the IR outputs.
      HAL_GPIO_WritePin(IR_UD_GPIO_Port, IR_UD_Pin, final_UD_state);
      HAL_GPIO_WritePin(IR_LR_GPIO_Port, IR_LR_Pin, final_LR_state);
    }
    else
    {
      // When the process is inactive, ensure IR outputs remain off.
      HAL_GPIO_WritePin(IR_UD_GPIO_Port, IR_UD_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(IR_LR_GPIO_Port, IR_LR_Pin, GPIO_PIN_RESET);
    }

    DEV_Delay_ms(10);  // Small delay for polling
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Initializes the CPU, AHB and APB busses clocks */
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the pins connected to EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // Check if the interrupt comes from the touch sensor (PA0)
  if(GPIO_Pin == TOUCH_SENSOR_Pin)
  {
    HAL_Delay(50);  // Debounce delay
    if(HAL_GPIO_ReadPin(TOUCH_SENSOR_GPIO_Port, TOUCH_SENSOR_Pin) == GPIO_PIN_SET)
    {
      // Toggle process state (and LED indicator) on each valid touch
      ledState = !ledState;
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, (ledState) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
  }
}
/* USER CODE END 4 */

/*
   Original PAJ7620U2 sensor initialization function.
   This remains mostly unchanged.
*/
unsigned char PAJ7620U2_init(void)
{
    unsigned char i, State;
    DEV_Set_I2CAddress(PAJ7620U2_I2C_ADDRESS);
    DEV_Delay_ms(5);
    State = DEV_I2C_ReadByte(0x00);   // Read sensor state
    if (State != 0x20)
        return 0;  // Wake-up failed
    DEV_I2C_WriteByte(PAJ_BANK_SELECT, 0);  // Select Bank 0
    for (i = 0; i < Init_Array; i++)
    {
        DEV_I2C_WriteByte(Init_Register_Array[i][0], Init_Register_Array[i][1]);
    }
    return 1;
}
