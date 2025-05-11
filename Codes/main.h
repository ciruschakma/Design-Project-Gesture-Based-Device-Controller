/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void SystemClock_Config(void);

/* Private defines -----------------------------------------------------------*/
#define LD2_Pin         GPIO_PIN_5
#define LD2_GPIO_Port   GPIOA

#define B1_Pin          GPIO_PIN_13
#define B1_GPIO_Port    GPIOC

// Touch sensor input (PA0)
#define TOUCH_SENSOR_Pin        GPIO_PIN_0
#define TOUCH_SENSOR_GPIO_Port  GPIOA

// IR sensor output pins (for the two relay channels)
// (Here we use PC10 and PC11; adjust if needed.)
#define IR_UD_Pin       GPIO_PIN_10  // Up/Down control
#define IR_UD_GPIO_Port GPIOC

#define IR_LR_Pin       GPIO_PIN_11  // Left/Right control
#define IR_LR_GPIO_Port GPIOC


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
