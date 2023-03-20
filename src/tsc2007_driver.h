/**
 * @defgroup touch_controller Touch Controller
 * @ingroup ecu_abstracion_layer
 * @file      tsc2007_driver.h
 * @authors   Ricard Molins
 * @copyright Closed
 *
 * @brief Driver for TSC2007 Touch controller from Texas instrument for the STM32F7XX HAL
 *
 */

#ifndef __TSC2007_H_
#define __TSC2007_H_

#ifdef __cplusplus
extern "C" {
#endif


/*******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "stm32f7xx_hal.h"

/*******************************************************************************
* Defines
*******************************************************************************/


/*******************************************************************************
* Local Types and Typedefs
*******************************************************************************/
typedef enum{
    TSC_IT_IDLE,
    TSC_IT_GET_X,
    TSC_IT_GET_Y,
    TSC_IT_GET_Z,
    TSC_IT_PROCESS,
} FSM_TSC2007DriverIT;

/*******************************************************************************
* Variables
*******************************************************************************/

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void TSC2007_Init(void);
void Task_TSC2007Driver(void);
void Task_TSC2007DriverIT(void);
void TSC2007_RegisterI2CHandler(I2C_HandleTypeDef* i2c_handler);

void TSC2007_PENIRQ(uint8_t val);
void TSC2007_I2C_TxCompleteCallback(void);
void TSC2007_I2C_RxCompleteCallback(void);
void TSC2007_I2C_ErrorCallback(void);

bool TSC2007_SampleTouch(uint16_t * x, uint16_t *y);

/*******************************************************************************
* Functions
*******************************************************************************/

#ifdef __cplusplus
}
#endif


#endif /* __TSC2007_H_ */
