/**
 * @ingroup touch_controller
 * @file      tsc2007_driver.c
 * @authors   Ricard Molins
 * @copyright Closed
 *
 * @brief Driver for TSC2007 Touch controller from Texas instrument for the STM32F7XX HAL
 *
 */

/*******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "tsc2007_driver.h"
#include "tsc2007_driver_hw_conf.h"
#include "i2c.h"
#ifndef TEST
#include "stm32f7xx_hal.h"
#else
#endif
#include "gpio.h"

/*******************************************************************************
* Defines
*******************************************************************************/
/* Addres Byte defines */
#define TSC2007_D7_D4 0x12 /* 0b1 0010 */
#define TSC2007_D7_D4_OFFSET 3
#define TSC2007_D7_D4_MASK 0xF8 /* */
#define TSC2007_I2C_ADDR_READ_WRITE_OFFSET 0
#define TSC2007_I2C_ADDR_READ_WRITE_MASK 0x1
#define TSC2007_I2C_ADDR_READ_VAL 1 /* I2C master read from TSC (I2C read addressing) */
#define TSC2007_I2C_ADDR_WRITE_VAL 0 /* I2C master write to TSC (I2C write addressing) */
#define TSC2007_I2C_ADDR (((TSC2007_D7_D4<<TSC2007_D7_D4_OFFSET)&TSC2007_D7_D4_MASK) \
        | ((TSC2007_ADDR_HW_CONF_VAL << TSC2007_ADDR_HW_CONF_VAL_OFFSET) & TSC2007_ADDR_HW_CONF_VAL_MASK))

/* Converter Function Select as in Table 3 from TSC2007 datasheet */
#define TC_FUNCTION_MEASURE_TEMP_0 0
#define TC_FUNCTION_MEASURE_AUX 2
#define TC_FUNCTION_MEASURE_TEMP_1 4
#define TC_FUNCTION_ACTIVATE_X_DRIVERS 8
#define TC_FUNCTION_ACTIVATE_Y_DRIVERS 9
#define TC_FUNCTION_ACTIVATE_Yp_Xn_DRIVERS 10
#define TC_FUNCTION_SETUP_COMMAND 11
#define TC_FUNCTION_MEASURE_X_POSITION 12
#define TC_FUNCTION_MEASURE_Y_POSITION 13
#define TC_FUNCTION_MEASURE_Z1_POSITION 14
#define TC_FUNCTION_MEASURE_Z2_POSITION 15

/* Power modes */
#define POWER_CONFIG_POWER_DOWN_BETWEEN_CYCLES 0
#define POWER_CONFIG_ADC_ON_IRQ_DISABLED 1
#define POWER_CONFIG_ADC_OFF_IRQ_ENABLED 2
#define POWER_CONFIG_ADC_ON_IRQ_DISABLED2 3 /* Same as previous? Possible errata */

/* Adc resolution */
#define ADC_CONFIG_12_BITS_RESOLUTION 0
#define ADC_CONFIG_8_BITS_RESOLUTION 1
/* Filter configuration */
#define FILTER_CONTROL_USE_ONBOARD_MAV_FILTER 0
#define FILTER_CONTROL_BYPASS_ONBOARD 1
/* Pull up configuration*/
#define PULL_UP_RESISTORS_50K 0
#define PULL_UP_RESISTORS_90k 1
/* RX buffer lengths */
#define MAX_LENGTH_RX_MESSAGE 2

#define IRQ_TRIGGERED_BY_PUSH 0
#define IRQ_TRIGGERED_BY_RELEASE 1


/*******************************************************************************
* Local Types and Typedefs
*******************************************************************************/
/**
 * Union to build command bytes
 */
typedef union {
    struct {
        unsigned char dont_care :1; /**< do not care*/
        unsigned char adc_config :1; /**< 12 or 8 bits sampling */
        unsigned char power_config :2; /**< Select PENIRQ enable and poder down */
        unsigned char tc_function :4; /**< function selector*/
    } cmd_bits;
    unsigned char cmd_byte;
} TSC2007_CommandByte;

/**
 * Union to build command bytes for the Setup command special case
 */
typedef union {
    struct {
        unsigned char pull_up_resistors :1; /**< Select 50/80 Ohm filters */
        unsigned char filter_control :1; /**< Onboard vs Bypass */
        unsigned char reserved :2; /**< must write 00 */
        unsigned char tc_function :4; /**< function selector*/
    } cmd_bits;
    unsigned char cmd_byte;
} TSC2007_CommandByteSetupCommand;


/*******************************************************************************
* Variables
*******************************************************************************/
I2C_HandleTypeDef * tsc2007_i2c_handle;
uint8_t x_buffer[MAX_LENGTH_RX_MESSAGE] = {0};
uint8_t y_buffer[MAX_LENGTH_RX_MESSAGE] = {0};
uint8_t z_buffer[MAX_LENGTH_RX_MESSAGE] = {0};
uint8_t dummy_buffer[MAX_LENGTH_RX_MESSAGE] = {0};
uint16_t y_value = 0;
uint16_t x_value = 0;
bool currenlty_touhced = false;
FSM_TSC2007DriverIT fsm_tsc2007_driver_it = TSC_IT_IDLE;
TSC2007_CommandByte driver_command = { 0 };

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static void TSC2007_GetX(void);
static void TSC2007_GetY(void);
static void TSC2007_GetZ(void);
static void TSC2007_ProcessTouch(void);

/*******************************************************************************
* Functions
*******************************************************************************/
void TSC2007_Init(void)
{

}

/**
 * @brief Reads the  current touch status of the driver in blocking mode.
 */
void Task_TSC2007Driver(void)
{
    TSC2007_CommandByte driver_command = { 0 };

    driver_command.cmd_bits.tc_function = TC_FUNCTION_MEASURE_Y_POSITION;
    driver_command.cmd_bits.power_config = POWER_CONFIG_ADC_ON_IRQ_DISABLED;
    driver_command.cmd_bits.adc_config = ADC_CONFIG_12_BITS_RESOLUTION;
    HAL_I2C_Master_Transmit(tsc2007_i2c_handle, TSC2007_I2C_ADDR, &driver_command.cmd_byte, 1,100);
    HAL_I2C_Master_Receive(tsc2007_i2c_handle, TSC2007_I2C_ADDR, &y_buffer[0], 2,100);
    driver_command.cmd_bits.tc_function = TC_FUNCTION_MEASURE_X_POSITION;
    driver_command.cmd_bits.power_config = POWER_CONFIG_ADC_ON_IRQ_DISABLED;
    driver_command.cmd_bits.adc_config = ADC_CONFIG_12_BITS_RESOLUTION;
    HAL_I2C_Master_Transmit(tsc2007_i2c_handle, TSC2007_I2C_ADDR, &driver_command.cmd_byte, 1, 100);
    HAL_I2C_Master_Receive(tsc2007_i2c_handle, TSC2007_I2C_ADDR, &x_buffer[0], 2,100);

    driver_command.cmd_bits.tc_function = TC_FUNCTION_MEASURE_X_POSITION;
    driver_command.cmd_bits.power_config = POWER_CONFIG_ADC_ON_IRQ_DISABLED;
    driver_command.cmd_bits.adc_config = ADC_CONFIG_12_BITS_RESOLUTION;
    HAL_I2C_Master_Transmit(tsc2007_i2c_handle, TSC2007_I2C_ADDR, &driver_command.cmd_byte, 1, 100);
    HAL_I2C_Master_Receive(tsc2007_i2c_handle, TSC2007_I2C_ADDR, &x_buffer[0], 2,100);
    y_value =  (((uint16_t) y_buffer[1]) >> 4) & 0x0F;
    y_value = y_value | ((((uint16_t) y_buffer[0]) << 4) & 0xFF0);
    x_value =  (((uint16_t) x_buffer[1]) >> 4) & 0x0F;
    x_value = x_value | ((((uint16_t) x_buffer[0]) << 4) & 0xFF0);


    driver_command.cmd_bits.tc_function = TC_FUNCTION_MEASURE_Z1_POSITION;
    driver_command.cmd_bits.power_config = POWER_CONFIG_ADC_ON_IRQ_DISABLED;
    driver_command.cmd_bits.adc_config = ADC_CONFIG_12_BITS_RESOLUTION;
    HAL_I2C_Master_Transmit(tsc2007_i2c_handle, TSC2007_I2C_ADDR, &driver_command.cmd_byte, 1, 100);
    HAL_I2C_Master_Receive(tsc2007_i2c_handle, TSC2007_I2C_ADDR, &z_buffer[0], 2,100);

    if (z_buffer[0] > 0){
        currenlty_touhced = true;
    } else {
        currenlty_touhced = false;
    }

}

/**
 * @brief Triggers a start of touch read in non-blocking, interrupt driven mode.
 */
void Task_TSC2007DriverIT(void)
{
    /* Trigger the start of a read */
    TSC2007_GetY();
}

/**
 * @brief Starts a read X request
 */
static void TSC2007_GetX(void)
{
    fsm_tsc2007_driver_it = TSC_IT_GET_X;
    driver_command.cmd_bits.tc_function = TC_FUNCTION_MEASURE_X_POSITION;
    driver_command.cmd_bits.power_config = POWER_CONFIG_ADC_ON_IRQ_DISABLED;
    driver_command.cmd_bits.adc_config = ADC_CONFIG_12_BITS_RESOLUTION;
    HAL_I2C_Master_Transmit_IT(tsc2007_i2c_handle, TSC2007_I2C_ADDR, &driver_command.cmd_byte, 1);
}

/**
 * @brief Starts a read Y request
 */
static void TSC2007_GetY(void)
{
    fsm_tsc2007_driver_it = TSC_IT_GET_Y;
    driver_command.cmd_bits.tc_function = TC_FUNCTION_MEASURE_Y_POSITION;
    driver_command.cmd_bits.power_config = POWER_CONFIG_ADC_ON_IRQ_DISABLED;
    driver_command.cmd_bits.adc_config = ADC_CONFIG_12_BITS_RESOLUTION;
    HAL_I2C_Master_Transmit_IT(tsc2007_i2c_handle, TSC2007_I2C_ADDR,&driver_command.cmd_byte, 1);

}

/**
 * @brief Starts a read Z request
 */
static void TSC2007_GetZ(void)
{
    fsm_tsc2007_driver_it = TSC_IT_GET_Z;
    driver_command.cmd_bits.tc_function = TC_FUNCTION_MEASURE_Z1_POSITION;
    driver_command.cmd_bits.power_config = POWER_CONFIG_ADC_ON_IRQ_DISABLED;
    driver_command.cmd_bits.adc_config = ADC_CONFIG_12_BITS_RESOLUTION;
    HAL_I2C_Master_Transmit_IT(tsc2007_i2c_handle, TSC2007_I2C_ADDR, &driver_command.cmd_byte, 1);
}

/**
 * @brief Process the information received from the driver.
 */
static void TSC2007_ProcessTouch(void)
{
    fsm_tsc2007_driver_it = TSC_IT_PROCESS;

    y_value = (((uint16_t) y_buffer[1]) >> 4) & 0x0F;
    y_value = y_value | ((((uint16_t) y_buffer[0]) << 4) & 0xFF0);

    x_value = (((uint16_t) x_buffer[1]) >> 4) & 0x0F;
    x_value = x_value | ((((uint16_t) x_buffer[0]) << 4) & 0xFF0);

    if (z_buffer[0] > 0) {
        currenlty_touhced = true;
    } else {
        currenlty_touhced = false;
    }
}


/**
 * @brief Register the I2C handler for the driver
 *
 * @param i2c_handler
 */
void TSC2007_RegisterI2CHandler(I2C_HandleTypeDef* i2c_handler)
{
    tsc2007_i2c_handle = i2c_handler;
}


/**
 * @brief IRQ assigned to the PENIRQ PIN.
 *
 * @param val Status of the Pin after the interrupt (allows knowing if the trigger was a pulse/release)
 */
void TSC2007_PENIRQ(uint8_t val)
{
    DisablePENIRQ();
}

/**
 * @brief I2C Rx Complete Callback. Steps the FSM
 */
void TSC2007_I2C_RxCompleteCallback(void)
{
   switch (fsm_tsc2007_driver_it) {
    case TSC_IT_GET_X:
        TSC2007_GetZ();
        break;
    case TSC_IT_GET_Y:
        TSC2007_GetX();
        break;
    case TSC_IT_GET_Z:
        TSC2007_ProcessTouch();
        break;
    default:
        break;
    }
}

/**
 * @brief I2C Tx Complete Callback. Steps the FSM
 */
void TSC2007_I2C_TxCompleteCallback(void)
{
   switch (fsm_tsc2007_driver_it) {
    case TSC_IT_IDLE:
        HAL_I2C_Master_Receive_IT(tsc2007_i2c_handle, TSC2007_I2C_ADDR, &dummy_buffer[0], 2);
        break;
    case TSC_IT_GET_X:
        HAL_I2C_Master_Receive_IT(tsc2007_i2c_handle, TSC2007_I2C_ADDR, &x_buffer[0], 2);
        break;
    case TSC_IT_GET_Y:
        HAL_I2C_Master_Receive_IT(tsc2007_i2c_handle, TSC2007_I2C_ADDR, &y_buffer[0], 2);
        break;
    case TSC_IT_GET_Z:
        HAL_I2C_Master_Receive_IT(tsc2007_i2c_handle, TSC2007_I2C_ADDR, &z_buffer[0], 2);
        break;
    case TSC_IT_PROCESS:
        HAL_I2C_Master_Receive_IT(tsc2007_i2c_handle, TSC2007_I2C_ADDR, &dummy_buffer[0], 2);
        break;
    default:
        break;
    }
}


void TSC2007_I2C_ErrorCallback(void)
{
    /* TODO */
}

/**
 * @brief Returns current status of the Touch panel
 *
 * @param *x placeholder for x value
 * @param *y placeholder for x value
 *
 * @retval true if currently under touch
 */
bool TSC2007_SampleTouch(uint16_t * x, uint16_t *y)
{
    /* X and Y axis are interchanged X<-->Y*/
    *x = y_value;
    *y = x_value;
    return currenlty_touhced;
}
