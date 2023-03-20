# TSC2007 Driver for STM32Cube Library {#TouchController}

The TSC2007 is 4 wire I2C touch screen controller. It interfaces with the Touch panel with a 4 wire Interface (X+,X-,Y+,Y-) and with a I2C + a interrupt line with the microcontroller.

The interrupt line is called ```PENIRQ```

# Integration with STM32Cube

## HW configuration
The TSC2007 allows for HW configuratino of the two LSB of the address. Those are configured in  ```tsc2007_driver_hw_conf.h``` with the defines:
* ```TSC2007_A1_HW_CONF```
* ```TSC2007_A1_HW_CONF```


## I2C Configuration

The MCU acts as master while the TSC2007 acts as slave

* Standard Mode
* I2C speed 100
* Raise time 100
* Fall time 100
* Coefficient of Digital Filter 15
* Analog filter Enabled
  
## External Interrupt
The PENIRQ itnerrupt shall be connected to a GPIO. This GPIO shall be configured as ```External Interrupt Mode with Risin/Falling edge trigger detection```

# SW integrations

The driver works with a polling scheme. The polling can be done in blocking or interrupt mode. Before the polling functions it is necessary to:
* ```TSC2007_RegisterI2CHandler(...)```: register the I2C that is used.
* ```TSC2007_Init(...)```: Initialize the drivers.

## Touch polling in blocking mode 

Calling ```Task_TSC2007Driver``` will read the X/Y/Z information from the driver and process the received information.

The last read position can be retrieved with ```bool TSC2007_SampleTouch(uint16_t * x, uint16_t *y);```

## Touch polling interrupt driven 

Calling ```Task_TSC2007DriverIT``` will trigger a read of the X/Y/Z information from the driver and process the received information.

The last read position can be retrieved with ```bool TSC2007_SampleTouch(uint16_t * x, uint16_t *y);```

### Interruption inputs

* ```TSC2007_PENIRQ(...)```: triggered by PENIRQ including the value of the status of the GPIO (currently not used)
* ```TSC2007_I2C_RxCompleteCallback(...)```: I2C Rx Complete Callback
* ```TSC2007_I2C_TxCompleteCallback(...)```: I2C Tx Complete Callback


# Notes 
* Tested with STM32F767