/********************************************************************************************************************
                                         MAX30101 header file
*********************************************************************************************************************
*
*           AUTHOR: Anass KAZIZ (kaziz.anas@gmail.com)
*           FILENAME: MAX30101.h
*           LAST MODIFIED: 29/12/2017
*           TARGET: MK64FN1M0VDC12 (CPU: ARM® Cortex®-M4 32-bit core)
*           COMPILER: MBED
*
*           The nouns of functions have been ispired from the code of MAX30102 sensor, which was written
*           by Maxim Integrated Products, Inc, which own all its rights.
*
*********************************************************************************************************************/



/********************************************************************************************************************
*                                               INCLUDES
*********************************************************************************************************************/

#ifndef MAX30101_H_
#define MAX30101_H_

#include "mbed.h"

/********************************************************************************************************************
*                                               REGISTERS
*********************************************************************************************************************/

/*Sensor's adress (picked up form Datasheet)*/
#define MAX30101_I2C_ADDRESS 0x57


/*register addresses (Same Map rigesters in the Datasheet)*/

#define REG_INTR_STATUS_1 0x00
#define REG_INTR_STATUS_2 0x01
#define REG_INTR_ENABLE_1 0x02
#define REG_INTR_ENABLE_2 0x03
#define REG_FIFO_WR_PTR 0x04
#define REG_OVF_COUNTER 0x05
#define REG_FIFO_RD_PTR 0x06
#define REG_FIFO_DATA 0x07
#define REG_FIFO_CONFIG 0x08
#define REG_MODE_CONFIG 0x09
#define REG_SPO2_CONFIG 0x0A
#define REG_LED1_PA 0x0C
#define REG_LED2_PA 0x0D
#define REG_PILOT_PA 0x10
#define REG_MULTI_LED_CTRL1 0x11
#define REG_MULTI_LED_CTRL2 0x12
#define REG_TEMP_INTR 0x1F
#define REG_TEMP_FRAC 0x20
#define REG_TEMP_CONFIG 0x21
#define REG_PROX_INT_THRESH 0x30
#define REG_REV_ID 0xFE
#define REG_PART_ID 0xFF


/********************************************************************************************************************
*                                                CLASS
*********************************************************************************************************************/

class MAX30101
{
public:

    /* Class constructor*/
    MAX30101(PinName sda, PinName scl); 
      
    bool who_I_am(void);
    bool maxim_max30101_init();
    bool maxim_max30101_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led);
    bool maxim_max30101_write_reg(uint8_t uch_addr, uint8_t uch_data);
    bool maxim_max30101_read_reg(uint8_t uch_addr, uint8_t *puch_data);
    bool maxim_max30101_reset(void);    
    
     
private:

    I2C* i2c_; 

};


#endif /* MAX30101_H_ */