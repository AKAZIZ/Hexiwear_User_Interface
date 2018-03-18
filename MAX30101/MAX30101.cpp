/********************************************************************************************************************
                                         MAX30101 Implementation
*********************************************************************************************************************
*
*           AUTHOR: Anass KAZIZ (kaziz.anas@gmail.com)
*           FILENAME: MAX30101.cpp
*           LAST MODIFIED: 29/12/2017
*           TARGET: MK64FN1M0VDC12 (CPU: ARM® Cortex®-M4 32-bit core)
*           COMPILER: MBED
*
*           Some parts of this code have been ispired from the code of MAX30102 sensor, which was written
*           by Maxim Integrated Products, Inc, which own all its rights.
*
*********************************************************************************************************************/
                                         
#include "mbed.h"
#include "MAX30101.h"

/* Classe Constructor*/

MAX30101::MAX30101(PinName sda, PinName scl) {

    i2c_ = new I2C(sda, scl);
    //400KHz, as specified in the datasheet.
    i2c_->frequency(400000);
}



/* This function writes a value to a max30101 register
*  param[in]    uch_addr    - register address
*  param[in]    uch_data    - register data
*  Return True on Success 
*/

bool MAX30101::maxim_max30101_write_reg(uint8_t uch_addr, uint8_t uch_data)

{
  char ach_i2c_data[2];
  ach_i2c_data[0]=uch_addr;
  ach_i2c_data[1]=uch_data;

  int WR = (i2c_->write((MAX30101_I2C_ADDRESS << 1) & 0xFE, ach_i2c_data, 2, false));
  wait_ms(1);        
  if(WR==0)
    return true;
  else
    return false;
}




/* This function reads a value to a max30101 register
*  param[in]    uch_addr    - register address
*  param[in]    puch_data   - pointer that stores the register data
*  Return True on Success 
*/

bool MAX30101::maxim_max30101_read_reg(uint8_t uch_addr, uint8_t *puch_data)

{     
  char ch_i2c_data[1];
  ch_i2c_data[0]=uch_addr;
  
  int WR = (i2c_->write((MAX30101_I2C_ADDRESS << 1) & 0xFE, ch_i2c_data, 1, true));
  wait_ms(16);
    
  int RD= (i2c_->read((MAX30101_I2C_ADDRESS << 1) | 0x01, ch_i2c_data, 1, false));
  wait_ms(1);
    
  if(WR!=0) 
    return false;
  if(RD==0)
  {
    *puch_data= ch_i2c_data[0];
    return true;
  }
  else
    return false;
}





/* Who I am function
*  This function serves to check
*  if the I2C interface was established successfully
*  Ispired from the MAX30102's Code 
*/

bool  MAX30101::who_I_am(void) {
    
    unsigned char uch_temp;
    if (maxim_max30101_read_reg(REG_PART_ID, &uch_temp)==true)
    {
     return true;   
    }
    else

     return false;
}




/* This function initializes the max30101
*  param        None
*  Return True on Success
*  Ispired from the MAX30102's Code
*/

bool MAX30101::maxim_max30101_init()

{
  if(!maxim_max30101_write_reg(REG_INTR_ENABLE_1,0xc0)) // INTR setting
    return false;
  if(!maxim_max30101_write_reg(REG_INTR_ENABLE_2,0x00))
    return false;
  if(!maxim_max30101_write_reg(REG_FIFO_WR_PTR,0x00))  //FIFO_WR_PTR[4:0]
    return false;
  if(!maxim_max30101_write_reg(REG_OVF_COUNTER,0x00))  //OVF_COUNTER[4:0]
    return false;
  if(!maxim_max30101_write_reg(REG_FIFO_RD_PTR,0x00))  //FIFO_RD_PTR[4:0]
    return false;
  if(!maxim_max30101_write_reg(REG_FIFO_CONFIG,0x0f))  //sample avg = 1, fifo rollover=false, fifo almost full = 17
    return false;
  if(!maxim_max30101_write_reg(REG_MODE_CONFIG,0x03))   //0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
    return false;
  if(!maxim_max30101_write_reg(REG_SPO2_CONFIG,0x27))  // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)
    return false;
  
  if(!maxim_max30101_write_reg(REG_LED1_PA,0x24))   //Choose value for ~ 7mA for LED1
    return false;
  if(!maxim_max30101_write_reg(REG_LED2_PA,0x24))   // Choose value for ~ 7mA for LED2
    return false;
  if(!maxim_max30101_write_reg(REG_PILOT_PA,0x7f))   // Choose value for ~ 25mA for Pilot LED
    return false;
  return true;  
}




/* This function reads reads a set of samples from the max30101 FIFO register
*  param[out]   *pun_red_led   - pointer that stores the red LED reading data
*  param[out]   *pun_ir_led    - pointer that stores the IR LED reading data
*  Return True on Success 
*  Ispired from the MAX30102's Code 
*/

bool MAX30101::maxim_max30101_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)

{
  uint32_t un_temp;
  unsigned char uch_temp;
  *pun_red_led=0;
  *pun_ir_led=0;
  char ach_i2c_data[6];
  
  //read and clear status register
  maxim_max30101_read_reg(REG_INTR_STATUS_1, &uch_temp);
  maxim_max30101_read_reg(REG_INTR_STATUS_2, &uch_temp);
  
  ach_i2c_data[0]=REG_FIFO_DATA;
  
  if((i2c_->write((MAX30101_I2C_ADDRESS << 1) & 0xFE, ach_i2c_data, 1, true))!=0)
    return false;
  if((i2c_->read((MAX30101_I2C_ADDRESS << 1) | 0x01, ach_i2c_data, 6, false))!=0)
  {
    return false;
  }
  un_temp=(unsigned char) ach_i2c_data[0];
  un_temp<<=16;
  *pun_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[1];
  un_temp<<=8;
  *pun_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[2];
  *pun_red_led+=un_temp;
  
  un_temp=(unsigned char) ach_i2c_data[3];
  un_temp<<=16;
  *pun_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[4];
  un_temp<<=8;
  *pun_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[5];
  *pun_ir_led+=un_temp;
  *pun_red_led&=0x03FFFF;  //Mask MSB [23:18]
  *pun_ir_led&=0x03FFFF;  //Mask MSB [23:18]
  
  return true;
}




/* This function resets the max30101
*  param        None
*  Return True on Success 
*  Ispired from the MAX30102's Code 
*/

bool MAX30101::maxim_max30101_reset()

{
    if(!maxim_max30101_write_reg(REG_MODE_CONFIG,0x40))
        return false;
    else
        return true;    
}
