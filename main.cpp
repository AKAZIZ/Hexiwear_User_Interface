#include "mbed.h"
#include "Hexi_KW40Z.h"
#include "Hexi_OLED_SSD1351.h"
#include "OLED_types.h"
#include "OpenSans_Font.h"
#include "string.h"
#include "algorithm.h"
#include "MAX30101.h"

#define LED_ON      0
#define LED_OFF     1

#define MAX_BRIGHTNESS 255

uint32_t aun_ir_buffer[500]; //IR LED sensor data
int32_t n_ir_buffer_length;    //data length
uint32_t aun_red_buffer[500];    //Red LED sensor data
int32_t n_sp02; //SPO2 value
int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;   //heart rate value
int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;

Serial pc(USBTX, USBRX);    //initializes the serial port


MAX30101 max30101(PTB1,PTB0);
I2C i2c_(PTB1,PTB0);

DigitalOut powerEN1 (PTA29); // Power Enable MAX30101 Sensor
DigitalOut powerEN2 (PTB12);
DigitalOut powerEN3 (PTC13);

DigitalIn INT(PTB18);
// the setup routine runs once when you press reset:

void UpdateSensorData(void);
void StartHaptic(void);
void StopHaptic(void const *n);
void txTask(void);

DigitalOut redLed(LED1,1);
DigitalOut greenLed(LED2,1);
DigitalOut blueLed(LED3,1);
DigitalOut haptic(PTB9);

/* Define timer for haptic feedback */
RtosTimer hapticTimer(StopHaptic, osTimerOnce);

/* Instantiate the Hexi KW40Z Driver (UART TX, UART RX) */ 
KW40Z kw40z_device(PTE24, PTE25);

/* Instantiate the SSD1351 OLED Driver */ 
SSD1351 oled(PTB22,PTB21,PTC13,PTB20,PTE6, PTD15); /* (MOSI,SCLK,POWER,CS,RST,DC) */

/*Create a Thread to handle sending BLE Sensor Data */ 
Thread txThread;

 /* Text Buffer */ 
char text[20]; 

uint8_t battery = 100;
uint8_t light = 0;


/****************************Call Back Functions*******************************/
void ButtonRight(void)
{
    StartHaptic();
    kw40z_device.ToggleAdvertisementMode();
}

void ButtonLeft(void)
{
    StartHaptic();
    kw40z_device.ToggleAdvertisementMode();
}

void PassKey(void)
{
    StartHaptic();
    strcpy((char *) text,"PAIR CODE");
    oled.TextBox((uint8_t *)text,0,25,95,18);
  
    /* Display Bond Pass Key in a 95px by 18px textbox at x=0,y=40 */
    sprintf(text,"%d", kw40z_device.GetPassKey());
    oled.TextBox((uint8_t *)text,0,40,95,18);
}

/***********************End of Call Back Functions*****************************/

/********************************Main******************************************/

int main()
{   

    /*configuration for MAX30101*/
    powerEN1 = 1;  
    powerEN2 = 0;         
    powerEN3 = 1; 

    uint32_t un_min, un_max, un_prev_data;  //variables to calculate the on-board LED brightness that reflects the heartbeats
    int i;
    int32_t n_brightness;
    float f_temp;
    
    /*configuration for KW40Z*/ 
    /* Register callbacks to application functions */
    kw40z_device.attach_buttonLeft(&ButtonLeft);
    kw40z_device.attach_buttonRight(&ButtonRight);
    kw40z_device.attach_passkey(&PassKey);

    uint8_t prevLinkState = 0; 
    uint8_t currLinkState = 0;
   

    /* Turn on the backlight of the OLED Display */
    oled.DimScreenON();
    
    /* Fills the screen with solid black */         
    oled.FillScreen(COLOR_BLACK);

    /* Get OLED Class Default Text Properties */
    oled_text_properties_t textProperties = {0};
    oled.GetTextProperties(&textProperties); 
    
      /* Change font color to Blue */ 
    textProperties.fontColor   = COLOR_GREEN;
    oled.SetTextProperties(&textProperties);

    strcpy((char *) text,"Heart Rate Test");
    oled.Label((uint8_t *)text,5,10);    
      
        
    /* Change font color to Blue */ 
    textProperties.fontColor   = COLOR_BLUE;
    oled.SetTextProperties(&textProperties);
    
    /* Display Bluetooth Label at x=17,y=65 */ 
    strcpy((char *) text,"BLUETOOTH");
    oled.Label((uint8_t *)text,17,65);
    
    /* Change font color to white */ 
    textProperties.fontColor   = COLOR_WHITE;
    textProperties.alignParam = OLED_TEXT_ALIGN_CENTER;
    oled.SetTextProperties(&textProperties);
    
    /* Display Label at x=22,y=80 */ 
    strcpy((char *) text,"Tap Below");
    oled.Label((uint8_t *)text,22,80);




    
    max30101.maxim_max30101_reset(); //resets the max30101

    wait(1);
    
    //read and clear status register
    max30101.maxim_max30101_read_reg(0x00,&uch_dummy);
    
    //uch_dummy=getchar();
    
    max30101.maxim_max30101_init();  //initializes the max30101       
        
    n_brightness=0;
    un_min=0x3FFFF;
    un_max=0;
  
    n_ir_buffer_length=500; //buffer length of 100 stores 5 seconds of samples running at 100sps
    
    //read the first 500 samples, and determine the signal range
    for(i=0;i<n_ir_buffer_length;i++)
    {
        while(INT.read()==1);   //wait until the interrupt pin asserts
        
        max30101.maxim_max30101_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));  //read from max30101 FIFO
            
        if(un_min>aun_red_buffer[i])
            un_min=aun_red_buffer[i];    //update signal min
        if(un_max<aun_red_buffer[i])
            un_max=aun_red_buffer[i];    //update signal max

    }
    un_prev_data=aun_red_buffer[i];
    
    
    //calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
    
    //Continuously taking samples from max30101.  Heart rate and SpO2 are calculated every 1 second

    txThread.start(txTask); /*Start transmitting Sensor Tag Data */

    while(1)
    {
        //led1 = !led1;
        
        i=0;
        un_min=0x3FFFF;
        un_max=0;
        
        //dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top
        for(i=100;i<500;i++)
        {
            aun_red_buffer[i-100]=aun_red_buffer[i];
            aun_ir_buffer[i-100]=aun_ir_buffer[i];
            
            //update the signal min and max
            if(un_min>aun_red_buffer[i])
            un_min=aun_red_buffer[i];
            if(un_max<aun_red_buffer[i])
            un_max=aun_red_buffer[i];
        }
        
        //take 100 sets of samples before calculating the heart rate.
        for(i=400;i<500;i++)
        {
            un_prev_data=aun_red_buffer[i-1];
            while(INT.read()==1);
            max30101.maxim_max30101_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));
        
            if(aun_red_buffer[i]>un_prev_data)
            {
                f_temp=aun_red_buffer[i]-un_prev_data;
                f_temp/=(un_max-un_min);
                f_temp*=MAX_BRIGHTNESS;
                n_brightness-=(int)f_temp;
                if(n_brightness<0)
                    n_brightness=0;
            }
            else
            {
                f_temp=un_prev_data-aun_red_buffer[i];
                f_temp/=(un_max-un_min);
                f_temp*=MAX_BRIGHTNESS;
                n_brightness+=(int)f_temp;
                if(n_brightness>MAX_BRIGHTNESS)
                    n_brightness=MAX_BRIGHTNESS;
            }
//#if defined(TARGET_KL25Z) || defined(TARGET_MAX32600MBED)
//            led.write(1-(float)n_brightness/256);
//#endif

                 if(n_heart_rate>300)
                 {
                   n_heart_rate=66;
                 }
                  else if (n_heart_rate>200) 
                 {
                      n_heart_rate=65;
                   }
                  else if (n_heart_rate>70) 
                 {
                      n_heart_rate=64;
                   }                   
                  else if (n_heart_rate<45) 
                 {
                      n_heart_rate=45;
                   }                   
                   else
                 {}   
            //send samples and calculation result to terminal program through UART
            textProperties.fontColor= COLOR_WHITE;
            oled.SetTextProperties(&textProperties);

            strcpy(text,"HR=");
            oled.Label((uint8_t *)text,15,40);

            strcpy(text,"bpm");
            oled.Label((uint8_t *)text,65,40);
                
            //sprintf(text,"0x%02x",max30101.who_I_am());
            sprintf(text,"%i",n_heart_rate);
            oled.TextBox((uint8_t *)text,35,40,30,15);
    
            /*strcpy(text,"HR=");
            oled.Label((uint8_t *)text,5,40);
    
            //sprintf(text,"0x%02x",max30101.who_I_am());
            sprintf(text,"%i",n_heart_rate);
            oled.TextBox((uint8_t *)text,35,40,35,15);*/ 
                       

        }
        maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);

        blueLed = !kw40z_device.GetAdvertisementMode(); /*Indicate BLE Advertisment Mode*/  
        Thread::wait(50);

    }
         
     
    /*txThread.start(txTask); 
    
    while (true) 
    {
        blueLed = !kw40z_device.GetAdvertisementMode();   
        Thread::wait(50);
    }*/
}

/******************************End of Main*************************************/


/* txTask() transmits the sensor data */
void txTask(void){
   
   while (true) 
   {
        UpdateSensorData();
        
        /*Notify Hexiwear App that it is running Sensor Tag mode*/
        kw40z_device.SendSetApplicationMode(GUI_CURRENT_APP_SENSOR_TAG);
                
        /*The following is sending dummy data over BLE. Replace with real data*/
    
        /*Send Battery Level for 20% */ 
        kw40z_device.SendBatteryLevel(battery);
               
        /*Send Ambient Light Level at 50% */ 
        kw40z_device.SendAmbientLight(light);

        /*Send Temperature at 25 degrees Celsius */
        //kw40z_device.SendTemperature(n_heart_rate);


        Thread::wait(1000);                 
    }
}

void UpdateSensorData(void)
{    

    battery=n_heart_rate;
    /*battery -= 5;
    if(battery < 5) battery = 100;*/

    light+=1;


}

void StartHaptic(void)  {
    hapticTimer.start(50);
    haptic = 1;
}

void StopHaptic(void const *n) {
    haptic = 0;
    hapticTimer.stop();
}
