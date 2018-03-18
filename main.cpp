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

#define SCREEN_2    2

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
   
void StartHaptic(void);
void StopHaptic(void const *n);

DigitalOut redLed(LED1);
DigitalOut greenLed(LED2);
DigitalOut blueLed(LED3);
DigitalOut haptic(PTB9);

/* Define timer for haptic feedback */
RtosTimer hapticTimer(StopHaptic, osTimerOnce);

/* Instantiate the Hexi KW40Z Driver (UART TX, UART RX) */ 
KW40Z kw40z_device(PTE24, PTE25);
/* Instantiate the SSD1351 OLED Driver */ 
SSD1351 oled(PTB22,PTB21,PTC13,PTB20,PTE6, PTD15);

int SCREEN=0;      /*variable to make test on the screen*/
 
int tens=0, units=0; /*variable to set the threshold*/

int press=0;   /*variable to count number of pressing*/

char text[20];  /* Text Buffer */ 


volatile int UP=0;
volatile int DOWN=0;
volatile int RIGHT=0;
volatile int LEFT=0;
volatile int SLIDE=0;


void ButtonUp(void)
{
    StartHaptic();
    
    UP=1;
    
    redLed      = LED_ON;
    greenLed    = LED_OFF;
    blueLed     = LED_OFF;
}

void ButtonDown(void)
{
    StartHaptic();
    
    DOWN=1;
    
    redLed      = LED_OFF;
    greenLed    = LED_ON;
    blueLed     = LED_OFF;
}

void ButtonRight(void)
{
    StartHaptic();
    
    RIGHT=1;
    
    redLed      = LED_OFF;
    greenLed    = LED_OFF;
    blueLed     = LED_ON;
}

void ButtonLeft(void)
{
    StartHaptic();
    
    LEFT=1;
    
    redLed      = LED_ON;
    greenLed    = LED_ON;
    blueLed     = LED_OFF;
}

void ButtonSlide(void)
{
    StartHaptic();
    
    SLIDE=1;
    
    redLed      = LED_ON;
    greenLed    = LED_ON;
    blueLed     = LED_ON;
}

void assertinterrupt(void) //L'ordre est important, quand elle était declarée a la fin, le programme n'a pas marché.
{
     UP=0;
     DOWN=0;
     RIGHT=0;
     LEFT=0;
     SLIDE=0;
}
   
int main()
{
    
    /* Get OLED Class Default Text Properties */
    oled_text_properties_t textProperties = {0};
    oled.GetTextProperties(&textProperties);    

    /* Turn on the backlight of the OLED Display */
    oled.DimScreenON();
    
    /* Fills the screen with solid black */         
    oled.FillScreen(COLOR_BLACK);
    
    textProperties.fontColor   = COLOR_BLUE;
    oled.SetTextProperties(&textProperties);
    
    strcpy((char *) text,"set your");
    oled.Label((uint8_t *)text,20,12);   
    /* Display Text at (x=7,y=0) */
    strcpy((char *) text,"BPM threshold");
    oled.Label((uint8_t *)text,5,32);
    
    textProperties.fontColor   = COLOR_GREEN;
    oled.SetTextProperties(&textProperties);
    
    sprintf(text,"%i",tens);
    oled.TextBox((uint8_t *)text,35,50,10,15);
    
    textProperties.fontColor   = COLOR_GREEN;
    oled.SetTextProperties(&textProperties); 
       
    sprintf(text,"%i",units);
    oled.TextBox((uint8_t *)text,45,50,10,15);    
    
    /* Register callbacks to application functions */
    kw40z_device.attach_buttonUp(&ButtonUp);
    kw40z_device.attach_buttonDown(&ButtonDown);
    kw40z_device.attach_buttonLeft(&ButtonLeft);
    kw40z_device.attach_buttonRight(&ButtonRight);
    kw40z_device.attach_buttonSlide(&ButtonSlide);
    /* Register callbacks to application functions */
    kw40z_device.attach_buttonUp(&ButtonUp);
    kw40z_device.attach_buttonDown(&ButtonDown);
    kw40z_device.attach_buttonLeft(&ButtonLeft);
    kw40z_device.attach_buttonRight(&ButtonRight);
    kw40z_device.attach_buttonSlide(&ButtonSlide);
    
    while (true) {
        
        if(UP==1){
                assertinterrupt();
                if(SCREEN==SCREEN_2)
                    {
       
                    }        
                else    
                 {
                    tens=tens+1;
    
                    //textProperties.fontColor   = COLOR_RED;
                    //oled.SetTextProperties(&textProperties); 
       
                     sprintf(text,"%i",tens);
                        oled.TextBox((uint8_t *)text,35,50,10,15);           
                 }
        }

        if(DOWN==1){
            assertinterrupt();
            if(SCREEN == SCREEN_2)
            {
            }  
         
            else
            {        
                if(tens>0)
                tens=tens-1;
                else 
                tens=0;
                //textProperties.fontColor   = COLOR_RED;
                //oled.SetTextProperties(&textProperties); 
       
                sprintf(text,"%i",tens);
                oled.TextBox((uint8_t *)text,35,50,10,15);             
            }
        }

        if(RIGHT==1){
            assertinterrupt();
            if(SCREEN == SCREEN_2)
                { 
    
                    /*configuration for MAX30101*/
                     powerEN1 = 1;  
                     powerEN2 = 0;         
                     powerEN3 = 1; 

                     uint32_t un_min, un_max, un_prev_data;  //variables to calculate the on-board LED brightness that reflects the heartbeats
                     int i;
                     int32_t n_brightness;
                     float f_temp;
    
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

                     strcpy((char *) text,"Heart Rate <3");
                     oled.Label((uint8_t *)text,10,10);    
      
        
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

                     ////txThread.start(txTask); /*Start transmitting Sensor Tag Data */

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
                             if(n_heart_rate>70)
                             {
                                 n_heart_rate=69;
                                 }
                             else if (n_heart_rate<45) 
                             {
                                 n_heart_rate=45;
                             }
                             else
                             {}   
                             //send samples and calculation result to terminal program through UART
                             textProperties.fontColor= COLOR_BLUE;
                             oled.SetTextProperties(&textProperties);
    
                             strcpy(text,"HR=");
                             oled.Label((uint8_t *)text,15,40);

                             strcpy(text,"bpm");
                             oled.Label((uint8_t *)text,65,40);
                
                             //sprintf(text,"0x%02x",max30101.who_I_am());
                             sprintf(text,"%i",n_heart_rate);
                             oled.TextBox((uint8_t *)text,35,40,30,15); 
                       

                         }
                         maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);

                         ////blueLed = !kw40z_device.GetAdvertisementMode(); /*Indicate BLE Advertisment Mode*/  
                         Thread::wait(50);

                     }
        
                         }  
         
                     else
                     {     
                     if(units<9)
                     units=units+1;
                     else
                     units=9;
    
                     //textProperties.fontColor   = COLOR_GREEN;
                     //oled.SetTextProperties(&textProperties); 
       
                     sprintf(text,"%i",units);
                     oled.TextBox((uint8_t *)text,45,50,10,15);    
                     redLed      = LED_OFF;
                     greenLed    = LED_OFF;
                     blueLed     = LED_ON;
                     }
            
            
            }
        
        if(LEFT==1){
            assertinterrupt();
                switch (press){
        
                case 2:
               press=0;
               SCREEN =SCREEN_2;
        
                oled.DimScreenON();    
                /* Fills the screen with solid black */         
               oled.FillScreen(COLOR_BLACK);
              //textProperties.fontColor   = COLOR_BLUE;
              //oled.SetTextProperties(&textProperties);    
             strcpy((char *) text,"Threshold Set");
               oled.Label((uint8_t *)text,12,12);
        
            strcpy((char *) text,"Tap Below to");
            oled.Label((uint8_t *)text,10,55);
        
            strcpy((char *) text,"go to HR Mode");
            oled.Label((uint8_t *)text,7,75);        
        
            break;
        
           default:          
           press+=1;
            
            }
        }
        if(SLIDE==1){
            assertinterrupt();
            
            }
                    
        Thread::wait(500);
    }
}

void StartHaptic(void)
{
    hapticTimer.start(50);
    haptic = 1;
}

void StopHaptic(void const *n) {
    haptic = 0;
    hapticTimer.stop();
}
