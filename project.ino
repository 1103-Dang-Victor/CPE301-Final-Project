// Code Contributors: Victor Dang, Gianna Everest, Anthony Felix, Jordan Imgard
// CPE 301 Group 13

// LCD Screen: Pins 22, 23, 24, 25, 26, 27
// Stepper Motor: Pins 2, 3, 4, 5
// Green LED: Pin 7 (PH4)
// Yellow LED: Pin 8 (PH5)
// Red LED: Pin 9 (PH6)
// Blue LED (Fan): Pin 10 (PB4)
// DHT11 Sensor: Pin 6
// Water Sensor: Pin A0
// Start Button: Pin 19 (PD2 - Interrupt)
// Vent Button Left: Pin A12 (PK4)
// Vent Button Right: Pin A13 (PK5)
// RTC: Pins 20 (SDA), 21 (SCL)

#include <LiquidCrystal.h>
#include <Stepper.h>
#include <DHT.h>
#include <RTClib.h>
#include <Wire.h>

#define RDA 0x80
#define TBE 0x20

// UART Pointers
volatile unsigned char *myUCSR0A  = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B  = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C  = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0   = (unsigned int *)0x00C4;
volatile unsigned char *myUDR0    = (unsigned char *)0x00C6;

// GPIO Pointers
volatile unsigned char *portB     = (unsigned char *)0x25;
volatile unsigned char *ddr_b  = (unsigned char *)0x24;
unsigned char* ddr_h = (unsigned char*) 0x101;
unsigned char* port_h = (unsigned char*) 0x102;

//Port K Pointers
volatile unsigned char *port_k = (unsigned char *)0x108;
volatile unsigned char *ddr_k  = (unsigned char *)0x107;
volatile unsigned char *pin_k  = (unsigned char *)0x106;

//Timer Pointers
volatile unsigned char *myTCCR1A  = (unsigned char *)0x80;
volatile unsigned char *myTCCR1B  = (unsigned char *)0x81;
volatile unsigned char *myTCCR1C  = (unsigned char *)0x82;
volatile unsigned char *myTIMSK1  = (unsigned char *)0x6F;
volatile unsigned char *myTIFR1   = (unsigned char *)0x36;
volatile unsigned int  *myTCNT1   = (unsigned int *)0x84;

//Port A Pointers 
volatile unsigned char *port_a    = (unsigned char *)0x22;
volatile unsigned char *ddr_a     = (unsigned char *)0x21;
volatile unsigned char *pin_a     = (unsigned char *)0x20; 

//Port D Pointers
volatile unsigned char *port_d = (unsigned char *)0x2B;
volatile unsigned char *ddr_d  = (unsigned char *)0x2A;
volatile unsigned char *pin_d  = (unsigned char *)0x29;

//ADC Pointers (Banned library replacement)
volatile unsigned char *my_ADMUX  = (unsigned char *)0x7C;
volatile unsigned char *my_ADCSRA = (unsigned char *)0x7A;
volatile unsigned char *my_ADCSRB = (unsigned char *)0x7B;
volatile unsigned int  *my_ADC_DATA = (unsigned int *)0x78;

//DHT Pin
#define DHTPIN 6
//DHT Type
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

#define BTN_LEFT  0b00010000
#define BTN_RIGHT 0b00100000

//RTC
RTC_DS1307 rtc; 

//Hardware Constants
LiquidCrystal lcd(22, 23, 24, 25, 26, 27);

//Stepper Motor
const int stepsPerRev = 2038; 
Stepper myStepper(stepsPerRev, 2, 4, 3, 5);

//Thresholds
const int TEMP_THRESHOLD = 24;
const int WATER_THRESHOLD = 100; //Adjust based on sensor calibration

//State Variables
enum State {
  DISABLED,
  IDLE,
  RUNNING,
  ERROR
};

volatile State currentState = DISABLED;
//Timing Variables
unsigned long lastEnvUpdate = 0;
volatile unsigned long last_isr_time = 0;

//ISR Flag
volatile bool start_request = false;

//Function Prototypes
void U0Init(unsigned long U0baud);
void U0putstr(const char *string);
unsigned int adc_read(unsigned char adc_channel_num);
void adc_init();
void updateLEDs();
void transitionTo(State nextState);
void monitorEnvironment();
void checkWaterLevel();
void controlVent();
void startISR();

//setup all required functions, timers, lcd, motors, etc.
void setup() 
{

    //1. Setup LEDs (Output)
    *ddr_h |= 0b01110000; //Red, Yellow, Green
    *ddr_b |= 0b00010000; //Blue

    //2. Setup Vent Buttons (Port K)
    *ddr_k &= ~(BTN_LEFT);
    *ddr_k &= ~(BTN_RIGHT);
    *port_k |= BTN_LEFT;   //enable pull-up
    *port_k |= BTN_RIGHT;  //enable pull-up

    //3. Setup Start Button (Port D, Pin 19 is PD2), we must ensure PD2 is input and pulled up
    *ddr_d &= 0b11111011;  //Force Bit 2 to Input
    *port_d |= 0b00000100; //Force Bit 2 Pull-up

    //4. Initialize Peripherals
    U0Init(9600);
    adc_init();
    dht.begin();
    lcd.begin(16, 2);
    rtc.begin();
    
    //5. Attach Interrupts
    attachInterrupt(digitalPinToInterrupt(19), startISR, FALLING);

    //Initial State
    updateLEDs();
}

void loop()
{
	//Handle State Transitions and Logic
    if (start_request) {
        start_request = false; //Reset flag
        if (currentState == DISABLED) {
            transitionTo(IDLE);
        } else {
            transitionTo(DISABLED);
        }
    }
    switch (currentState) {
        case DISABLED:
            //Yellow LED ON. No monitoring.
            //Start button monitored via ISR
            break;
        case IDLE:
            //Green LED ON. Monitor Water & Temp.
            monitorEnvironment();
            checkWaterLevel();
            controlVent();
            //Logic: If Temp > Threshold -> RUNNING
            if (dht.readTemperature() > TEMP_THRESHOLD) {
                transitionTo(RUNNING);
            }
            break;
        case RUNNING:
            //Blue LED ON. Fan ON. Monitor Water & Temp.
            monitorEnvironment();
            checkWaterLevel();
            controlVent();
            //Logic: If Temp <= Threshold -> IDLE
            if (dht.readTemperature() <= TEMP_THRESHOLD) {
                transitionTo(IDLE);
            }
            break;
        case ERROR:
            //Red LED ON. Motor OFF.
            //Display Error Message
            lcd.setCursor(0, 0);
            lcd.print("Water Low");
            lcd.setCursor(0, 1);
            lcd.print("Error State");
           	if (adc_read(0) > WATER_THRESHOLD) {
    			lcd.print("Level OK-Press Btn"); 
			} else {
    			lcd.print("Error State");
			}
            break;
    }  
    //Refresh LEDs to match state
    updateLEDs();
}

void transitionTo(State nextState) {
    DateTime now = rtc.now();
    char buffer[50];
    sprintf(buffer, "State: %d -> %d at %02d:%02d:%02d\n", currentState, nextState, now.hour(), now.minute(), now.second());
    U0putstr(buffer);
    currentState = nextState;
    lcd.clear();
}

void U0putstr(const char *string) {
    for (int i = 0; string[i] != '\0'; i++){
        putChar(string[i]);
    }
}

void updateLEDs() {
    //Turn off all LEDs first
    *port_h &= 0b10001111; 
    *port_b &= 0b11101111; 

    switch (currentState) {
        case DISABLED:
            *port_h |= 0b00100000; //Yellow ON
            break;
        case IDLE:
            *port_h |= 0b00010000; //Green ON
            break;
        case RUNNING:
            *port_b |= 0b00010000; //Blue ON
            break;
        case ERROR:
            *port_h |= 0b01000000; //Red ON
            break;
    }
}

void monitorEnvironment() {
    if (millis() - lastEnvUpdate > 1000) { 
        lastEnvUpdate = millis();
        float h = dht.readHumidity();
        float t = dht.readTemperature();
        lcd.setCursor(0, 0);
        lcd.print("Temp: "); lcd.print(t); lcd.print("C");
        lcd.setCursor(0, 1);
        if (h > 70.0) {  
             lcd.print("High Humidity!  ");
        } else {
             lcd.print("Humid: "); lcd.print(h); lcd.print("%  "); 
        }
    }
}

void checkWaterLevel() {
    unsigned int waterLevel = adc_read(0); 
    if (waterLevel <= WATER_THRESHOLD) {
        transitionTo(ERROR);
    }
}

void controlVent() {
    if ((*pin_k & BTN_LEFT) == 0) {  
        myStepper.step(-10);
        return;
    }
    if ((*pin_k & BTN_RIGHT) == 0) {  
        myStepper.step(10);
        return;
    }
}

void startISR() {
    if (millis() - last_isr_time > 200) {
        start_request = true;
        last_isr_time = millis();
    }
}

//ADC Functions
void adc_init() {
    *my_ADCSRA |= 0b10000000; 
    *my_ADCSRA &= 0b11011111; 
    *my_ADCSRA &= 0b11110111; 
    *my_ADCSRA &= 0b11111000; 
    *my_ADCSRB &= 0b11110111; 
    *my_ADCSRB &= 0b11111000;
    *my_ADMUX &= 0b01111111; 
    *my_ADMUX |= 0b01000000;
    *my_ADMUX &= 0b11011111; 
    *my_ADMUX &= 0b11100000; 
}

unsigned int adc_read(unsigned char adc_channel_num) {
    *my_ADMUX &= 0b11100000; 
    *my_ADCSRB &= 0b11110111;
    if (adc_channel_num > 7) {
        adc_channel_num -= 8;
        *my_ADCSRB |= 0b00001000;
    }
    *my_ADMUX += adc_channel_num;
    *my_ADCSRA |= 0x40; 
    while ((*my_ADCSRA & 0x40) != 0); 
    return *my_ADC_DATA;
}

/*
 * UART FUNCTIONS
*/
void U0Init(unsigned long U0baud)
{
	unsigned long FCPU = 16000000;
	unsigned int tbaud;
	tbaud = (FCPU / 16 / U0baud - 1);
	*myUCSR0A = 0x20;
	*myUCSR0B = 0x18;
	*myUCSR0C = 0x06;
	*myUBRR0 = tbaud;
}

unsigned char kbhit() 
{
	return *myUCSR0A & RDA;
}

unsigned char getChar() 
{
	return *myUDR0;
}

void putChar(unsigned char U0pdata)
{
	while((*myUCSR0A & TBE) == 0);
  *myUDR0 = U0pdata;
}
