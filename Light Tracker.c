/* 
Light follower
Microcontroller: ATmege328P

The microcontroller is placed on an Arduino Uno board and through the USB on the PCB.
PlatformIO in Visual studio code was used to upload the code to the microntroller

*/

#include <avr/io.h>   
#include <util/delay.h> 

// Defining constants /////////////////////////////////////////////////////////
#define RIGHT_LT_PIN    PC0 // Arduino pin A0
#define LEFT_LT_PIN     PC1 // Arduino pin A1
#define DOWN_LT_PIN     PC2 // Arduino pin A2
#define UP_LT_PIN       PC3 // Arduino pin A3

#define HORI_SERVO_PIN PB1 // Arduino pin 9, OC1A PWM-controller
#define VERT_SERVO_PIN PB2 // Arduino pin 10, OC1B PWM-controller

//Defining values related to PWM period.
#define PWM_TOP (39999u) // Equivalent to 20ms period

// The servo we're using has a low duty cycle
#define SERVO_MIN (1999u)  // Equivalent to 1ms duty cycle
#define SERVO_MAX (4999u)  // equivalent to 2.5ms duty cycle


/////////// Function that initializes the ADC/////////////////////////////////////////
void adcInit(){ // 
    ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // [0x7A] ADEN: Activates the ADC, ADPS(2-0): Sets the prescaler to 128 to lower the ADC frequency
    ADMUX |= (1 << REFS0);                                              // [0x7C] Sets Vref to Vcc with capacitor at AREF pin of Atmega328P
}

/////////// Function that reads the ADC, the argument is the PIN you wish to read/////////////////////////////////////////
int adcRead(int analogInPin) {
    int analogVal = 0;

    ADMUX &= 0b11110000;            // [0x7C] Resets the 4 first bits in the ADMUX register (which are the bits that decide which pin to read from)
    ADMUX |= (analogInPin & 0x0F);  // [0x7C] Sets the lower most bits of the register to the pin we wish to read (with a mask to ensure we dont affect the highest 4 bits)

    ADCSRA |= (1 << ADSC);          // [0x7A] Start the AD conversion.

    while (ADCSRA & (1 << ADSC));   // While loop that runs while the AD conversion flag is active

    analogVal = ADCL;               // The ADC gives a 10-bit value, so we need to read the lowest 8 bits first from the ADCL register
    analogVal |= (ADCH << 8);       // Then read the 2 highest bits from the ADCH register and combine them with the lower 8 bits from the ADCL register.

    return analogVal;  // The function returns the final 10-bit value.
}

/////////// Function that initilializes PWM /////////////////////////////////////////
void pwmInit(void){
    DDRB |= (1 << HORI_SERVO_PIN) | (1 << VERT_SERVO_PIN); // [0x04] DDRB: Sets the pins used for the servos to output.

    // ICR1x: Defines the top value of the PWM. The timer is reset once it hits this value, its 16 bit so its contained in two registers
    ICR1H = (PWM_TOP & 0xFF00) >> 8; // [0x87]
    ICR1L = (PWM_TOP & 0x00FF);      // [0x86]
    
    // TCCR1x: Timer1 control register x. 
    TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11); // [0x80] COM1x1 = 1: Chooses compare mode for OCR1A/B, WGM11 along with WGM12 and 13 selects PWM mode (mode 14)
    TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11);    // [0x81] CS(11) chooses the prescaler (8 in this project)
}

/////////// Function that decides the horizontal servo position /////////////////////////////////////////
void servoSetHori(uint16_t deg, uint16_t max_deg){
    float set = (float)deg / (float)max_deg;                            

    set = (((float)SERVO_MAX-(float)SERVO_MIN)*set) + (float)SERVO_MIN; //Calculates from angle ratio to angle we can set in the OCR1A register

    uint16_t point = (uint16_t)set;                                     //Converts data type

    // Writes the value to the OCR1A register
    OCR1AH = (point & 0xFF00) >> 8;
    OCR1AL = (point & 0x00FF);
}

/////////// Function that decides the vertical servo position /////////////////////////////////////////
void servoSetVert(uint16_t deg1, uint16_t max_deg1){    //This function is the same as the previous one so just look at that one
    float set1 = (float)deg1 / (float)max_deg1;                           

    set1 = (((float)SERVO_MAX-(float)SERVO_MIN)*set1) + (float)SERVO_MIN; 

    uint16_t point1 = (uint16_t)set1;                                     

    OCR1BH = (point1 & 0xFF00) >> 8;
    OCR1BL = (point1 & 0x00FF);
}


int main(){

    //Initializing variables for comparing values read by ADC where phototransistors are connected
    int left;
    int right;
    int up;
    int down;

    //Initizlising servo position variables
    float horiAngle = 90;
    float vertAngle = 90;

    adcInit();
    pwmInit();

    _delay_ms(1000);

    // Endless loop //////////////////////////
    while(1){
        
        // LEFT RIGHT Controls horizontal servo ////////////////////////
        left = adcRead(LEFT_LT_PIN);
        right = adcRead(RIGHT_LT_PIN);

        int horiVal = left - right;         // Makes a delta to check if the servo needs to rotate

        if(horiVal > 70){
            horiAngle = horiAngle + 0.2;
        }
        
        if(horiVal < -70){
            horiAngle = horiAngle - 0.2;
        }

        if(horiAngle > 180){                // Make a max value for the servo to prevent overflow
            horiAngle = 180;
        }

        if(horiAngle < 0){                  // Same as above just for a min value
            horiAngle = 0;
        }

        servoSetHori(horiAngle, 180);       //Sets the horizontal servo angle

        // UP DOWN Controls vertical servo ///////////////////////
        up = adcRead(UP_LT_PIN);
        down = adcRead(DOWN_LT_PIN); // Same logic as above just for the vertical servo

        int vertiVal = up-down;
        
        if(vertiVal > 70){ 
            vertAngle = vertAngle + 0.2;

        }
        
        if(vertiVal < -70){
            vertAngle = vertAngle - 0.2;
        }

        if(vertAngle > 90){
            vertAngle = 90;
        }

        if(vertAngle < 0){
            vertAngle = 0;
        }

        servoSetVert(vertAngle, 180);

        _delay_us(800);                     // Delay for stability

    }
}


