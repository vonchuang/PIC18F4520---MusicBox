
// CONFIG1H
#pragma config OSC = INTIO67    // Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
// CONFIG3H
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)
// CONFIG4L
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)

#include <p18f4520.h>

// Ref: http://www.phy.mtu.edu/~suits/notefreqs.html
float ToneFreq[] = {
    104.650, //C6
    117.466, //D6
    131.851, //E6
    139.691, //F6
    156.798, //G6
    176.000, //A6
    197.553, //B6
    209.300, //C7
    234.932, //D7
    263.702, //E7
    279.383, //F7
    313.596, //G7
    352.000, //A7
    395.107, //B7
    186.466, //B6b
    372.931, //B7b
    418.601, //C8
    -1 // End of array (if -1, then restart array index)
};

#define C6 ToneFreq[0]
#define D6 ToneFreq[1]
#define E6 ToneFreq[2]
#define F6 ToneFreq[3]
#define G6 ToneFreq[4]
#define A6 ToneFreq[5]
#define B6 ToneFreq[6]

#define C7 ToneFreq[7]
#define D7 ToneFreq[8]
#define E7 ToneFreq[9]
#define F7 ToneFreq[10]
#define G7 ToneFreq[11]
#define A7 ToneFreq[12]
#define B7 ToneFreq[13]
#define B6b ToneFreq[14]
#define B7b ToneFreq[15]
#define C8 ToneFreq[16]

#define silence ToneFreq[17]

#define Dunit1 35
#define Dunit2 70
#define Dunit4 105
#define Dunit8 140
#define cut 20

#define round(x) ((x) + 0.5)
#define FOSC (1e6) // 10MHz HS mode
#define PWM_PRESCALE (16)
#define PWM_DUTY_CYCLE (10) // Percentage of Duty Cycle

/* Calculation for Timer 0 as 1 sec
 *   10MHz clock, 256 prescaler, 8 bit mode
 *   Instruction freq = 10MHz / 4 = 2.5MHz
 *   Timer clock freq = 2.5MHz / 256 prescaler
 *   No of cycles = 1Hz / (2.5MHz / 256)^-1
 *   No of cycles = 2.5e6 / 256 = 9765.625
 *   Reset value = 65535 - 9765.625 = 55769.375
 *   Formula: 65535 - FOSC / 4 / PRESCALE 
 */ 
#define TIMER0_RESET_VAL (55769)

int array_index = 0;
float freq = 1046.50;

void main(void);
void ADCInit();
void Seru();
void TopOfTheWorld();
void delayMusic(int a, int b);
void setPWMFrequency(void);
void setPWMDutyCycleCCP2(int pwm_percentage);
void interrupt int0(void);
void resetTimer0OneSecond();

void setC6();
void setD6();
void setE6();
void setF6();
void setG6();
void setA6();
void setB6();

void setC7();
void setD7();
void setE7();
void setF7();
void setG7();
void setA7();
void setB7();

void setB6b();
void setB7b();
void setC8();

void setCUT();

unsigned int MyadcValue;

void main(void) {

    
    // Set up Timer 0
    INTCONbits.PEIE = 1; // enable peripheral interrupt
    INTCON2bits.TMR0IP = 1; // TMR0 high priority
    INTCONbits.TMR0IE = 1; // TMR0 interrupt enable
    T0CONbits.PSA = 0; // Assign prescaler
    T0CONbits.T0PS = 0b111; // Prescaler 1:256
    T0CONbits.T0CS = 0; // internal clock source
    T0CONbits.T08BIT = 0; // 0 = Timer0 is configured as a 16-bit timer/counter 
    T0CONbits.TMR0ON = 1; // Enable Timer 0
    
    // Set up external interrupt -> RB0, RB1 push button 
    TRISB = 3; // RB0, RB1 as input
    RCONbits.IPEN = 1;
    INTCONbits.INT0IE = 1; // INT0 enabled  
    INTCONbits.INT0IF = 0;
    INTCON2bits.INTEDG0 = 0; // Interrupt on falling edge
    INTCONbits.GIE = 1; // Enable global interrupts
    
    ADCInit();
    
    /****************************************************
    The following steps should be taken when configuring
    the CCP module for PWM operation:
    1. Set the PWM period by writing to the PR2 register.
    2. Set the PWM duty cycle by writing to the
    CCPRxL register and CCPxCON<5:4> bits.
    3. Make the CCPx pin an output by clearing the
    appropriate TRIS bit.
    4. Set the TMR2 prescale value, then enable
    Timer2 by writing to T2CON.
    5. Configure the CCPx module for PWM operation.
    ****************************************************/
    
    // 1. Set the PWM period by writing to the PR2 register.
    setPWMFrequency();
    
    // 2. Set the PWM duty cycle by writing to the CCPRxL register and CCPxCON<5:4> bits.
    setPWMDutyCycleCCP2(0); // Set PWM duty cycle to 0% on first boot
    
    // 3. Make the CCPx pin an output by clearing the appropriate TRIS bit.
    // Set RC1 to output: RC1/T1OSI/CCP2(1)
    TRISCbits.RC1 = 0;
    
    // 4. Set up TMR2
    T2CONbits.T2CKPS = 0b10; // 1x = Prescaler is 16 
    T2CONbits.TMR2ON = 1; // 1 = Timer2 is on
    
    // 5. Configure the CCPx module for PWM operation.
    CCP2CONbits.CCP2M = 0b1100; // CCP2 as PWM mode -> 11xx = PWM mode
    
    //TRISDbits.RD0 = 0;
    //int buton = PORTB & 0x01;
    while (1) {
        // play song 1 -seru
        if((PORTB & 0x01) == 0){
            Seru();
        }
        
        // play song 2 - country word
        if((PORTB & 0x02) == 0){
            TopOfTheWorld();
        }
        
        if(ADCON0bits.GO_DONE == 0){
            ADCON0bits.GO_DONE = 1;
            MyadcValue = ADRESH << 8 + ADRESL << 6;
            
            if((ADRESH == 1) && ((ADRESL >=253) && (ADRESL <=255))){    //F
                freq = ToneFreq[10];
            }else if((ADRESH == 1) && ((ADRESL >=246) && (ADRESL <=247))){    //E
                freq = ToneFreq[11];
            }else if((ADRESH == 1) && ((ADRESL >=238) && (ADRESL <=240))){    //D
                freq = ToneFreq[12];
            }else if((ADRESH == 1) && ((ADRESL >=231) && (ADRESL <=232))){    //C
                freq = ToneFreq[13];
            }else if((ADRESH == 1) && ((ADRESL >=242) && (ADRESL <=243))){    //B
                //freq = ToneFreq[3];
            }else if((ADRESH == 1) && ((ADRESL >=234) && (ADRESL <=235))){    //3
                freq = ToneFreq[7];
            }else if((ADRESH == 1) && ((ADRESL >=227) && (ADRESL <=228))){    //6
                freq = ToneFreq[8];
            }else if((ADRESH == 1) && ((ADRESL >=220) && (ADRESL <=221))){    //9
                freq = ToneFreq[9];
            }else if((ADRESH == 1) && ((ADRESL >=229) && (ADRESL <=230))){    //A
                freq = ToneFreq[3];
            }else if((ADRESH == 1) && ((ADRESL >=222) && (ADRESL <=224))){    //2
                freq = ToneFreq[4];
            }else if((ADRESH == 1) && ((ADRESL >=216) && (ADRESL <=217))){    //5
                freq = ToneFreq[5];
            }else if((ADRESH == 1) && ((ADRESL >=209) && (ADRESL <=211))){    //8
                freq = ToneFreq[6];
            }else if((ADRESH == 1) && ((ADRESL >=218) && (ADRESL <=219))){    //0
                //freq = ToneFreq[3];
            }else if((ADRESH == 1) && ((ADRESL >=212) && (ADRESL <=213))){    //1
                freq = ToneFreq[0];
            }else if((ADRESH == 1) && ((ADRESL >=205) && (ADRESL <=206))){    //4
                freq = ToneFreq[1];
            }else if((ADRESH == 1) && ((ADRESL >=198) && (ADRESL <=201))){    //7
                freq = ToneFreq[2];
            }else{
                freq = ToneFreq[14];
                setPWMDutyCycleCCP2(0);
                continue;
            }
            
            setPWMFrequency();
            setPWMDutyCycleCCP2(PWM_DUTY_CYCLE);
            resetTimer0OneSecond();
        }    
    }
}

void ADCInit()
{
    ADCON1bits.VCFG1 = 0 ;  //setting vref- //Vss
    ADCON1bits.VCFG0 = 0 ;  //setting vref+ //VDD
    ADCON1bits.PCFG  = 0xE ;  //Setting A/D Port Configuration Control  //1110 = 2+4+8=14=E //only AN0 is Analog
    ADCON0bits.CHS = 0x0 ;    //setting input channel    // RA0/AN0 ->2
    TRISAbits.RA0 = 1;
    ADCON2bits.ADFM = 1 ;    //setting Left justified
    
     
    //setting acquisition time (ADCON2) ACQT 2:0
    //setting conversion time (ADCON2))
    ADCON2bits.ACQT = 0x0;  //Tacq = 2 Tad
    ADCON2bits.ADCS = 0;  // 2 TOSC    // 2.86 MHz
    ADCON0bits.ADON = 1;    //turn on ad module // 1: enable 
    /*setting adc interrupt 
     * 1.clear ADIF
     * 2.set ADIE
     * 3.SET GIE
    */
    //PIR1bits.ADIF = 0;       //ADIF(bit6) : 0
    //PIE1bits.ADIE = 1;    //ADIE(bit6) : 1 //0100 0000
    
    ADCON0bits.GO_DONE = 1;
    return ;
}

/* Calculation for PWM Period
 *   PWM Period = [(PR2) + 1] • 4 • TOSC • (TMR2 Prescale Value)
 *   1/PWM freq = [(PR2) + 1] • 4 • (1/FOSC) • (TMR2 Prescale Value)
 *   (1/PWM freq) / 4 / (1/FOSC) / (TMR2 Prescale Value) = [(PR2) + 1]
 *   0.25 / PWM freq * FOSC / (TMR2 Prescale Value) = [(PR2) + 1]
 *   0.25 * FOSC / PWM freq / (TMR2 Prescale Value) - 1 = PR2
 *   PR2 = 0.25 * FOSC / freq / 8 - 1
 *
 * Calculation for Max Freq & Min Freq
 *   (256 / 0.25 * Prescale / FOSC)^-1 = freq min 
 *   (1 / 0.25 * Prescale / FOSC)^-1 = freq max
 */

//Calculated for 1MHz, 16 prescaler
//Freq min = 61 Hz
//Freq max = 15625 Hz
void setPWMFrequency(void) {
    PR2 = 0.25  * FOSC / freq / PWM_PRESCALE - 1;
}

void setPWMDutyCycleCCP2(int pwm_percentage) {
    /* Calculation for PWM Duty Cycle
     *   PWM Duty Cycle = (CCPRXL:CCPXCON<5:4>) • TOSC • (TMR2 Prescale Value)
     *   PWM Period = [(PR2) + 1] • 4 • TOSC • (TMR2 Prescale Value)
     *   PWM Ratio = Duty Cycle / Period
     *   PWM Ratio = (CCPRXL:CCPXCON<5:4>) / [(PR2) + 1] • 4]
     *   (CCPRXL:CCPXCON<5:4>) = Ratio * [(PR2) + 1] • 4]
     */
    int pwm_period = PR2;
    // round off the float
    int pwm_duty_cycle = round( (pwm_percentage * 0.01f) * (pwm_period + 1) * 4 ); 
    CCP2CONbits.DC2B = 0b11 & pwm_duty_cycle; // DCxB<1:0>: bits 0, 1
    CCPR2L = pwm_duty_cycle >> 2;
}

//----------------------------------------------------------------------------
// High priority interrupt routine

void interrupt int0(void) {
    if (INTCONbits.INT0IF) {
        INTCONbits.INT0IF = 0;
        /*
        freq = ToneFreq[++array_index];
        if (freq == -1) { // reset to first freq
            array_index = 0;
            freq = ToneFreq[array_index];
        }
        setPWMFrequency();
        //if((PORTB & 0x01) == 1)
            setPWMDutyCycleCCP2(PWM_DUTY_CYCLE);
        //else if((PORTB & 0x01) == 0)
        //    setPWMDutyCycleCCP2(0);
        //resetTimer0OneSecond();
        */
    }

    if((PORTB & 0x01) == 0){
        setPWMDutyCycleCCP2(0);
    }
    /*
    if (INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0;
        setPWMDutyCycleCCP2(0);
        
    }
    */
    
    
}


void resetTimer0OneSecond() {
    
    TMR0H = TIMER0_RESET_VAL >> 8; //reset timer
    TMR0L = TIMER0_RESET_VAL & 0xFF; //reset timer
}

void Seru(){
    //--------------- 1 ----------------------
    setC6();  delayMusic(Dunit2, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setC6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG6();  delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setG6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setB6b();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG6();  delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setF6();  delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setG6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA6();  delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setCUT(); delayMusic(Dunit2, Dunit2);
    //--------------- 2 ----------------------
    setA6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setC7();  delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setC7();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setC7();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG6(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setF6();  delayMusic(Dunit1, Dunit1);
    setF6();  delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setG6(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG6(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setA6(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setF6(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG6(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA6(); delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setCUT(); delayMusic(Dunit4, Dunit4);
    //--------------- 3 ----------------------
    setD6(); delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setD6(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA6(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG6(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF6(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setD6(); delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setC6(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setD6(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF6(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setE6(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setD6(); delayMusic(Dunit8, Dunit8);
    setCUT(); delayMusic(cut, cut);
    //--------------- 4 ----------------------
    setC6();  delayMusic(Dunit2, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setC6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setB6b();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG6();  delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setF6();  delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setG6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA6();  delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setCUT(); delayMusic(Dunit2, Dunit2);
    //--------------- 5 ----------------------
    setA6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setC7();  delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setC7();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setC7();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG6();  delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setF6();  delayMusic(Dunit1, Dunit1);
    setF6();  delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setG6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA6();  delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setCUT(); delayMusic(Dunit4, Dunit4);
    //--------------- 6 ----------------------
    setD6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setD6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setD6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setD6();  delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setC6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setD6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setE6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setD6();  delayMusic(Dunit8, Dunit8);
    setCUT(); delayMusic(cut, cut);
    //--------------- 7 ----------------------
    setA6();  delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setA6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setC7();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setC7();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA6();  delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setCUT(); delayMusic(Dunit4, Dunit4);
    //--------------- 8 ----------------------
    setA6();  delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setA6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setC7();  delayMusic(Dunit2, Dunit2);
    setC7();  delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setC7();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setD7();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setD7();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setC7();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG6();  delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setCUT(); delayMusic(Dunit4, Dunit4);
    //--------------- 9 ----------------------
    setA6();  delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setA6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setC7();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setC7();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA6();  delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setCUT(); delayMusic(Dunit2, Dunit2);
    //--------------- 9 ----------------------
    setA6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setD7();  delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setD7();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setE7();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setC7();  delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setA6();  delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setG6();  delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setF6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG6();  delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA6();  delayMusic(Dunit8, Dunit8);
    setCUT(); delayMusic(cut, cut);
    
}


void TopOfTheWorld(){
    //----------------- 1 --------------------
    setG6(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setE7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setD7(); delayMusic(Dunit1, Dunit1);
    setD7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setC7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setB6(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setC7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setA6(); delayMusic(Dunit1, Dunit1);
    setA6(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setB6(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setG6(); delayMusic(Dunit1, Dunit1);
    setG6(); delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    //----------------- 2 --------------------
    setE7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setG7(); delayMusic(Dunit1, Dunit1);
    setG7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setE7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setD7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setE7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF7(); delayMusic(Dunit1, Dunit1);
    setF7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setD7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setE7(); delayMusic(Dunit4, Dunit4);
    setE7(); delayMusic(Dunit8, Dunit8);
    setCUT(); delayMusic(cut, cut);
    //----------------- 3 --------------------
    setF7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA7(); delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setA7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setD7(); delayMusic(Dunit1, Dunit1);
    setD7(); delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setD7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG7(); delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setF7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setG7(); delayMusic(Dunit2, Dunit2);
    setG7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setE7(); delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setD7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setE7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF7(); delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setF7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setE7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setF7(); delayMusic(Dunit1, Dunit1);
    setF7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setE7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setD7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setC7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setG6(); delayMusic(Dunit1, Dunit1);
    setG6(); delayMusic(Dunit8, Dunit8);
    setCUT(); delayMusic(cut, cut);
    //----------------- 4 --------------------
    setG6(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setE7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setD7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setC7(); delayMusic(Dunit2, Dunit2);
    setC7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setB6(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setC7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setA6(); delayMusic(Dunit1, Dunit1);
    setA6(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setB6(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG6(); delayMusic(Dunit4, Dunit4);
    setG6(); delayMusic(Dunit8, Dunit8);
    setCUT(); delayMusic(cut, cut);
    //----------------- 5 --------------------
    setE7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setG7(); delayMusic(Dunit2, Dunit2);
    setG7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setG7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setE7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setD7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setE7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setF7(); delayMusic(Dunit1, Dunit1);
    setF7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setB7(); delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setG7(); delayMusic(Dunit8, Dunit8);
    setCUT(); delayMusic(cut, cut);
    //----------------- 6 --------------------
    setF7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA7(); delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setA7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setF7(); delayMusic(Dunit2, Dunit2);
    setF7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setD7(); delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setD7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG7(); delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setF7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setG7(); delayMusic(Dunit2, Dunit2);
    setG7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setE7(); delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setD7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setE7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setE7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setE7(); delayMusic(Dunit2, Dunit2);
    setE7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setD7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setC7(); delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setG6(); delayMusic(Dunit2, Dunit2);
    setG6(); delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    //----------------- 7 --------------------
    setG6(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setC7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setD7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setCUT(); delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setE7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setE7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setF7(); delayMusic(Dunit1, Dunit1);
    setF7(); delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setD7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setC7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    //----------------- 8 --------------------
    setA7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setB7b(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setE7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setE7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setD7(); delayMusic(Dunit1, Dunit1);
    setD7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setE7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setF7(); delayMusic(Dunit1, Dunit1);
    setF7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA7(); delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setG7(); delayMusic(Dunit8, Dunit8);
    setCUT(); delayMusic(cut, cut);
    //----------------- 9 --------------------
    setF7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA7(); delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setG7(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setA7(); delayMusic(Dunit1, Dunit1);
    setA7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setB7(); delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setA7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setB7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setC8(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setC8(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setC8(); delayMusic(Dunit1, Dunit1);
    setCUT(); delayMusic(cut, cut);
    setG7(); delayMusic(Dunit1, Dunit1);
    setG7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setA7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setCUT(); delayMusic(Dunit2, Dunit2);
    //----------------- 10 --------------------
    setG7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setE7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setG7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setF7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setE7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setD7(); delayMusic(Dunit4, Dunit4);
    setCUT(); delayMusic(cut, cut);
    setC7(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setB6(); delayMusic(Dunit2, Dunit2);
    setCUT(); delayMusic(cut, cut);
    setC7(); delayMusic(Dunit8, Dunit8);
    setCUT(); delayMusic(cut, cut);
    
    
}

void delayMusic(int a, int b){
    int i, j, k;
    for(i=0; i<a; ++i){
        for(j=0; j<b; ++j){
            k = i + j;
        }
    }
    
}

void setC6(){
    freq = C6;
    setPWMFrequency();
    setPWMDutyCycleCCP2(PWM_DUTY_CYCLE);
    resetTimer0OneSecond();
}

void setD6(){
    freq = D6;
    setPWMFrequency();
    setPWMDutyCycleCCP2(PWM_DUTY_CYCLE);
    resetTimer0OneSecond();
}

void setE6(){
    freq = E6;
    setPWMFrequency();
    setPWMDutyCycleCCP2(PWM_DUTY_CYCLE);
    resetTimer0OneSecond();
}

void setF6(){
    freq = F6;
    setPWMFrequency();
    setPWMDutyCycleCCP2(PWM_DUTY_CYCLE);
    resetTimer0OneSecond();
}

void setG6(){
    freq = G6;
    setPWMFrequency();
    setPWMDutyCycleCCP2(PWM_DUTY_CYCLE);
    resetTimer0OneSecond();
}

void setA6(){
    freq = A6;
    setPWMFrequency();
    setPWMDutyCycleCCP2(PWM_DUTY_CYCLE);
    resetTimer0OneSecond();
}

void setB6(){
    freq = B6;
    setPWMFrequency();
    setPWMDutyCycleCCP2(PWM_DUTY_CYCLE);
    resetTimer0OneSecond();
}

void setC7(){
    freq = C7;
    setPWMFrequency();
    setPWMDutyCycleCCP2(PWM_DUTY_CYCLE);
    resetTimer0OneSecond();
}

void setD7(){
    freq = D7;
    setPWMFrequency();
    setPWMDutyCycleCCP2(PWM_DUTY_CYCLE);
    resetTimer0OneSecond();
}

void setE7(){
    freq = E7;
    setPWMFrequency();
    setPWMDutyCycleCCP2(PWM_DUTY_CYCLE);
    resetTimer0OneSecond();
}

void setF7(){
    freq = F7;
    setPWMFrequency();
    setPWMDutyCycleCCP2(PWM_DUTY_CYCLE);
    resetTimer0OneSecond();
}

void setG7(){
    freq = G7;
    setPWMFrequency();
    setPWMDutyCycleCCP2(PWM_DUTY_CYCLE);
    resetTimer0OneSecond();
}

void setA7(){
    freq = A7;
    setPWMFrequency();
    setPWMDutyCycleCCP2(PWM_DUTY_CYCLE);
    resetTimer0OneSecond();
}

void setB7(){
    freq = B7;
    setPWMFrequency();
    setPWMDutyCycleCCP2(PWM_DUTY_CYCLE);
    resetTimer0OneSecond();
}

void setB6b(){
    freq = B6b;
    setPWMFrequency();
    setPWMDutyCycleCCP2(PWM_DUTY_CYCLE);
    resetTimer0OneSecond();
}

void setCUT(){
    freq = silence;
    setPWMFrequency();
    setPWMDutyCycleCCP2(PWM_DUTY_CYCLE);
    resetTimer0OneSecond();
}

void setB7b(){
    freq = B7b;
    setPWMFrequency();
    setPWMDutyCycleCCP2(PWM_DUTY_CYCLE);
    resetTimer0OneSecond();
}

void setC8(){
    freq = C8;
    setPWMFrequency();
    setPWMDutyCycleCCP2(PWM_DUTY_CYCLE);
    resetTimer0OneSecond();
}


//----------------------------------------------------------------------------
