/* 
 * File:   Water4Project.c
 * Author: Jacqui
 *
 * Created on January 20, 2016, 6:35 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <xc.h>
#include <string.h>




// ****************************************************************************
// *** PIC24F32KA302 Configuration Bit Settings *******************************
// ****************************************************************************
// FBS
#pragma config BWRP = OFF // Boot Segment Write Protect (Disabled)
#pragma config BSS = OFF // Boot segment Protect (No boot program flash segment)
// FGS
#pragma config GWRP = OFF // General Segment Write Protect (General segment may be written)
#pragma config GSS0 = OFF // General Segment Code Protect (No Protection)
// FOSCSEL
#pragma config FNOSC = FRC // Oscillator Select (Fast RC Oscillator (FRC))
#pragma config SOSCSRC = ANA // SOSC Source Type (Analog Mode for use with crystal)
#pragma config LPRCSEL = HP // LPRC Oscillator Power and Accuracy (High Power, High Accuracy Mode)
#pragma config IESO = OFF // Internal External Switch Over bit (Internal External Switchover mode enabled (Two-speed Start-up enabled))
// FOSC
#pragma config POSCMOD = NONE // Primary Oscillator Configuration bits (Primary oscillator disabled)
#pragma config OSCIOFNC = OFF // CLKO Enable Configuration bit (CLKO output disabled)
#pragma config POSCFREQ = HS // Primary Oscillator Frequency Range Configuration bits (Primary oscillator/external clock input frequency greater than 8MHz)
#pragma config SOSCSEL = SOSCHP // SOSC Power Selection Configuration bits (Secondary Oscillator configured for high-power operation)
#pragma config FCKSM = CSDCMD // Clock Switching and Monitor Selection (Both Clock Switching and Fail-safe Clock Monitor are disabled)
// FWDT
#pragma config WDTPS = PS32768 // Watchdog Timer Postscale Select bits (1:32768)
#pragma config FWPSA = PR128 // WDT Prescaler bit (WDT prescaler ratio of 1:128)
#pragma config FWDTEN = OFF // Watchdog Timer Enable bits (WDT disabled in hardware; SWDTEN bit disabled)
#pragma config WINDIS = ON // Windowed Watchdog Timer Disable bit (Windowed WDT enabled)
// FPOR
#pragma config BOREN = BOR3 // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware, SBOREN bit disabled)
#pragma config LVRCFG = OFF // (Low Voltage regulator is not available)
#pragma config PWRTEN = ON // Power-up Timer Enable bit (PWRT enabled)
#pragma config I2C1SEL = PRI // Alternate I2C1 Pin Mapping bit (Use Default SCL1/SDA1 Pins For I2C1)
//#pragma config BORV = V20 // Brown-out Reset Voltage bits (Brown-out Reset set to lowest voltage (2.0V))
#pragma config MCLRE = ON // MCLR Pin Enable bit (RA5 input pin disabled,MCLR pin enabled)
// FICD
#pragma config ICS = PGx1 // ICD Pin Placement Select bits (EMUC/EMUD share PGC1/PGD1)
// FDS
#pragma config DSWDTPS = DSWDTPSF // Deep Sleep Watchdog Timer Postscale Select bits (1:2,147,483,648 (25.7 Days))
#pragma config DSWDTOSC = LPRC // DSWDT Reference Clock Select bit (DSWDT uses Low Power RC Oscillator (LPRC))
#pragma config DSBOREN = ON // Deep Sleep Zero-Power BOR Enable bit (Deep Sleep BOR enabled in Deep Sleep)
#pragma config DSWDTEN = ON // Deep Sleep Watchdog Timer Enable bit (DSWDT enabled)

const int pulseWidthThreshold = 20; // The value to check the pulse width against (2048)
static volatile int buttonFlag = 0; // alerts us that the button has been pushed and entered the inerrupt subroutine

void initAdc(void); // forward declaration of init adc
/*********************************************************************
 * Function: initialization()
 * Input: None
 * Output: None
 * Overview: configures chip to work in our system (when power is turned on, these are set once)
 * Note: Pic Dependent
 * TestDate: 06-03-14
 ********************************************************************/
void initialization(void) {
    ////------------Sets up all ports as digial inputs-----------------------
    //IO port control
    ANSA = 0; // Make PORTA digital I/O
    TRISA = 0xFFFF; // Make PORTA all inputs
    ANSB = 0; // All port B pins are digital. Individual ADC are set in the readADC function
    TRISB = 0xFFFF; // Sets all of port B to input

    // Timer control (for WPS)
    T1CONbits.TCS = 0; // Source is Internal Clock (8MHz)
    T1CONbits.TCKPS = 0b11; // Prescalar to 1:256
    T1CONbits.TON = 1; // Enable the timer (timer 1 is used for the water sensor)

    // UART config
    U1BRG = 51; // Set baud to 9600, FCY = 8MHz (#pragma config FNOSC = FRC)
    U1STA = 0;
    U1MODE = 0x8000; //enable UART for 8 bit data//no parity, 1 stop bit
    U1STAbits.UTXEN = 1; //enable transmit

    //H2O sensor config
    // WPS_ON/OFF pin 2
    TRISAbits.TRISA0 = 0; //makes water presence sensor pin an output.
    PORTAbits.RA0 = 1; //turns on the water presnece sensor.
    
    CNEN2bits.CN24IE = 1; // enable interrupt for pin 15
    IEC1bits.CNIE = 1; // enable change notification interrupt

    initAdc();
}

/*********************************************************************
 * Function: readWaterSensor
 * Input: None
 * Output: pulseWidth
 * Overview: RB5 is one water sensor, start at beginning of positive pulse
 * Note: Pic Dependent
 * TestDate: Not tested as of 03-05-2015
 ********************************************************************/
int readWaterSensor(void) // RB5 is one water sensor
{
    // WPS_OUT - pin14
    if (PORTBbits.RB5) {
        while (PORTBbits.RB5) {
        }; //make sure you start at the beginning of the positive pulse
    }
    while (!PORTBbits.RB5) {
    }; //wait for rising edge
    int prevICTime = TMR1; //get time at start of positive pulse
    while (PORTBbits.RB5) {
    };
    int currentICTime = TMR1; //get time at end of positive pulse
    long pulseWidth = 0;
    if (currentICTime >= prevICTime) {
        pulseWidth = (currentICTime - prevICTime);
    } else {
        pulseWidth = (currentICTime - prevICTime + 0x100000000);
    }
    //Check if this value is right
    return (pulseWidth <= pulseWidthThreshold);
}

/*********************************************************************
 * Function: initAdc()
 * Input: None
 * Output: None
 * Overview: Initializes Analog to Digital Converter
 * Note: Pic Dependent
 * TestDate: 06-02-2014
 ********************************************************************/
void initAdc(void) {
    // 10bit conversion
    AD1CON1 = 0; // Default to all 0s
    AD1CON1bits.ADON = 0; // Ensure the ADC is turned off before configuration
    AD1CON1bits.FORM = 0; // absolute decimal result, unsigned, right-justified
    AD1CON1bits.SSRC = 0; // The SAMP bit must be cleared by software
    AD1CON1bits.SSRC = 0x7; // The SAMP bit is cleared after SAMC number (see
    // AD3CON) of TAD clocks after SAMP bit being set
    AD1CON1bits.ASAM = 0; // Sampling begins when the SAMP bit is manually set
    AD1CON1bits.SAMP = 0; // Don't Sample yet
    // Leave AD1CON2 at defaults
    // Vref High = Vcc Vref Low = Vss
    // Use AD1CHS (see below) to select which channel to convert, don't
    // scan based upon AD1CSSL
    AD1CON2 = 0;
    // AD3CON
    // This device needs a minimum of Tad = 600ns.
    // If Tcy is actually 1/8Mhz = 125ns, so we are using 3Tcy
    //AD1CON3 = 0x1F02; // Sample time = 31 Tad, Tad = 3Tcy
    AD1CON3bits.SAMC = 0x1F; // Sample time = 31 Tad (11.6us charge time)
    AD1CON3bits.ADCS = 0x2; // Tad = 3Tcy
    // Conversions are routed through MuxA by default in AD1CON2
    AD1CHSbits.CH0NA = 0; // Use Vss as the conversion reference
    AD1CSSL = 0; // No inputs specified since we are not in SCAN mode
    // AD1CON2
}

/*********************************************************************
 * Function: readAdc()
 * Input: channel
 * Output: adcValue
 * Overview: check with accelerometer
 * Note: Pic Dependent
 * TestDate:
 ********************************************************************/
int readAdc(int channel) //check with accelerometer
{
    switch (channel) {
        case 4:
            ANSBbits.ANSB2 = 1; // AN4 is analog
            TRISBbits.TRISB2 = 1; // AN4 is an input
            AD1CHSbits.CH0SA = 4; // Connect AN4 as the S/H input
            break;
    }
    AD1CON1bits.ADON = 1; // Turn on ADC
    AD1CON1bits.SAMP = 1;
    while (!AD1CON1bits.DONE) {
    }
    return ADC1BUF0;
}
/*********************************************************************
 * Function: delayMs()
 * Input: milliseconds
 * Output: None
 * Overview: Delays the specified number of milliseconds
 * Note: Depends on Clock speed. Pic Dependent
 * TestDate: 05-20-14
 ********************************************************************/
void delayMs(int ms) {
    int myIndex;
    while (ms > 0) {
        myIndex = 0;
        while (myIndex < 667) {
            myIndex++;
        }
        ms--;
    }
}

void sendCommand(char command); // forward dec
void sendData(char data); // forward dec

void initLCD(void){
     PORTBbits.RB1 = 0;       //LCD enable pin CTL3
     delayMs(100);           //Wait >15 msec after power is applied
     sendCommand(0x30);      //command 0x30 = Wake up
     delayMs(30);            //must wait 5ms, busy flag not available
     sendCommand(0x30);      //command 0x30 = Wake up #2
     delayMs(10);            //must wait 160us, busy flag not available
     sendCommand(0x30);      //command 0x30 = Wake up #3
     delayMs(10);            //must wait 160us, busy flag not available
     sendCommand(0x30);      //Function set: 8-bit/1-line
     sendCommand(0x10);      //Set cursor
     sendCommand(0x0c);      //Display ON; Cursor ON
     sendCommand(0x06);      //Entry mode set

}

void sendCommand(char command){
//    PORTBbits.RB12  = (command&0b00000001); // DIS_B[0]
//    PORTBbits.RB4  = (command&0b00000010); //  DIS_B[1]
//    PORTBbits.RB9  = (command&0b00000100);  // DIS_B[2]
//    PORTBbits.RB5  = (command&0b00001000);  // DIS_B[3]
//    PORTBbits.RB14  = (command&0b00010000); // DIS_B[4]
//    PORTBbits.RB2  = (command&0b00100000);  // DIS_B[5]
//    PORTBbits.RB13  = (command&0b01000000); // DIS_B[6]
//    PORTBbits.RB3  = (command&0b10000000);  // DIS_B[7]
    PORTBbits.RB12 = (command & 0x01) >> 0;
    PORTBbits.RB4  = (command & 0x02) >> 1;
    PORTBbits.RB9  = (command & 0x04) >> 2;
    PORTBbits.RB5  = (command & 0x08) >> 3;
    PORTBbits.RB14 = (command & 0x10) >> 4;
    PORTBbits.RB2  = (command & 0x20) >> 5;
    PORTBbits.RB13 = (command & 0x40) >> 6;
    PORTBbits.RB3  = (command & 0x80) >> 7;

    PORTBbits.RB0 = 1;                 //high for command CTL1

     PORTBbits.RB1 = 1;      // CTL3
     delayMs(1);
     PORTBbits.RB1 = 0;     // CTL3
}
void sendData(char data){
//    PORTBbits.RB12  = (data & 0b00000001);    // DIS_B[0]
//    PORTBbits.RB4  = (data & 0b00000010);     // DIS_B[1]
//    PORTBbits.RB9  = (data & 0b00000100);     // DIS_B[2]
//    PORTBbits.RB5  = (data & 0b00001000);     // DIS_B[3]
//    PORTBbits.RB14  = (data & 0b00010000);    // DIS_B[4]
//    PORTBbits.RB2  = (data & 0b00100000);     // DIS_B[5]
//    PORTBbits.RB13  = (data & 0b01000000);    // DIS_B[6]
//    PORTBbits.RB3  = (data & 0b10000000);     // DIS_B[7]
    
    PORTBbits.RB12 = (data & 0x01) >> 0;
    PORTBbits.RB4  = (data & 0x02) >> 1;
    PORTBbits.RB9  = (data & 0x04) >> 2;
    PORTBbits.RB5  = (data & 0x08) >> 3;
    PORTBbits.RB14 = (data & 0x10) >> 4;
    PORTBbits.RB2  = (data & 0x20) >> 5;
    PORTBbits.RB13 = (data & 0x40) >> 6;
    PORTBbits.RB3  = (data & 0x80) >> 7;
    PORTBbits.RB0 = 0; //low for data CTL1

    PORTBbits.RB1 = 1;  // CTL3
    delayMs(1); //Clocks on falling edge of enable
    PORTBbits.RB1 = 0;  // CTL3
}
void __attribute__((__interrupt__,__auto_psv__)) _DefaultInterrupt()
{ //Tested 06-05-2014

}
 void __attribute__ (( interrupt, auto_psv )) _CNInterrupt(void) 
 {
     if(IFS1bits.CNIF && PORTBbits.RB6)
     {  // if button pushed it goes high

       buttonFlag = 1;  // I have entered the ISR

       IFS1bits.CNIF = 0;
     }

}
 
void hoursToAsciiDisplay(int hours){
    initLCD();
    int startLcdView = 0;
    if (hours == 0)
    {
        sendData(48); // send 0 as the number of hours
    }
    else
    {
        if(startLcdView || hours / 10000 != 0)
        {
            sendData(hours / 10000 + 48);
            startLcdView = 1;
        }
        hours /= 10;
        if(startLcdView || hours / 1000 != 0)
        {
            sendData(hours / 1000 + 48);
            startLcdView = 1;
        }
        hours /= 10;
        if(startLcdView || hours / 100 != 0)
        {
            sendData(hours / 100 + 48);
            startLcdView = 1;
        }
        hours /= 10;
        if(startLcdView || hours / 10 != 0)
        {
            sendData(hours / 10 + 48);
            startLcdView = 1;
        }
        hours /= 10;
        sendData(hours + 48);
    }
}

/*
 * 
 */
#define delayTime       500
#define msHr            (uint32_t)3600000
#define hourTicks       msHr / delayTime
int main (void){
    initialization();
    int counter = 0;
    int hourCounter = 0;
    PORTBbits.RB15 = 0;  //R/W always low for write CTL2

     initLCD();
    // There's a chance that our numbers aren't ASCII
    sendData(0x48);   //H
//            sendData(0x65);   //E
//            sendData(0x6C);   //L
//            sendData(0x6C);   //L
//            sendData(0x6F);   //0



    while(1)
    {        
        delayMs(delayTime);
            // is there water?
        if(readWaterSensor())
        {
            counter++; // increments every half a second

            if (counter >= hourTicks)
            { // 7200 counter has reached 1 hour's worth
                hourCounter++;
                counter = 0; 
            }
        }
        
        if(buttonFlag)
        {
            buttonFlag = 0;
            hoursToAsciiDisplay(hourCounter);
            delayMs(5000);
        }
    }

    return -1;
}

