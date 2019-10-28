#include "IWPUtilities.h"
#include "Pin_Manager.h"
#include "FONAUtilities.h"
#include "I2C.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <xc.h>
#include <string.h>
#include <p24FV32KA302.h>

/*********************************
Table of Contents
1) Constants & Global Variables
2) Pin Management Functions
int digitalPinSet(int pin, int io);
void specifyAnalogPin(int pin, int analogOrDigital);
void analogIOandSHinput(int pin, int IO);
int digitalPinStatus(int pin);
3) Initialization
void initialization(void);
4) String Functions
int longLength(long num);
void longToString(long num, char *numString);
int stringLength(char *string);
void concat(char *dest, const char *src);
void floatToString(float myValue, char *myString);
5) SIM Functions - Moved to FONAUtilities

6) Sensor Functions
int readWaterSensor(void);
void initAdc(void);
int readAdc(int channel);
float getHandleAngle();
void initializeQueue(float value);
void pushToQueue(float value);
float queueAverage();
float queueDifference();
7) I2C Functions - Moved to I2C file

8) RTCC Functions
void turnOffClockOscilator (void);
int getSecondI2C (void);
int getMinuteI2C (void);
int getHourI2C (void);
int getYearI2C (void);
int getMonthI2C (void);
int getWkdayI2C (void);
int getDateI2C (void);
void setTime(char sec, char min, char hr, char wkday, char date, char month, char year);
9) Misc Functions
float degToRad(float degrees);
void delayMs(int ms);
int getLowerBCDAsDecimal(int bcd);
int getUpperBCDAsDecimal(int bcd);
int getTimeHour(void);
long timeStamp(void);
void pressReset();
int translate(char digit);
void RTCCSet(void);
int getMinuteOffset();
char BcdToDec(char val);
char DecToBcd(char val);
 * 
void EEProm_Write_Int(int addr, int newData);
int EEProm_Read_Int(int addr);
EEProm_Read_Float(unsigned int ee_addr, void *obj_p);
EEProm_Write_Float(unsigned int ee_addr, void *obj_p);
void noonMessage(void);
 **********************************/

float codeRevisionNumber = 5.0;  //Current as of 10/1/19
// HELLO THIS IS CORY

int __attribute__((space(eedata))) eeData; // Global variable located in EEPROM

const float radToDegConst = 57.29579143313326; // (180/PI)
const int xAxis = 11; // analog pin connected to x axis of accelerometer
const int yAxis = 12; // analog pin connected to y axis of accelerometer
const int batteryVoltage = 15; // analog channel connected to the battery
const float MKII = .624; //0.4074 L/Radian; transfer variable for mkII delta handle angle to outflow

const float a = 27.336; // a in quadratic equation to solve for volume
const float b = -35.169; // b in quadratic equation to solve for volume
const float c = 12.476; // c in quadratic equation to solve for volume
const float quadVertex = 1.1644; // the y value of the vertex of the parabola used to calculate volume; = (-(b^2)/(4*a))+c

const float leakSensorVolume = 0.01781283; // This is in Liters; pipe dia. = 33mm; rod diam 12 mm; gage length 24mm
// THE FASTEST WE COULD POSSIBLE BE IF IT DOES PRIME IS 42ms FOR THE AVERAGE PERSON but we'll do half that incase you're pumping hard to get it to prime
const int alarmHour = 0x0000; // The weekday and hour (24 hour format) (in BCD) that the alarm will go off
const int alarmStartingMinute = 1; // The minimum minute that the alarm will go off
const int alarmMinuteMax = 5; // The max number of minutes to offset the alarm (the alarmStartingMinute + a random number between 0 and this number)
const int signedNumAdjustADC = 512; // Used to divide the total range of the output of the 10 bit ADC into positive and negative range.
const int NoWaterThreshold = 78; // The value to check the pulse width against (2048). Changed from 20 to 78
const int BrokenWPSThreshold = 19;

///const int pulseWidthThreshold = 130; // This is just for Zantele we see about 160hz, not when water is there.  Not sure what we would see with no water

const int upstrokeInterval = 10; // The number of milliseconds to delay before reading the upstroke
const int max_pause_while_pumping = 1020; //The maximum time (in loops, each loop delaying 9.8ms) that the pump handle is not moving before we say that the person stopped trying
long leakRateTimeOut = 3000; // Maximum number of milliseconds to wait for water to drain when calculating leak rate 
//long timeBetweenUpstrokes = 18000; // 18000 seconds (based on upstrokeInterval)
const int decimalAccuracy = 3; // Number of decimal places to use when converting floats to strings
const float angleThresholdSmall = 0.1; //number close to zero to determine if handle is moving w/o interpreting accelerometer noise as movement.
const float handleMovementThreshold = 5.0; // When the handle has moved this many degrees from rest, we start calculating priming 
//roughly based on calculations from India Mark II sustainability report that 
//67.6 strokes/minute average pumper
//25.4 degrees/pk-pk movement
//2 pk-pk/stroke
//(25.4*2) degrees/stroke
// 0.572 degrees/10ms delay for the average pumper
//It would be nice to have data for the slowest possible pumper
const float angleThresholdLarge = 5.0; //total angle movement to accumulate before identifying movement as intentional pumping
const float upstrokeToMeters = 0.01287;
const int minimumAngleDelta = 10;
//const float batteryLevelConstant = 0.476; //This number is found by Vout = (R32 * Vin) / (R32 + R31), Yields Vin = Vout / 0.476
const float batteryLevelConstant = 8.86; // this is the average value s I'm puttig in in Yiwogu
//This number is used to convert fraction of full range to a voltage Vcc(Batt_Level/V_Batt) 
//                                          Nominal value is 8.748 (Vin = 3.6 and resistor divider values are exact)
const float BatteryDyingThreshold = 0.05; //When the battery voltage drops this much in 2 hours, go to sleep
const int secondI2Cvar = 0x00;
const int minuteI2Cvar = 0x01;
const int hourI2Cvar = 0x02;
const int dayI2Cvar = 0x03;
const int dateI2Cvar = 0x04;
const int monthI2Cvar = 0x05;
const int yearI2Cvar = 0x06;
const float PI = 3.141592;

const float angleRadius = .008; // this is 80 millimeters so should it equal 80 or .008?
// EEPROM locations
int DailyReportEEPromStart = 21; // this is the EEPROM slot that Daily Report Messages will begin to be saved
int DiagnosticEEPromStart = 102; // this is the EEPROM slot that Diagnostic information can begin to be saved
int EEpromDiagStatus = 104; // 1 means report hourly to diagnostic phone number, 0 = don't report
int EEpromCountryCode = 105;
int EEpromMainphoneNumber = 106;
int EEpromDebugphoneNumber = 108;
int EEpromCodeRevisionNumber = 110;
int depthSensorInUse;


int prevTimer2 = 0; // Should intially begin at zero


int prevDayDepthSensor;
float midDayDepth;
//int prevMinute;
int prevHour; // used during debug to send noon message every hour
int invalid = 0;
float angle1 = 0;
float angle2 = 0;
float angle3 = 0;
float angle4 = 0;
float angle5 = 0;
float angle6 = 0;
float angle7 = 0;
float angle8 = 0;
float angle9 = 0;
float angle10 = 0;

/*
// 53 tap filter @ 106 Hz fs
const float filter[53] = {0.0010, 0.0022, 0.0039, 0.0056, 0.0074, 0.0092, 0.0111, 0.0130, 0.0150, 0.0169, 
    0.0188, 0.0208, 0.0226, 0.0245, 0.0262, 0.0279, 0.0295, 0.0310, 0.0323, 0.0335,
    0.0346, 0.0355, 0.0363, 0.0369, 0.0374, 0.0376, 0.0377, 0.0376, 0.0374, 0.0369, 
    0.0363, 0.0355, 0.0346, 0.0335, 0.0323, 0.0310, 0.0295, 0.0279, 0.0262, 0.0245, 
    0.0226, 0.0208, 0.0188, 0.0169, 0.0150, 0.0130, 0.0111, 0.0092, 0.0074, 0.0056, 
    0.0039, 0.0022, 0.0010};
*/
    
// 51 @ 102 Hz fs
const float filter[51] = {0.0011, 0.0024, 0.0042, 0.0060, 0.0080, 0.0100, 0.0120, 0.0141, 0.0162, 0.0183,
    0.0204, 0.0225, 0.0245, 0.0264, 0.0283, 0.0300, 0.0316, 0.0332, 0.0345, 0.0357,
    0.0368, 0.0376, 0.0383, 0.0388, 0.0391, 0.0392, 0.0391, 0.0388, 0.0383, 0.0376,
    0.0368, 0.0357, 0.0345, 0.0332, 0.0316, 0.0300, 0.0283, 0.0264, 0.0245, 0.0225,
    0.0204, 0.0183, 0.0162, 0.0141, 0.0120, 0.0100, 0.0080, 0.0060, 0.0042, 0.0024,
    0.0011};


/*
// 61 tap filter @ 120 Hz fs
const float filter[61] = {0.0002, 0.0011, 0.0024, 0.0036, 0.0050, 0.0064, 0.0078, 0.0093, 0.0108, 0.0123,
    0.0138, 0.0153, 0.0168, 0.0183, 0.0198, 0.0212, 0.0226, 0.0239, 0.0252, 0.0264, 
    0.0276, 0.0286, 0.0296, 0.0304, 0.0312, 0.0318, 0.0324, 0.0328, 0.0331, 0.0333,
    0.0333, 0.0333, 0.0331, 0.0328, 0.0324, 0.0318, 0.0312, 0.0304, 0.0296, 0.0286,
    0.0276, 0.0264, 0.0252, 0.0239, 0.0226, 0.0212, 0.0198, 0.0183, 0.0168, 0.0153,
    0.0138, 0.0123, 0.0108, 0.0093, 0.0078, 0.0064, 0.0050, 0.0036, 0.0024, 0.0011,
    0.0002};
*/
int success = 0;

// ****************************************************************************
// *** Global Variables *******************************************************
// ****************************************************************** 


int angleArray[51];

//****************Hourly Diagnostic Message Variables************************
int tech_at_pump = 0; //default to technician not at pump
float sleepHrStatus = 0; // 1 if we slept during the current hour, else 0
float timeSinceLastRestart = 0; // Total time in hours since last restart 
int internalHour = 0; // Hour of the day according to internal RTCC
int internalMinute = 0; // Minute of the hour according to the internal RTCC
float extRtccTalked = 0; // set to 1 if the external RTCC talked during the last hour and didn't time out every time
float numberTries = 0; // Keeps track of the number of times we attempt to connect to the network to send diagnostic messages
float extRTCCset = 0; // To keep track if the VTCC time was used to set the external RTCC
float resetCause = 0; //0 if no reset occurred, else the RCON register bit number that is set is returned

//*****************VTCC Variables*******************************************
char monthArray[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
int secondVTCC = 0;
char minuteVTCC = 0;
char hourVTCC = 0;
char dateVTCC = 0;
char monthVTCC = 0;

float longestPrime = 0; // total upstroke fo the longest priming event of the day
float leakRateLong = 0; // largest leak rate recorded for the day
//float batteryFloat;
float BatteryLevelArray[3];
char active_volume_bin = 0; //keeps track of which of the 12 volume time slots is being updated
char never_primed = 0; //set to 1 if the priming loop is exited without detecting water
char print_debug_messages = 0; //set to 1 when we want the debug messages to be sent to the Tx pin.
float diagnostic = 0; //set to 1 when we want the text messages to be sent hourly
float debugCounter = 0; // DEBUG used as a variable for various things while debugging 
float volume02 = 0; // Total Volume extracted from 0:00-2:00
float volume24 = 0;
float volume46 = 0;
float volume68 = 0;
float volume810 = 0;
float volume1012 = 0;
float volume1214 = 0;
float volume1416 = 0;
float volume1618 = 0;
float volume1820 = 0;
float volume2022 = 0;
float volume2224 = 0;
float EEFloatData = 0; // to be used when trying to write a float to EEProm EEFloatData = 123.456 then pass as &EEFloatData
int hour = 0; // Hour of day according to chosen time source; external RTCC or internal VTCC
int min = 0; // Minute of day according to chosen time source; external RTCC or internal VTCC
int TimeSinceLastHourCheck = 0; // we check this when we have gone around the no pumping loop enough times that 1 minute has gone by
int TimeSinceLastBatteryCheck = 0; // we only check the battery every 20min when sleeping
int minute = 0; //minute of the day
int year = 0; //current year
int month = 0; //current month
int day = 0; // current date
float date = 0; // date variable
//Pin assignments
int mclrPin = 1;
int depthSensorPin = 2;
int simVioPin = 3;
int Pin4 = 4;
int Pin5 = 5;
int rxPin = 6;
int depthSensorOnOffPin = 7;
int GND2Pin = 8;
int Pin9 = 9;
int Pin10 = 10;
int batteryLevelPin = 11;
int Pin12 = 12;
int vccPin = 13;
int waterPresenceSensorPin = 14;
int pwrKeyPin = 15;
int txPin = 16;
int sclI2CPin = 17;
int sdaI2CPin = 18;
int statusPin = 19;
int vCapPin = 20;
int picKit4Pin = 21;
int picKit5Pin = 22;
int yAxisAccelerometerPin = 23;
int xAxisAccelerometerPin = 24;
int netLightPin = 25;
int waterPresenceSensorOnOffPin = 26;
int GNDPin = 27;
int vcc2Pin = 28;


/////////////////////////////////////////////////////////////////////
////                                                             ////
////                    INITIALIZATION                           ////
////                                                             ////
/////////////////////////////////////////////////////////////////////

/*********************************************************************
 * Function: initialization()
 * Input: None
 * Output: None
 * Overview: configures chip to work in our system (when power is turned on, these are set once)
 * Note: Pic Dependent
 * TestDate: 06-03-14
 ********************************************************************/
void initialization(void) {    
    char localSec = 0;
    char localMin = 3;
    char localHr = 8;
    char localWkday = 2;
    char localDate = 8;
    char localMonth = 10;
    char localYear = 18;
    ////------------Sets up all ports as digital inputs-----------------------
    //IO port control
    ANSA = 0; // Make PORTA digital I/O
    TRISA = 0xFFFF; // Make PORTA all inputs
    ANSB = 0; // All port B pins are digital. Individual ADC are set in the readADC function
    TRISB = 0xFFFF; // Sets all of port B to input
    TRISBbits.TRISB0 = 0; //DEBUG Make spare test pin#4 on Pic an output
    TRISBbits.TRISB1 = 0;
    CNPU1bits.CN5PUE = 1; //put a weak pull up resistor on RB1 which is pin 5.
    // use this pin to detect when the Diagnostic board is plugged in.
    //PORTBbits.RB0 = 1; //DEBUG Set this high,  When we go to sleep we will make it low


    // Timer control (for WPS and FONA interactions)
    T1CONbits.TCS = 0; // Source is Internal Clock if FNOSC = FRC, Fosc/2 = 4Mhz
    T1CONbits.TCKPS = 0b11; // Prescalar to 1:256 
    // if FNOSC = FRC, Timer Clock = 15.625khz
    T1CONbits.TON = 1; // Enable the timer (timer 1 is used for the water sensor and checking for network)

    // Timer control (for internal VTCC time calculations)
    T2CONbits.TCS = 0; //Source is Internal Clock Fosc/2  if #pragma config FNOSC = FRC, Fosc/2 = 4Mhz
    T2CONbits.T32 = 0; // Using 16-bit timer2
    T2CONbits.TCKPS = 0b11; // Prescalar to 1:256 (Need prescalar of at least 1:8 for this) 
    // if FNOSC = FRC, Timer Clock = 15.625khz
    T2CONbits.TON = 1; // Starts 16-bit Timer2

    // Timer control (for regulating accelerometer sampling rate)
    T4CONbits.TCS = 0; //Source is Internal Clock Fosc/2  if #pragma config FNOSC = FRC, Fosc/2 = 4Mhz
    T4CONbits.T32 = 0; // Using 16-bit timer2
    T4CONbits.TCKPS = 0b11; // Prescalar to 1:256 (Need prescalar of at least 1:8 for this) 
    // if FNOSC = FRC, Timer Clock = 15.625khz
    

    // UART config
    //    U1MODE = 0x8000;  
    U1MODEbits.BRGH = 0; // Use the standard BRG speed
    U1BRG = 25; // set baud to 9600, assumes FCY=4Mhz (FNOSC = FRC)
    U1MODEbits.PDSEL = 0; // 8 bit data, no parity
    U1MODEbits.STSEL = 0; // 1 stop bit

    // The Tx and Rx PINS are enabled as the default 
    U1STA = 0; // clear Status and Control Register 
    U1STAbits.UTXEN = 1; // enable transmit
    // no need to enable receive.  The default is that a 
    // receive interrupt will be generated when any character
    // is received and transferred to the receive buffer
    U1STAbits.URXISEL = 0; // generate an interrupt each time a character is received
    IFS0bits.U1RXIF = 0; // clear the Rx interrupt flag
    pinDirectionIO(rxPin, 1); //explicitly make Rx pin an input
    _U1RXIF = 0;
    // IEC0bits.U1RXIE = 1;  // enable Rx interrupts
    // _U1RXIE = 1;
    U1MODEbits.UARTEN = 1; // Turn on the UART
    ReceiveTextMsg[0] = 0; // Start with an empty string
    ReceiveTextMsgFlag = 0;


    //H2O sensor config
    pinDirectionIO(waterPresenceSensorOnOffPin, 0); //makes water presence sensor pin an output.
    digitalPinSet(waterPresenceSensorOnOffPin, 0); //turns off the water presence sensor.

    // From fona code (for enabling Texting)
    pinDirectionIO(pwrKeyPin, 0); //TRISBbits.TRISB6 = 0; //sets power key as an output (Pin 15)
    pinDirectionIO(simVioPin, 0); //TRISAbits.TRISA1 = 0; //sets Vio as an output (pin 3)
    digitalPinSet(pwrKeyPin, 1); //PORTBbits.RB6 = 1; // Reset the Power Key so it can be turned off later (pin 15)
    digitalPinSet(simVioPin, 1); //PORTAbits.RA1 = 1; //Tells Fona what logic level to use for UART
    turnOffSIM(); //Make sure the FONA board is powered down

    //depth sensor I/O         
    depthSensorInUse = 0; // If Depth Sensor is in use, make a 1. Else make it zero.

    initAdc();

    //batteryFloat = batteryLevel();
    BatteryLevelArray[0] = batteryLevel(); //Used to track change in battery voltage
    BatteryLevelArray[1] = BatteryLevelArray[0]; //Used to track change in battery voltage
    BatteryLevelArray[2] = BatteryLevelArray[0]; //Used to track change in battery voltage
    
    int i;
    for (i = 0; i < 51; i++) {
        angleArray[i] = atan2(readAdc(yAxis) - signedNumAdjustADC, 
                readAdc(xAxis) - signedNumAdjustADC) * radToDegConst;
    }

    // We may be waking up because the battery was dead or the WatchDog expired.  
    // If that is the case, Restart Status, EEProm#20, will be zero and we want 
    // to continue using the leakRateLong and longestPrime from EEProm. Otherwise, 
    // it will be a bogus number indicating the original start up after programming 
    // and we want to clear our EEProm memory locations
    EEProm_Read_Float(20, &EEFloatData);
    print_debug_messages = 2;
    if (EEFloatData == 0) {
        EEProm_Read_Float(0, &leakRateLong);
        EEProm_Read_Float(1, &longestPrime);
        EEPROMtoPhonenumber(EEpromMainphoneNumber,MainphoneNumber); //get Main Phone Number from EEPROM
        EEPROMtoPhonenumber(EEpromDebugphoneNumber,DebugphoneNumber); //get Debug Phone Number from EEPROM
        EEPROMtoPhonenumber(EEpromCountryCode,CountryCode); //get Country Code from EEPROM
        EEProm_Read_Float(EEpromDiagStatus,&diagnostic); //Get the current Diagnostic Status from EEPROM
        initializeVTCC(0, BcdToDec(getMinuteI2C()), BcdToDec(getHourI2C()), BcdToDec(getDateI2C()), BcdToDec(getMonthI2C()));
        // Reset the phone numbers from EEPROM
    } else {
        ClearEEProm();
        // Only set the time if this is the first time the system is coming alive
        //   (sec, min, hr, wkday, date, month, year)
        success = setTime(localSec, localMin, localHr, localWkday, localDate, localMonth, localYear);
        print_debug_messages = 1;

        initializeVTCC(localSec, localMin, localHr, localDate, localMonth);
        year = BcdToDec(getYearI2C()); //just here to return value if RTCC failed to communicate
        //Use the pre-programmed values for MainphoneNumber, DebugphoneNumber and Country Code
        strncpy(MainphoneNumber,origMainphoneNumber,stringLength(origMainphoneNumber));
        PhonenumberToEEPROM(EEpromMainphoneNumber,MainphoneNumber);
        strncpy(DebugphoneNumber,origDebugphoneNumber,stringLength(origDebugphoneNumber));
        PhonenumberToEEPROM(EEpromDebugphoneNumber,DebugphoneNumber);
        strncpy(CountryCode,origCountryCode,stringLength(origCountryCode));
        PhonenumberToEEPROM(EEpromCountryCode,CountryCode);
        EEProm_Write_Float(EEpromDiagStatus,&diagnostic); //Save the current Diagnostic Status to EEPROM
        EEProm_Write_Float(EEpromCodeRevisionNumber,&codeRevisionNumber); //Write the code revision number to eeprom
    }
     // check and update the reset cause
    resetCause = checkResetStatus();
    timeSinceLastRestart = 0;

    // check the diagnostic status and phone number
    //EEProm_Write_Float(DiagnosticEEPromStart + 2, &LocalFloatLower);
    //EEProm_Write_Float(DiagnosticEEPromStart + 3, &LocalFloatUpper);
    // Don't do this until we have the DeianoticEEProm stuff working
    //    checkDiagnosticStatus(); 

    // do timing things
    // hour = BcdToDec(getHourI2C());
    hour = localHr; // The RTCC may not be working
    active_volume_bin = hour / 2; //Which volume bin are we starting with
    prevHour = hour; //We use previous hour in debug to know if we should send hour message to local phone

    // just so we know the board is working
    turnOnSIM();
    delayMs(2000);
    turnOffSIM();
}
/////////////////////////////////////////////////////////////////////
////                                                             ////
////                    STRING FUNCTIONS                         ////
////                                                             ////
/////////////////////////////////////////////////////////////////////

/*********************************************************************
 * Function: longLength
 * Input: number
 * Output: length
 * Overview: Returns the number of digits in the given integer
 * Note: Library
 * TestDate: 06-03-2014
 ********************************************************************/
int longLength(long num) {
    int length = 0;
    do {
        length++; //Increment length
        num /= 10; //Get rid of the one's place
    } while (num != 0);
    return length;
}

/*********************************************************************
 * Function: longToString
 * Input: integer and string
 * Output: None
 * Overview: Sets the given char array to an array filled with the digits of the given long
 * Note: Library
 * TestDate: 06-04-2014
 ********************************************************************/
void longToString(long num, char *numString) {
    //Declares an array of digits to refer to
    char const digits[] = "0123456789";
    //Gets the number of digits in the given number
    int length = longLength(num);
    //Creates a char array with the appropriate size (plus 1 for the \0 terminating character)
    char *result = numString;
    // Add 1 to the length to make room for the '-' and inserts the '-' if the number is negative
    if (num < 0) {
        length++;
        result[0] = '-';
        num *= -1; // Make the number positive
    }
    //Sets i to be the end of the string
    int i = length - 1;
    //Loops through the char array and inserts digits
    do {
        //Set the current index of the array to the corresponding digit
        result[i] = digits[num % 10];
        //Divide num by 10 to lose the one's place
        num /= 10;
        i--;
    } while (num != 0);
    //Insert a terminating \0 character
    result[length] = '\0';
}

/*********************************************************************
 * Function: stringLength
 * Input: string
 * Output: Interger
 * Overview: Returns the number of characters (not including \0) in the given string
 * Note: Library
 * TestDate: 06-09-2014
 ********************************************************************/
int stringLength(char *string) {
    int i = 0;
    //Checks for the terminating character
    while (string[i] != '\0') {
        i++;
    }
    return i;
}

/*********************************************************************
 * Function: concat
 * Input: Two strings
 * Output: None
 * Overview: Concatenates two strings
 * Note: Library
 * TestDate: 06-09-2014
 ********************************************************************/
void concat(char *dest, const char *src) {
    //Increments the pointer to the end of the string
    while (*dest) {
        dest++;
    }
    //Assigns the rest of string two to the incrementing pointer
    while ((*dest++ = *src++) != '\0');
}

/*********************************************************************
 * Function: floatToString
 * Input: float myvalue and character myString
 * Output: None
 * Overview: Fills the character array with the digits in the given float
 * Note: Make the mantissa and exponent positive if they were negative
 * TestDate: 06-20-2014
 ********************************************************************/
//Fills the char array with the digits in the given float

void floatToString(float myValue, char *myString) //tested 06-20-2014
{
    int padLength = 0; // Stores the number of 0's needed for padding (2 if the fractional part was .003)
    long digit; // Stores a digit from the mantissa
    char digitString[5]; // Stores the string version of digit
    char mString[20]; // Stores the mantissa as a string
    char eString[20]; // Stores the exponent as a string
    int decimalShift; // Keeps track of how many decimal places we counted
    long exponent = (long) myValue; // Stores the exponent value of myValue
    float mantissa = myValue - (float) exponent; //Stores the fractional part
    int sLength = 0; // The length of the final string
    // Make the mantissa and exponent positive if they were negative
    if (myValue < 0) {
        mantissa *= -1;
        exponent *= -1;
    }
    // Counts the padding needed
    while (mantissa < 1 && mantissa != 0) {
        mantissa = mantissa * 10.0; // Stores the mantissa with the given decimal accuracy
        padLength++; // Increment the number of 0's needed
    }
    padLength--; // Subtract one because we don't want to count the last place shift
    mString[0] = '\0';
    eString[0] = '\0';
    digitString[0] = '\0';
    myString[0] = '\0';
    //Gets the string for the exponent
    longToString(exponent, eString);
    // Get the mantissa digits only if needed
    // (if padLength is -1, the number was a whole number. If it is above the decimal accuracy,
    // we had all 0's and do not need a mantissa
    if (padLength > -1 && padLength < decimalAccuracy) {
        // Updates the decimal place
        decimalShift = padLength;
        // Extracts the next one's place from the mantissa until we reached our decimal accuracy
        while (decimalShift < decimalAccuracy) {
            digit = (long) mantissa; // Get the next digit
            longToString(digit, digitString); // Convert the digit to a string
            concat(mString, digitString); // Add the digit string to the mantissa string
            mantissa = mantissa - (float) digit;
            mantissa = mantissa * 10; // Shift the decimal places to prepare for the next digit
            decimalShift++; // Update the decimal shift count
        }
        if (myValue < 0) {
            concat(myString, "-"); // Adds the '-' character
            sLength++; // Add one to the length for the '-' character
        }
        // Concatenates the exponent, decimal point, and mantissa together
        concat(myString, eString);
        concat(myString, ".");
        // Adds 0's to the mantissa string for each 0 needed for padding
        int i;
        for (i = 0; i < padLength; i++) {
            concat(myString, "0");
        }
        concat(myString, mString);
        //The length of the final string (lengths of the parts plus 1 for decimal point, 1 for \0 character, and the number of 0's)
        sLength += stringLength(eString) + stringLength(mString) + 2 + padLength;
        // Removes any trailing 0's
        while (myString[sLength - 2] == '0') {
            myString[sLength - 2] = '\0';
            sLength--;
        }
    } else {
        if (myValue < 0) {
            concat(myString, "-"); // Adds the '-' character
            sLength++; // Add one to the length for the '-' character
        }
        // Concatenates the exponent, decimal point, and mantissa together
        concat(myString, eString);
        //The length of the final string (lengths of the parts plus 1 for \0 character)
        sLength += stringLength(eString) + 1;
    }
    myString[sLength - 1] = '\0'; // Add terminating character
}


/////////////////////////////////////////////////////////////////////
////                                                             ////
////                   SENSOR FUNCTIONS                          ////
////                                                             ////
/////////////////////////////////////////////////////////////////////

/*********************************************************************
* Function: readWaterSensor
 * Input: None (assumes that the WPS_On/Off signal to the 555 timer has 
 *              been set high) digitalPinSet(waterPresenceSensorOnOffPin, 1);
 * Output: 0 if water is present, 
 *         1 if it is not and 
 *         2 if the wire to the WPS is broken
 * Overview: The output of the 555 times is monitored to see what its
 *           output frequency is.  When there is no water, the frequency
 *           is less than 100hz.  If the frequency is greater than 411 hz
 *           we assume water is present.  If it is between 100 - 400hz, 
 *           the wire to the WPS must be broken or disconnected
 * 
 *           We only measure the duration of the high part of the pulse. 
 *           This routine assumes that Timer1 is clocked at 15.625khz
 * Note: Pic Dependent
 * TestDate: changed and not tested as of 1-31-2018
 ********************************************************************/
int readWaterSensor(void) // RB5 is one water sensor
{
    int WaterPresent = 0; //assume there is no water
    int QuitLooking = 0; // set to 1 if we know there is no water
    // turn WPS on and off in the Main loop 
    //delayMs(5); //make sure the 555 has had time to turn on 


    //make sure you start at the beginning of the positive pulse
    TMR1 = 0;
    if (digitalPinStatus(waterPresenceSensorPin) == 1) {
        while ((digitalPinStatus(waterPresenceSensorPin))&&(TMR1 <= NoWaterThreshold)) { //quit if the line is high for too long
        };
        if(TMR1 >= NoWaterThreshold){
            QuitLooking = 1; 
            WaterPresent = 1; //The pulse was high long enough that we know there is no water
        }
    }
   
    //wait for rising edge
    TMR1 = 0;
    while ((digitalPinStatus(waterPresenceSensorPin) == 0)&&(TMR1 <= NoWaterThreshold)&&(!QuitLooking)) { //quit if the line is low for too long
    };
    if(TMR1 >= NoWaterThreshold){
        QuitLooking = 1; 
        WaterPresent = 1; // The pulse was low long enough that we know there is no water
    }
    //Now measure the high part of the signal
    TMR1 = 0;
    while ((digitalPinStatus(waterPresenceSensorPin))&&(TMR1 <= NoWaterThreshold)&&(!QuitLooking)) { //quit if the line is high for too long
    };
    if (!QuitLooking) { // If we already know the answer don't bother with this
        if (TMR1 <= BrokenWPSThreshold) {
            WaterPresent = 0; //water is present, frequency is greater than 411 hz
        } 
        else if ((TMR1 > BrokenWPSThreshold)&&(TMR1 <= NoWaterThreshold)){
            WaterPresent = 2; // The wire to the WPS is disconnected since the 
                              // frequency is between 100hz and 400hz
        }
        else {
            WaterPresent = 1; // There is no water
        }
    }

    return WaterPresent;
}

/*********************************************************************
 * Function: initAdc()
 * Input: None
 * Output: None
 * Overview: Initializes Analog to Digital Converter
 * Note: Pic Dependent
 * TestDate: 06-02-2014
 * Code Update Date: 12-8-2016
 ********************************************************************/
void initAdc(void) {
    AD1CON1 = 0; // Default to all 0s
    AD1CON1bits.MODE12 = 0; //Use 10bit rather than 12 bit conversions
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

//problem: is in radADC

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
        case 0:
            specifyAnalogPin(depthSensorPin, 1); //make depthSensor Analog
            pinDirectionIO(depthSensorPin, 1);
            pinSampleSelectRegister(depthSensorPin);
            break;
        case 2: //Currently unused, may be used in the future.
            specifyAnalogPin(Pin4, 1); // makes Pin4 analog
            pinDirectionIO(Pin4, 1); // Pin4 in an input
            pinSampleSelectRegister(Pin4); // Connect Pin4 as the S/H input

            //ANSBbits.ANSB0 = 1; // AN2 is analog
            //TRISBbits.TRISB0 = 1; // AN2 is an input
            //AD1CHSbits.CH0SA = 2; // Connect AN2 as the S/H input
            break;
        case 4:
            specifyAnalogPin(rxPin, 1); // make rx analog
            pinDirectionIO(rxPin, 1); // makes rxPin an input
            pinSampleSelectRegister(rxPin); // Connect rxPin as the S/H input
            //ANSBbits.ANSB2 = 1; // AN4 is analog
            //TRISBbits.TRISB2 = 1; // AN4 is an input
            //AD1CHSbits.CH0SA = 4; // Connect AN4 as the S/H input
            break;
        case 11:
            specifyAnalogPin(xAxisAccelerometerPin, 1); // makes xAxis analog
            pinDirectionIO(xAxisAccelerometerPin, 1); // makes xAxis an input
            pinSampleSelectRegister(xAxisAccelerometerPin); // Connect xAxis as the S/H input
            //ANSBbits.ANSB13 = 1; // AN11 is analog
            //TRISBbits.TRISB13 = 1; // AN11 is an input
            //AD1CHSbits.CH0SA = 11; //Connect AN11 as the S/H input (sample and hold)
            break;
        case 12:
            specifyAnalogPin(yAxisAccelerometerPin, 1); // makes yAxis analog
            pinDirectionIO(yAxisAccelerometerPin, 1); // makes yAxis an input
            pinSampleSelectRegister(yAxisAccelerometerPin); // Connect yAxis as the S/H input
            //PORTBbits.RB12 = 1; // AN12 is analog ***I changed this to ANSBbits.ANSBxx 03-31-2015
            //TRISBbits.TRISB12 = 1; // AN12 is an input
            //AD1CHSbits.CH0SA = 12; // Connect AN12 as the S/H input
            break;
        case 15:
            specifyAnalogPin(batteryLevelPin, 1); // makes batteryLevelPin analog
            pinDirectionIO(batteryLevelPin, 1); // makes batteryLevelPin an input
            pinSampleSelectRegister(batteryLevelPin); // Connect batteryLevelPin
            break;
    }
    AD1CON1bits.ADON = 1; // Turn on ADC
    AD1CON1bits.SAMP = 1;
    while (!AD1CON1bits.DONE) {
    }
    unsigned int adcValue = ADC1BUF0;
    return adcValue;
}

/*********************************************************************
 * Function: getHandleAngle()
 * Input: None
 * Output: Float
 * Overview: Returns the average angle of the pump. The accelerometer
should be oriented on the pump handle so that when the
pump handle (the side the user is using) is down (water
present), the angle is positive. When the pump handle
(the side the user is using) is up (no water present),
the angle is negative.Gets a snapshot of the current sensor values.
 * Note: Library
 * NOTE2: It turns out that averaging the hangle angles would be the most accurate way to report pumping
 * TestDate: TBD
 ********************************************************************/
float getHandleAngle() {
    int i;
    float angleSum = 0;
    
    signed int xValue = readAdc(xAxis) - signedNumAdjustADC;
    signed int yValue = readAdc(yAxis) - signedNumAdjustADC;        

    float angle = atan2(yValue, xValue) * radToDegConst; //returns angle in degrees 06-20-2014

    for (i = 50; i > 0; i--) {
        angleArray[i] = angleArray[i-1];
        angleSum += angleArray[i] * filter[i];
    }
    angleArray[0] = angle;
    
    angleSum += angle * filter[0];

    return angleSum;
}

/*********************************************************************
 * Function: batteryLevel()
 * Input: None
 * Output: float
 * Overview: returns an output of a float with a value of the battery voltage compared to an
 * expected VCC of 3.6V
 * Note:
 * TestDate: 6/24/2015
 ********************************************************************/
float batteryLevel(void)//this has not been tested
{
    //    char voltageAvgFloatString[20];
    //    voltageAvgFloatString[0] = 0;
    float adcVal1;
    float adcVal2;
    float adcVal3;
    float adcAvg;
    float realVoltage;

    adcVal1 = readAdc(batteryVoltage); // - adjustmentFactor;

    delayMs(50);

    adcVal2 = readAdc(batteryVoltage); // - adjustmentFactor;

    delayMs(50);

    adcVal3 = readAdc(batteryVoltage); // - adjustmentFactor;


    adcAvg = (adcVal1 + adcVal2 + adcVal3) / 3;

    // V = adcVal / maxAdcVal * 1 / (voltage divider values) * VCC
    realVoltage = adcAvg / 1024;
    //realVoltage *= 8.879; // Unique to each board
    realVoltage *= batteryLevelConstant; // Unique to each board
    // mod  realVoltage *= 2.43;
    // mod  realVoltage *= 3.6;
    //realVoltage = adcAvg / 1024 * 1 / (100 / 243) * 3.6;

    //floatToString(battVoltage, voltageAvgFloatString);

    //return voltageAvgFloatString;

    return realVoltage;
}

/*********************************************************************
 * Function: readDepthSensor()
 * Input: None
 * Output: float
 * Overview: returns the depth of the probe in meters
 * Note: Library
 * TestDate: TBD
 ********************************************************************/
float readDepthSensor(void) {
    float adcVal1;
    float adcVal2;
    float adcVal3;
    float adcAvg;
    float depthInMeters;
    float realVoltage;
    adcVal1 = readAdc(depthSensorPin);

    delayMs(50);

    adcVal2 = readAdc(depthSensorPin);

    delayMs(50);

    adcVal3 = readAdc(depthSensorPin);


    adcAvg = (adcVal1 + adcVal2 + adcVal3) / 3;

    // V = adcVal / maxAdcVal * VCC
    realVoltage = adcAvg / 1024 * 3.6;

    //depthInMeters = 2.2629 * realVoltage * realVoltage - 5.7605 * realVoltage + 3.4137;
    depthInMeters = 2.2629 * realVoltage;
    depthInMeters *= realVoltage;
    depthInMeters -= 5.7605 * realVoltage;
    depthInMeters += 3.4137;

    return depthInMeters;

}

/////////////////////////////////////////////////////////////////////
////                                                             ////
////                    MISC FUNCTIONS                           ////
////                                                             ////
/////////////////////////////////////////////////////////////////////

/*********************************************************************
 * Function: checkResetStatus()
 * Input: void
 * Output: float
 * Overview: Returns the number of the set register of the cause of the system restart
 *********************************************************************/
float checkResetStatus(void) {
    float resetcause = RCON;
    /*if ((RCONbits.BOR == 1) && (RCONbits.POR != 1)) {
        resetcause = 1; //bit 1 is the brown out restart
    }
    else if (RCONbits.POR == 1) {
        resetcause = 0; //bit 0 is the Power-on reset
    }
    else if (RCONbits.TRAPR == 1) {
        resetcause = 15; //bit 15 is the trap conflict event
    }
    else if (RCONbits.IOPUWR == 1) {
        resetcause = 14; //bit 14 is the Illegal opcode or uninitialized W register access
    }
    else if (RCONbits.CM == 1) {
        resetcause = 9; //bit 9 is the configuration mismatch reset
    }
    else if (RCONbits.EXTR == 1) {
        resetcause = 7; //bit 7 is the !MCLR Reset
    }
    else if (RCONbits.SWR == 1) {
        resetcause = 6; //bit 6 is the RESET instruction
    }
    else if (RCONbits.WDTO == 1) {
        resetcause = 4; //bit 4 is the Watchdog Timer time-out reset
    }*/
    EEProm_Read_Float(DiagnosticEEPromStart + 1, &EEFloatData);
    resetcause = (int) EEFloatData | (int) resetcause;
    EEProm_Write_Float(DiagnosticEEPromStart + 1, &resetcause);
    RCON = 0;
    return resetcause;
}

/*********************************************************************
 * Function: degToRad()
 * Input: float
 * Output: float
 * Overview: Converts angles in degrees to angle in radians.
 * Note: Library
 * TestDate: 06-20-2014
 ********************************************************************/
float degToRad(float degrees) {
    return degrees * (PI / 180);
}

/*********************************************************************
 * Function: delayMs()
 * Input: milliseconds
 * Output: None
 * Overview: Delays the specified number of milliseconds
 * Note: Assumes that TMR1 is clocked by 15.625khz
 *       1ms is actually 1.12, 2ms = 2.12, 3ms = 3.08, 5ms = 5.12
 * TestDate: 12-20-2017 RKF
 ********************************************************************/
void delayMs(int ms) {
    int end_count;
    while (ms > 4000) {
        TMR1 = 0;
        while (TMR1 < 62500) {
        } //wait 4000ms
        ms = ms - 4000;
    }
    // now we fit within the timer's range
    end_count = ms * 15.625; // assumes TC1 is clocked by 15.625khz
    TMR1 = 0;
    while (TMR1 < end_count) {
    }
}

/*********************************************************************
 * Function: getLowerBCDAsDecimal
 * Input: int bcd
 * Output: Decimal verision of BCD added in (lower byte)
 * Overview: Returns the decimal value for the lower 8 bits in a 16 bit BCD (Binary Coded Decimal)
 * Note: Library
 * TestDate: 06-04-2014
 ********************************************************************/
int getLowerBCDAsDecimal(int bcd) //Tested 06-04-2014
{
    //Get the tens digit (located in the second nibble from the right)
    //by shifting off the ones digit and anding
    int tens = (bcd >> 4) & 0b0000000000001111;
    //Get the ones digit (located in the first nibble from the right)
    //by anding (no bit shifting)
    int ones = bcd & 0b0000000000001111;
    //Returns the decimal value by multiplying the tens digit by ten
    //and adding the ones digit
    return (tens * 10) +ones;
}

//Returns the decimal value for the upper 8 bits in a 16 bit BCD (Binary Coded Decimal)

/*********************************************************************
 * Function: getUpperBCDAsDecimal
 * Input: int bcd
 * Output: Decimal verision of BCD added in (upper byte)
 * Overview: Returns the decimal value for the Upper 8 bits in a 16 bit BCD (Binary Coded Decimal)
 * Note: Library
 * TestDate: 06-04-2014
 ********************************************************************/
int getUpperBCDAsDecimal(int bcd) //Tested 06-04-2014
{
    //Get the tens digit (located in the first nibble from the left)
    //by shifting off the rest and anding
    int tens = (bcd >> 12) & 0b0000000000001111;
    //Get the ones digit (located in the second nibble from the left)
    //by shifting off the rest and anding
    int ones = (bcd >> 8) & 0b0000000000001111;
    //Returns the decimal value by multiplying the tens digit by ten
    //and adding the ones digit
    return (tens * 10) +ones;
}

/*********************************************************************
 * Function: setInternalRTCC()
 * Input: SS MM HH WW DD MM YY
 * Output: None
 * Overview: Initializes values for the internal RTCC
 * Note: 
 ********************************************************************/
void setInternalRTCC(int sec, int min, int hr, int wkday, int dte, int mnth, int yr) {

    __builtin_write_RTCWEN(); //does unlock sequence to enable write to RTCC, sets RTCWEN

    RCFGCALbits.RTCWREN = 1; // Allow user to change RTCC values
    RCFGCALbits.RTCPTR = 0b11; //Point to the top (year) register

    RTCVAL = DecToBcd(yr); // RTCPTR decrements automatically after this
    RTCVAL = DecToBcd(dte) + (DecToBcd(mnth) << 8);
    RTCVAL = DecToBcd(hr) + (DecToBcd(wkday) << 8);
    RTCVAL = DecToBcd(sec) + (DecToBcd(min) << 8); // = binaryMinuteSecond;

    _RTCEN = 1; // = 1; //RTCC module is enabled
    _RTCWREN = 0; // = 0; // disable writing

}

//Returns the hour of day from the internal clock
/*********************************************************************
 * Function: getTimeHour
 * Input: None
 * Output: hourDecimal
 * Overview: Returns the hour of day from the internal clock
 * Note: Pic Dependent
 * TestDate: 06-04-2014
 ********************************************************************/
//Tested 06-04-2014

int getTimeHour(void) {
    //don't want to write, just want to read
    _RTCWREN = 0;
    //sets the pointer to 0b01 so that reading starts at Weekday/Hour
    _RTCPTR = 0b01;
    // Ask for the hour from the internal clock
    int myHour = RTCVAL;
    int hourDecimal = getLowerBCDAsDecimal(myHour);
    return hourDecimal;
}

/*********************************************************************
 * Function: getTimeMinute
 * Input: None
 * Output: minuteDecimal
 * Overview: Returns the minute of the hour from the internal clock
 * Note: Pic Dependent
 * TestDate: NA
 ********************************************************************/
//Tested NA

int getTimeMinute(void) {
    //don't want to write, just want to read
    _RTCWREN = 0;
    //sets the pointer to 0b00 so that reading starts at Minutes/Seconds
    _RTCPTR = 0b00;
    // Ask for the hour from the internal clock
    int myMinute = RTCVAL;
    int minuteDecimal = getLowerBCDAsDecimal(myMinute);
    return minuteDecimal;
}


/* First, retrieve time string from the SIM 900 (I think this is reading the PIC RTCC not the SIM 900 rkf)
Then, parse the string into separate strings for each time partition
Next, translate each time partition, by digit, into a binary string
Finally, piece together strings (16bytes) and write them to the RTCC */
// Tested 06-02-2014

/*********************************************************************
 * Function: timeStamp()
 * Input: void
 * Output: long timeStampValue
 * Overview: Returns the current time in seconds (the seconds passed so far in the day)
 * Note:
 * TestDate: 06-04-2014
 ********************************************************************/
long timeStamp(void) {
    long timeStampValue = 0;
    //Set the pointer to 0b01 so that reading starts at Weekday/Hour
    _RTCPTR = 0b01; // decrements with read or write
    _RTCWREN = 0; //don't want to write, just want to read
    long binaryWeekdayHour = RTCVAL; // write wkdy & hour to variable, dec. RTCPTR
    long binaryMinuteSecond = RTCVAL; // write min & sec to variable, dec. RTCPTR
    //For some reason, putting the multiplication for hours on one line like this:
    //
    // timeStampValue = getLowerBCDAsDecimal(binaryWeekdayHour) * 60 * 60;
    //
    //caused an error. We would get some unknown value for the timestamp, so
    //we had to break the code up across multiple lines. So don't try to
    //simplify this!
    timeStampValue = getLowerBCDAsDecimal(binaryWeekdayHour);
    timeStampValue = timeStampValue * 60 * 60;
    timeStampValue = timeStampValue + (getUpperBCDAsDecimal(binaryMinuteSecond) * 60);
    timeStampValue = timeStampValue + getLowerBCDAsDecimal(binaryMinuteSecond);
    return timeStampValue; //timeStampValue;
}

/*********************************************************************
 * Function: ResetMsgVariables() 
 * Input: None
 * Output: None
 * Overview: Moves EEPROM saved data to locations for the next day and clears 
 *           positions that held data sent in the noon message
 * 
 * Note: 
 * TestDate: 
 ********************************************************************/
void ResetMsgVariables() //Not Tested
{
    // Move today's 0-12AM values into the yesterday positions
    // There is no need to relocate data from 12-24PM since it has not yet been measured
    // This should only be done if the message was able to be sent
    EEProm_Read_Float(14, &EEFloatData); // Overwrite saved volume with today's value
    EEProm_Write_Float(2, &EEFloatData);
    EEProm_Read_Float(15, &EEFloatData); // Overwrite saved volume with today's value
    EEProm_Write_Float(3, &EEFloatData);
    EEProm_Read_Float(16, &EEFloatData); // Overwrite saved volume with today's value
    EEProm_Write_Float(4, &EEFloatData);
    EEProm_Read_Float(17, &EEFloatData); // Overwrite saved volume with today's value
    EEProm_Write_Float(5, &EEFloatData);
    EEProm_Read_Float(18, &EEFloatData); // Overwrite saved volume with today's value
    EEProm_Write_Float(6, &EEFloatData);
    EEProm_Read_Float(19, &EEFloatData); // Overwrite saved volume with today's value
    EEProm_Write_Float(7, &EEFloatData);
    // Clear saved leakRateLong and longestPrime
    leakRateLong = 0; //Clear local and saved value 
    EEProm_Write_Float(0, &leakRateLong);
    longestPrime = 0; //Clear local and saved value
    EEProm_Write_Float(1, &longestPrime);

    // The RAM location for each volume bin was cleared when the value was saved to EEPROM

    //Clear slots for volume 1214-2224 to make sure they are zero in case there is no power to fill
    EEFloatData = 0.01;
    EEProm_Write_Float(8, &EEFloatData);
    EEProm_Write_Float(9, &EEFloatData);
    EEProm_Write_Float(10, &EEFloatData);
    EEProm_Write_Float(11, &EEFloatData);
    EEProm_Write_Float(12, &EEFloatData);
    EEProm_Write_Float(13, &EEFloatData);
    EEProm_Write_Float(14, &EEFloatData);
    EEProm_Write_Float(15, &EEFloatData);
    EEProm_Write_Float(16, &EEFloatData);
    EEProm_Write_Float(17, &EEFloatData);
    EEProm_Write_Float(18, &EEFloatData);
    EEProm_Write_Float(19, &EEFloatData);
}

/*********************************************************************
 * Function: translate()
 * Input: String
 * Output: int binaryNumber
 * Overview: The following integers are used for turning the corresponding time-value strings
into binary numbers that are used to program the RTCC registers
 * Note: Library
 * TestDate: 06-02-2014
 ********************************************************************/
int translate(char digit) {
    int binaryNumber;
    if (digit == '0') {
        binaryNumber = 0b0000;
    } else if (digit == '1') {
        binaryNumber = 0b0001;
    } else if (digit == '2') {
        binaryNumber = 0b0010;
    } else if (digit == '3') {
        binaryNumber = 0b0011;
    } else if (digit == '4') {
        binaryNumber = 0b0100;
    } else if (digit == '5') {
        binaryNumber = 0b0101;
    } else if (digit == '6') {
        binaryNumber = 0b0110;
    } else if (digit == '7') {
        binaryNumber = 0b0111;
    } else if (digit == '8') {
        binaryNumber = 0b1000;
    } else if (digit == '9') {
        binaryNumber = 0b1001;
    }
    return binaryNumber;
}

/*********************************************************************
 * Function: RTCCSet()
 * Input: None
 * Output: None
 * Overview: Get time string from SIM900 (actually external RTCC MCP7490N - rkf)
 * Note: Pic Dependent
 * TestDate: 06-02-2014
 ********************************************************************/
void RTCCSet(void) {
    // Write the time to the RTCC
    // The enclosed code was graciously donated by the KWHr project
    __builtin_write_RTCWEN(); //does unlock sequence to enable write to RTCC, sets RTCWEN
    RCFGCAL = 0b0010001100000000;
    RTCPWC = 0b0000010100000000;
    _RTCPTR = 0b11; // decrements with read or write
    // Thanks KWHr!!!
    RTCVAL = getYearI2C();
    RTCVAL = getDateI2C() + (getMonthI2C() << 8);
    RTCVAL = getHourI2C() + (1 << 8); // assume 1st day of the week
    RTCVAL = 0 + (getMinuteI2C() << 8); // = binaryMinuteSecond; (assume seconds = 0)
    _RTCEN = 1; // = 1; //RTCC module is enabled
    _RTCWREN = 0; // = 0; // disable writing
}

//Returns the minutes and seconds (in BCD) to set the alarm to.
//Generates a random number of seconds between 1 and the alarmMinuteMax
//global variable to use for the minutes and seconds.

/*********************************************************************
 * Function: void VerifyProperTimeSource(void)
 * Input: None
 * Output: None
 * Overview: See if the RTCC is responding and the hour is not stuck.  If 
 *           that is the case, continue using RTCC as the source of keeping
 *           track of time.  If the RTCC is not talking or is stuck, switch
 *           over to using the VTCC.  If the RTCC begins to work again, switch
 *           back to it.
 * TestDate: Not Tested
 ********************************************************************/
void VerifyProperTimeSource(void) {
    // Use the flow chart to write the code for this.
    float RTCChour;
    if (!extRTCCset) { //The RTCC is the source of system Time
        RTCChour = BcdToDec(getHourI2C());
        //sendDebugMessage("Using RTCC hour is = ", RTCChour);  //Debug
        //sendDebugMessage("      RTCC min is = ", BcdToDec(getMinuteI2C()));  //Debug
        //sendDebugMessage("      VTCC min is = ", minuteVTCC);  //Debug

        if (!extRtccTalked) { //We did not get a response from the RTCC
            extRTCCset = 1; //The RTCC did not respond, turn control over to VTCC
            //sendDebugMessage("The RTCC did not reply to I2C ", 0);  //Debug
        } else { //We got a response, but it may not be good
            if (RTCChour != prevHour) { //The hour has changed
                //sendDebugMessage("Its a new hour and previous hour = ",prevHour);  //Debug
                if ((RTCChour == 0)&&(prevHour != 23)) {
                    extRTCCset = 1; //The RTCC value is invalid, turn control over to VTCC
                }
                if ((RTCChour > 0)&&(RTCChour != prevHour + 1)) {
                    extRTCCset = 1; //The RTCC value is invalid, turn control over to VTCC
                }
            } else { //Is it OK that the hour did not change?
                if ((RTCChour != hourVTCC)&& (minuteVTCC >= 2)) {//The hour has not changed, but it should have
                    //sendDebugMessage("Must be stuck, VTCC hour is = ", hourVTCC);  //Debug
                    //sendDebugMessage("    and VTCC minute = ", minuteVTCC);  //Debug
                    extRTCCset = 1; //The RTCC is stuck, turn control over to VTCC
                }
            }
        }
        if (extRTCCset) {// We switched over to VTCC 
            setTime(0, minuteVTCC, hourVTCC, 1, dateVTCC, monthVTCC, 18); //Try to update RTCC from VTCC
            hour = hourVTCC;
            //sendDebugMessage("Just switched over to the VTCC, hour is = ", hour);  //Debug
        }
        // At this point we know whether or not we can trust the RTCC
        if ((RTCChour != prevHour)&&(!extRTCCset)) { // if the RTCC is good, resync the VTCC each hour
            initializeVTCC(0, BcdToDec(getMinuteI2C()), BcdToDec(getHourI2C()), BcdToDec(getDateI2C()), BcdToDec(getMonthI2C()));
            hour = RTCChour; //update the system hour to the RTCC value
            //sendDebugMessage("We trust the RTCC, update the VTCC.  The system hour is = ", hour);  //Debug
        }
    } else { // Coming into this routine, the VTCC is the source of system Time.
        sendDebugMessage("We are using the VTCC for time, hour is = ", hourVTCC); //Debug    
        sendDebugMessage("                                minute = ", minuteVTCC); //Debug
        if ((hourVTCC != prevHour)&&(minuteVTCC >= 2)) {// Time to reevaluate the RTCC
            hour = hourVTCC; //Still use VTCC for system Time at this point
            RTCChour = BcdToDec(getHourI2C()); //Check RTCC hour
            //sendDebugMessage("Time to reevaluate the RTCC.  RTCC hour =  ",RTCChour);  //Debug
            if (!extRtccTalked) { //We did not get a response from the RTCC
                //continue VTCC control of system Time and try to resync RTCC
                // Once a RESET_I2C routine is written, put it here to try to get I2C working again.                
                //sendDebugMessage("No response from RTCC, try to set its time ",0);  //Debug
                setTime(0, minuteVTCC, hourVTCC, 1, dateVTCC, monthVTCC, 18); //Try to update RTCC from VTCC
            } else { //We got a response, but it may not be good
                if (RTCChour != hourVTCC) {
                    //continue VTCC control of system Time and try to resync RTCC
                    setTime(0, minuteVTCC, hourVTCC, 1, dateVTCC, monthVTCC, 18); //Try to update RTCC from VTCC
                    //sendDebugMessage("RTCC hour does not match VTCC, must still be stuck ",0);  //Debug
                } else {// Looks like the RTCC is working again
                    extRTCCset = 0; //Turn control over to RTCC
                    initializeVTCC(0, BcdToDec(getMinuteI2C()), BcdToDec(getHourI2C()), BcdToDec(getDateI2C()), BcdToDec(getMonthI2C()));
                    //sendDebugMessage("RTCC seems to be working, turn control back over to it and resync the VTCC. Use minute = ",BcdToDec(getMinuteI2C()));  //Debug
                }
            }
        }
    }
}

/*********************************************************************
 * Function: getMinuteOffset()
 * Input: None
 * Output: int time in BCD
 * Overview: Randomizes sending of text messages a couple minutes after midnight.
 * Note: Library
 * TestDate: 06-13-2014
 ********************************************************************/
int getMinuteOffset() {
    //Get the number of seconds possible in alarmMinuteMax minuites plus 10 seconds
    //Plus 10 seconds is so that we aren't calling the alarm right at midnight
    int minutesInSeconds = (alarmMinuteMax * 60) + 10;
    //Sets the seed randomly based on the time of day
    long time = timeStamp(); // Gets the time of day in seconds (long)
    int timeConverted = (int) (time / 3); // Convert the time into an int to be supported by srand()
    srand(timeConverted);
    //Get a random time (in seconds)
    int randomTime = rand() % minutesInSeconds;
    //Get the minute part (plus the starting offset minute)
    int minutes = (randomTime / 60) + alarmStartingMinute;
    //Get the remaining seconds after minutes are taken out
    int seconds = randomTime % 60;
    //Get the tens and ones place for the minute
    int minuteTens = minutes / 10;
    int minuteOnes = minutes % 10;
    //Get the tens and ones place for the second
    int secondsTens = seconds / 10;
    int secondsOnes = seconds % 10;
    // Five minutes and one second (for an example reference)
    // 0x0501
    // 0b0000 0101 0000 0001
    //Get the time in BCD by shifting the minutes tens place
    int timeInBCD = minuteTens << 12;
    //Add the shifted minutes ones place
    timeInBCD = timeInBCD + (minuteOnes << 8);
    //Add the shifted seconds tens place
    timeInBCD = timeInBCD + (secondsTens << 4);
    //Add the seconds ones place
    timeInBCD = timeInBCD + secondsOnes;
    return timeInBCD;
}

//This function converts a BCD to DEC
//Input: BCD Value
//Returns: Hex Value

char BcdToDec(char val) {
    return ((val / 16 * 10) + (val % 16));
}

//This function converts HEX to BCD
//Input: Hex Value
//Returns: BCD Value

char DecToBcd(char val) {
    return ((val / 10 * 16) + (val % 10));
}

void midDayDepthRead(void) {
    if (depthSensorInUse == 1) {
        pinDirectionIO(depthSensorOnOffPin, 0); //makes depth sensor pin an output.
        digitalPinSet(depthSensorOnOffPin, 1); //turns on the depth sensor.

        delayMs(30000); // Wait 30 seconds for the depth sensor to power up

        midDayDepth = readDepthSensor();

        digitalPinSet(depthSensorOnOffPin, 0); //turns off the depth sensor.
        delayMs(1000);
        prevDayDepthSensor = BcdToDec(getDateI2C());

    }
}

/*********************************************************************
 * Function: EEProm_Write_Int(int addr, int newData)
 * Input: addr - the location to write to relative to the start of EEPROM
 *        newData - - Floating point value to write to EEPROM
 * Output: none
 * Overview: The value passed by newData is written to the location in EEPROM
 *           which is multiplied by 2 to only use addresses with even values
 *           and is then offset up from the start of EEPROM
 * Note: Library
 * TestDate: 12-26-2016
 ********************************************************************/
void EEProm_Write_Int(int addr, int newData) {
    unsigned int offset;
    NVMCON = 0x4004;
    // Set up a pointer to the EEPROM location to be erased
    TBLPAG = __builtin_tblpage(&eeData); // Initialize EE Data page pointer
    offset = __builtin_tbloffset(&eeData) + (2 * addr & 0x01ff); // Initizlize lower word of address
    __builtin_tblwtl(offset, newData); // Write EEPROM data to write latch
    asm volatile ("disi #5"); // Disable Interrupts For 5 Instructions
    __builtin_write_NVM(); // Issue Unlock Sequence & Start Write Cycle
    while (NVMCONbits.WR == 1); // Optional: Poll WR bit to wait for
    // write sequence to complete
}

/*********************************************************************
 * Function: int EEProm_Read_Int(int addr);
 * Input: addr - the location to read from relative to the start of EEPROM
 * Output: int value read from EEPROM
 * Overview: A single int is read from EEPROM start + offset and is returned
 * Note: Library
 * TestDate: 12-26-2016
 ********************************************************************/
int EEProm_Read_Int(int addr) {
    int data; // Data read from EEPROM
    unsigned int offset;

    // Set up a pointer to the EEPROM location to be erased
    TBLPAG = __builtin_tblpage(&eeData); // Initialize EE Data page pointer
    offset = __builtin_tbloffset(&eeData) + (2 * addr & 0x01ff); // Initialize lower word of address
    data = __builtin_tblrdl(offset); // Write EEPROM data to write latch
    return data;
}

/*********************************************************************
 * Function: EEProm_Read_Float(unsigned int ee_addr, void *obj_p)
 * Input: ee_addr - the location to read from relative to the start of EEPROM
 *        *obj_p - the address of the variable to be updated (assumed to be a float)
 * Output: none
 * Overview: A single float is read from EEPROM start + offset.  This is done by
 *           updating the contents of the float address provided, one int at a time
 * Note: Library
 * TestDate: 12-26-2016
 ********************************************************************/


void EEProm_Read_Float(unsigned int ee_addr, void *obj_p) {
    unsigned int *p = obj_p; //point to the float to be updated
    unsigned int offset;

    ee_addr = ee_addr * 4; // floats use 4 address locations

    // Read and update the first half of the float
    // Set up a pointer to the EEPROM location to be erased
    TBLPAG = __builtin_tblpage(&eeData); // Initialize EE Data page pointer
    offset = __builtin_tbloffset(&eeData) + (ee_addr & 0x01ff); // Initialize lower word of address
    *p = __builtin_tblrdl(offset); // Write EEPROM data to write latch
    // First half read is complete

    p++;
    ee_addr = ee_addr + 2;

    TBLPAG = __builtin_tblpage(&eeData); // Initialize EE Data page pointer
    offset = __builtin_tbloffset(&eeData) + (ee_addr & 0x01ff); // Initialize lower word of address
    *p = __builtin_tblrdl(offset); // Write EEPROM data to write latch
    // second half read is complete

}

/*********************************************************************
 * Function: EEProm_Write_Float(unsigned int ee_addr, void *obj_p)
 * Input: ee_addr - the location to write to relative to the start of EEPROM
 *                  it is assumed that you are referring to the # of the float 
 *                  you want to write.  The first is 0, the next is 1 etc.
 *        *obj_p - the address of the variable which contains the float
 *                  to be written to EEPROM
 * Output: none
 * Overview: A single float is written to EEPROM start + offset.  This is done by
 *           writing the contents of the float address provided, one int at a time
 * Note: Library
 * TestDate: 12-26-2016
 ********************************************************************/
void EEProm_Write_Float(unsigned int ee_addr, void *obj_p) {
    unsigned int *p = obj_p;
    unsigned int offset;
    NVMCON = 0x4004;
    ee_addr = ee_addr * 4; // floats use 4 address locations

    // Write the first half of the float
    // Set up a pointer to the EEPROM location to be erased
    TBLPAG = __builtin_tblpage(&eeData); // Initialize EE Data page pointer
    offset = __builtin_tbloffset(&eeData) + (ee_addr & 0x01ff); // Initizlize lower word of address
    __builtin_tblwtl(offset, *p); // Write EEPROM data to write latch
    asm volatile ("disi #5"); // Disable Interrupts For 5 Instructions
    __builtin_write_NVM(); // Issue Unlock Sequence & Start Write Cycle
    while (NVMCONbits.WR == 1); // Optional: Poll WR bit to wait for
    // first half of float write sequence to complete

    // Write the second half of the float
    p++;
    ee_addr = ee_addr + 2;
    TBLPAG = __builtin_tblpage(&eeData); // Initialize EE Data page pointer
    offset = __builtin_tbloffset(&eeData) + (ee_addr & 0x01ff); // Initizlize lower word of address
    __builtin_tblwtl(offset, *p); // Write EEPROM data to write latch
    asm volatile ("disi #5"); // Disable Interrupts For 5 Instructions
    __builtin_write_NVM(); // Issue Unlock Sequence & Start Write Cycle
    while (NVMCONbits.WR == 1); // Optional: Poll WR bit to wait for
    // second half of float write sequence to complete

}

/*********************************************************************
 * Function: ClearWatchDogTimer()
 * Input: none
 * Output: none
 * Overview: You can use just ClrWdt() as a command.  However, I read in a forum 
 *           http://www.microchip.com/forums/m122062.aspx
 *           that since ClrWdt() expands to an asm command, the presence of the 
 *          asm will stop the compiler from optimizing any routine that it is a 
 *          part of.  Since I want to call this in Main, that would be a problem
 * Note: Library
 * TestDate: 1-2-2017
 ********************************************************************/
void ClearWatchDogTimer(void) {
    ClrWdt();
}

/*********************************************************************
 * Function: SaveVolumeToEEProm(void)
 * Input: none
 * Output: none
 * Overview: Volume is saved in 2hr long bins.  When a new one begins, the total
 *          from the last bin should be saved from RAM to EEProm.
 * Note: Library
 * TestDate: 1-4-2017
 ********************************************************************/
void SaveVolumeToEEProm(void) {
    switch (hour / 2) {
        case 0:
            EEProm_Write_Float(13, &volume2224);
            volume2224 = 0; //Clear for the next days readings
            active_volume_bin = hour / 2;
            break;
        case 1:
            EEProm_Write_Float(14, &volume02);
            volume02 = 0; //Clear for the next days readings
            active_volume_bin = hour / 2;
            break;
        case 2:
            EEProm_Write_Float(15, &volume24);
            volume24 = 0; //Clear for the next days readings
            active_volume_bin = hour / 2;
            break;
        case 3:
            EEProm_Write_Float(16, &volume46);
            volume46 = 0; //Clear for the next days readings
            active_volume_bin = hour / 2;
            break;
        case 4:
            EEProm_Write_Float(17, &volume68);
            volume68 = 0; //Clear for the next days readings
            active_volume_bin = hour / 2;
            break;
        case 5:
            EEProm_Write_Float(18, &volume810);
            volume810 = 0; //Clear for the next days readings
            active_volume_bin = hour / 2;
            break;
        case 6:
            EEProm_Write_Float(19, &volume1012);
            volume1012 = 0; //Clear for the next days readings
            active_volume_bin = hour / 2;
            break;
        case 7:
            EEProm_Write_Float(8, &volume1214);
            volume1214 = 0; //Clear for the next days readings
            active_volume_bin = hour / 2;
            break;
        case 8:
            EEProm_Write_Float(9, &volume1416);
            volume1416 = 0; //Clear for the next days readings
            active_volume_bin = hour / 2;
            break;
        case 9:
            EEProm_Write_Float(10, &volume1618);
            volume1618 = 0; //Clear for the next days readings
            active_volume_bin = hour / 2;
            break;
        case 10:
            EEProm_Write_Float(11, &volume1820);
            volume1820 = 0; //Clear for the next days readings
            active_volume_bin = hour / 2;
            break;
        case 11:
            EEProm_Write_Float(12, &volume2022);
            volume2022 = 0; //Clear for the next days readings
            active_volume_bin = hour / 2;
            break;
    }
}

/*********************************************************************
 * Function: DebugReadEEProm(void)
 * Input: none
 * Output: none
 * Overview: This function reads all of the EEProm dedicated to Priming, Leak
 *           and the volume bins and sends them to the UART serial pins
 *           where they can be viewed with a Protocol interface
 * Note: Library
 * TestDate: 1-4-2017
 ********************************************************************/
void DebugReadEEProm(void) {
    EEProm_Read_Float(0, &EEFloatData);
    sendDebugMessage("Leak Rate = ", EEFloatData); //Debug
    EEProm_Read_Float(1, &EEFloatData);
    sendDebugMessage("Longest Prime = ", EEFloatData); //Debug
    EEProm_Read_Float(2, &EEFloatData);
    sendDebugMessage("Volume 02 = ", EEFloatData); //Debug
    EEProm_Read_Float(3, &EEFloatData);
    sendDebugMessage("Volume 24 = ", EEFloatData); //Debug
    EEProm_Read_Float(4, &EEFloatData);
    sendDebugMessage("Volume 46 = ", EEFloatData); //Debug
    EEProm_Read_Float(5, &EEFloatData);
    sendDebugMessage("Volume 68 = ", EEFloatData); //Debug
    EEProm_Read_Float(6, &EEFloatData);
    sendDebugMessage("Volume 810 = ", EEFloatData); //Debug
    EEProm_Read_Float(7, &EEFloatData);
    sendDebugMessage("Volume 1012 = ", EEFloatData); //Debug
    EEProm_Read_Float(8, &EEFloatData);
    sendDebugMessage("Volume 1214 = ", EEFloatData); //Debug
    EEProm_Read_Float(9, &EEFloatData);
    sendDebugMessage("Volume 1416 = ", EEFloatData); //Debug
    EEProm_Read_Float(10, &EEFloatData);
    sendDebugMessage("Volume 1618 = ", EEFloatData); //Debug
    EEProm_Read_Float(11, &EEFloatData);
    sendDebugMessage("Volume 1820 = ", EEFloatData); //Debug
    EEProm_Read_Float(12, &EEFloatData);
    sendDebugMessage("Volume 2022 = ", EEFloatData); //Debug
    EEProm_Read_Float(13, &EEFloatData);
    sendDebugMessage("Volume 2224 = ", EEFloatData); //Debug
    EEProm_Read_Float(14, &EEFloatData);
    sendDebugMessage("Today Volume 02 = ", EEFloatData); //Debug
    EEProm_Read_Float(15, &EEFloatData);
    sendDebugMessage("Today Volume 24 = ", EEFloatData); //Debug
    EEProm_Read_Float(16, &EEFloatData);
    sendDebugMessage("Today Volume 46 = ", EEFloatData); //Debug
    EEProm_Read_Float(17, &EEFloatData);
    sendDebugMessage("Today Volume 68 = ", EEFloatData); //Debug
    EEProm_Read_Float(18, &EEFloatData);
    sendDebugMessage("Today Volume 810 = ", EEFloatData); //Debug
    EEProm_Read_Float(19, &EEFloatData);
    sendDebugMessage("Today Volume 1012 = ", EEFloatData); //Debug
}

/*********************************************************************
 * Function: ClearEEProm(void)
 * Input: none
 * Output: none
 * Overview: This function writes a 0 to all of the EEPROM addresses
 *           in EEProm.  It should be called the first time a board
 *           is programmed but NOT every time we Initialize since we don't want 
 *           to lose data saved prior to shutting down because of lost power
 * Note: Library
 * TestDate: 1-5-2017 (changed to clear all EEPROM and not retested)
 ********************************************************************/
void ClearEEProm(void) {
    int i;
    EEFloatData = 0;
    for (i = 0; i <= DiagnosticEEPromStart + 3; i++) {
        EEProm_Write_Float(i, &EEFloatData);
    }
}

void __attribute__((interrupt, auto_psv)) _U1RXInterrupt(void) { //Receive UART data interrupt
    // Here is where we put the code to read a character from the Receive data buffer
    // and concatenate it onto a receive message string.

    while (U1STAbits.URXDA) {
        ReceiveTextMsg[NumCharInTextMsg] = U1RXREG;
        if (ReceiveTextMsg[NumCharInTextMsg] == 0x0A) {//is this a line feed.
            ReceiveTextMsgFlag++;
        }
        NumCharInTextMsg++;
        ReceiveTextMsg[NumCharInTextMsg] = 0;
    }
    // Always reset the interrupt flag
    IFS0bits.U1RXIF = 0;


}

/*********************************************************************
 * Function: void initializeVTCC(char sec, char min, char hr, char date, char month)
 * Input: char sec, char min, char hr, char date, char month
 * Output: none
 * Overview:  Sets the virtual real time clock with the passed variables. 
 * TestDate: 
 ********************************************************************/

void initializeVTCC(char sec, char min, char hr, char dte, char mnth) {
    secondVTCC = sec;
    minuteVTCC = min;
    hourVTCC = hr;
    dateVTCC = dte;
    monthVTCC = mnth;
    TMR2 = 0;
    PR2 = 62500; //set cycle time to 4sec
    IFS0bits.T2IF = 0;
    IEC0bits.T2IE = 1;
}

/*********************************************************************
 * Function: void updateVTCC(void)
 * Input: none - add to secondVTCC prior to calling this function.
 * Output: none
 * Overview:  Both the T2 interrupt and the Sleep routine need to update the VTCC
 *            to keep it as accurate as possible in the event that it is our
 *            source of system time.
 *             
 * TestDate: 
 ********************************************************************/

void updateVTCC(void) {
    while (secondVTCC >= 60) {
        secondVTCC = secondVTCC - 60;
        minuteVTCC++;
        TimeSinceLastHourCheck = 1;
    }
    while (minuteVTCC >= 60) {
        minuteVTCC = minuteVTCC - 60;
        hourVTCC++;
    }
    while (hourVTCC >= 24) {
        hourVTCC = hourVTCC - 24;
        dateVTCC++;
    }
    if (dateVTCC > monthArray[monthVTCC-1]) {
        dateVTCC = 1;
        monthVTCC++;
        if (monthVTCC == 13) {
            monthVTCC = monthVTCC - 12;
        }
    }
}
/*********************************************************************
 * Function: int HasTheHandleMoved(float rest_position)
 * Input: angle in degrees of the handle when it is assumed to be at rest.  
 *        This does not need to be at one of the stops.  A person could be holdig it anywhere but not moving it up and down.
 * Output: 0 if the handle has not moved enough (handleMovementThreshold) to say that it is really moving
 *         1 if the current position is at least handleMovementThreshold away from the assumed rest position
 * Overview:  Before pumping begins, or when it stops the handle position is noted.
 *            by looking at the current position relative to this measured resting position,
 *            we can decide if there has been enough movement to indicate that
 *            someone is deliberately moving it to try to pump water. *             
 * TestDate: NOT TESTED
 ********************************************************************/
int HasTheHandleMoved(float rest_position){
    int moving = 0;  // assume that the handle is not moving
    float deltaAngle = getHandleAngle() - rest_position;
    if(deltaAngle < 0) {
        deltaAngle *= -1;
    }
    if(deltaAngle > handleMovementThreshold){ // The total movement of the handle from rest has been exceeded
		moving = 1;
    }
    return moving;
}


/*********************************************************************
 * Function: void _T2Interrupt(void)
 * Input: none
 * Output: none
 * Overview:  An interrupt that advances the virtual real time clock by four seconds and roles over each variable when necessary
 * TestDate: 
 ********************************************************************/

void __attribute__((interrupt, auto_psv)) _T2Interrupt(void) {
    // assuming we are entering this when the over flow occurs
    IFS0bits.T2IF = 0; //clear the flag
    secondVTCC = secondVTCC + 4;
    updateVTCC();
}