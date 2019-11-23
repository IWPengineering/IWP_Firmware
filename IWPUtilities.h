/*
* File:   IWPUtilities.h
* Author: js1715
*
* Created on May 29, 2015, 4:41 PM
*/

#ifndef IWPUTILITIES_H
#define	IWPUTILITIES_H


//ENUM declarations

enum RTCCregister
{
    SEC_REGISTER,
    MIN_REGISTER,
    HOUR_REGISTER,
    WKDAY_REGISTER,
    DATE_REGISTER,
    MONTH_REGISTER,
    YEAR_REGISTER
};


//These variables were changed to be constants so that their values would
//not be changed accidentally - 6/6/14 Avery deGruchy
// ****************************************************************************
// *** Constants **************************************************************
// ****************************************************************************
extern const int xAxis; // analog pin connected to x axis of accelerometer
extern const int yAxis; // analog pin connected to y axis of accelerometer
extern const int batteryVoltage;                  // analog pin connected to the battery
extern const float MKII; // 0.4074 L/Radian; transfer variable for mkII delta handle angle to outflow

extern const float a; // a in quadratic equation to solve for volume
extern const float b; // b in quadratic equation to solve for volume
extern const float c; // c in quadratic equation to solve for volume
extern const float quadVertex; // the y value of the vertex of the parabola used to calculate volume; = (-(b^2)/(4*a))+c

extern const float leakSensorVolume; // This is in Liters; pipe dia. = 33mm; rod diam 12 mm; gage length 24mm
extern const int alarmHour; // The weekday and hour (24 hour format) (in BCD) that the alarm will go off
extern const int alarmStartingMinute; // The minimum minute that the alarm will go off
extern const int alarmMinuteMax; // The max number of minutes to offset the alarm (the alarmStartingMinute + a random number between 0 and this number)
extern const int signedNumAdjustADC; // Used to divide the total range of the output of the 10 bit ADC into positive and negative range.
extern const int pulseWidthThreshold; // The value to check the pulse width against (2048)

extern const int upstrokeInterval; // The number of milliseconds to delay before reading the upstroke
extern const int max_pause_while_pumping; //The maximum time (in loops, each loop delaying 10ms) that the pump handle is not moving before we say that the person stopped trying
extern long leakRateTimeOut; // Equivalent to 18 seconds (in 50 millisecond intervals); 50 = upstrokeInterval
//extern long timeBetweenUpstrokes; // 3 seconds (based on upstrokeInterval)
extern const int decimalAccuracy; // Number of decimal places to use when converting floats to strings
extern const float angleThresholdSmall; //number close to zero to determine if handle is moving w/o interpreting accelerometer noise as movement.
extern const float angleThresholdLarge; //total angle movement to accumulate before identifying movement as intentional pumping
extern const float handleMovementThreshold; //When the handle moves more then this number of degrees from rest, it is time to calculate Priming
extern const float PI;
extern const float upstrokeToMeters;
extern const int minimumAngleDelta;
extern const float batteryLevelConstant;       //This number is found by Vout = (R32 * Vin) / (R32 + R31), Yields Vin = Vout / 0.476
extern const float BatteryDyingThreshold;
extern int queueCount;
extern int queueLength; //don't forget to change angleQueue to this number also
extern float angleQueue[7];
extern int prevDay;
extern int prevHour; // used during debug to send noon message every hour
extern int prevDayDepthSensor;
extern int invalid;
extern int depthSensorInUse;
extern float midDayDepth;
extern float theta1;
extern float theta2;
extern float theta3;
extern float omega2;
extern float omega3;
extern float alpha;
extern double timeStep;
extern int prevTimer2;


// Global variables related to EEProm locations
extern int EELeakRateLongCurrent;
extern int EELongestPrimeCurrent;
extern int EELeakRateLong;
extern int EELongestPrime;
extern int EEVolume02; //Midnight to 2AM
extern int EEVolume1214; //noon to 2PM
extern int EEVolume2224; //10PM to Midnight
extern int EEVolumeNew02; //Midnight to 2AM of the day when report will be sent
extern int RestartStatus;  // This is a 0 if the system has been run and garbage just after programming
extern int NumMsgInQueue; //This is the number of noon messages waiting to be sent

extern int DailyReportEEPromStart; // this is the EEPROM slot that Daily Report Messages will begin to be saved

extern int DiagSystemWentToSleep; // This will be set to 1 if the system went to sleep
extern int DiagCauseOfSystemReset; // This is a number indicating why the system reset itself (need better comment to desribe the options)
extern int EEpromDiagStatus; // 1 means report hourly to diagnostic phone number, 0 = don't report
extern int EEpromCountryCode;
extern int EEpromMainphoneNumber;// also needs 107
extern int EEpromDebugphoneNumber; //also needs 109
extern int EEpromCodeRevisionNumber;




// ****************************************************************************
// *** Global Variables *******************************************************
// ****************************************************************************
extern float codeRevisionNumber;

extern int aveArray[];
extern int angleArray[];

extern char active_volume_bin;

extern float longestPrime; // total upstroke for the longest priming event of the day
extern char MainphoneNumber[]; // Upside Wireless
extern char DebugphoneNumber[]; // Number for the Black Phone
//extern char* phoneNumber; // Number Used to send text message report (daily or hourly)
///extern char phoneNumber[]; // Number Used to send text message report (daily or hourly)

    //****************Hourly Diagnostic Message Variables************************
extern int tech_at_pump; // 1 if debug switch is on
extern float sleepHrStatus; // 1 if we slept during the current hour, else 0
extern float timeSinceLastRestart; // Total time in hours since last restart
extern int diagnostic_msg_sent; // set to 1 when the hourly diagnostic message is sent 
extern float diagnostic; //set to 1 when we want the diagnostic text messages to be sent hourly
extern int internalHour; // Hour of the day according to internal RTCC
extern int internalMinute; // Minute of the hour according to the internal RTCC
extern float debugDiagnosticCounter;  // DEBUG used as a variable for various things while debugging diagnostic message
extern float extRtccTalked; // set to 1 if the external RTCC talked during the last hour and didn't time out every time
extern float numberTries; // number of times that tried to connect to network for hourly diagnostic messages 
extern float extRTCCset; // To keep track if the VTCC time was used to set the external RTCC
extern float resetCause; //0 if no reset occurred, else the RCON register bit number that is set is returned
extern int extRtccHourSet; //set to 0 if the external RTCC didn't update the hour in the current loop - used to check internal RTCC

//*******************VTCC Variables************************
extern int secondVTCC; //this needs to be an int to handle the time updates during sleep mode
extern char minuteVTCC;
extern char hourVTCC;
extern char dateVTCC;
extern char monthVTCC;

extern char active_volume_bin;
extern int noon_msg_sent;  //set to 1 when noon message has been sent
extern int hour_msg_sent;  //set to 1 when hourly message has been sent
extern float longestPrime; // total upstroke for the longest priming event of the day

extern float leakRateLong; // largest leak rate recorded for the day
//extern float batteryFloat; // batteryLevel measured by scaled A/D
extern float BatteryLevelArray[3]; //Used to keep track of the rate of change of the battery voltage
extern float volume02; // Total Volume extracted from 0:00-2:00
extern float volume24;
extern float volume46;
extern float volume68;
extern float volume810;
extern float volume1012;
extern float volume1214;
extern float volume1416;
extern float volume1618;
extern float volume1820;
extern float volume2022;
extern float volume2224;
extern float EEFloatData;
//Pin assignments
extern int mclrPin;
extern int depthSensorPin;
extern int depthSensorOnOffPin;
extern int simVioPin;
extern int Pin4;
extern int Pin5;
extern int rxPin;
extern int Pin7;
extern int GND2Pin;
extern int Pin9;
extern int Pin10;
extern int batteryLevelPin;
extern int Pin12;
extern int vccPin;
extern int waterPresenceSensorPin;
extern int pwrKeyPin;
extern int txPin;
extern int sclI2CPin;
extern int sdaI2CPin;
extern int statusPin;
extern int vCapPin;
extern int picKit4Pin;
extern int picKit5Pin;
extern int yAxisAccelerometerPin;
extern int xAxisAccelerometerPin;
extern int netLightPin;
extern int waterPresenceSensorOnOffPin;
extern int GNDPin;
extern int vcc2Pin;
// other global variables
extern float debugCounter; // DEBUG DEBUG DEBUG DEBUG DEBUG
extern int hour; // Hour of the day
extern int min; // Minute of the day
extern int TimeSinceLastHourCheck; //we check this when we have gone around the no pumping loop enough times that 1 minute has gone by
extern int TimeSinceLastBatteryCheck; // we check the battery when we are sleeping because of low battery every 10 times we wake up.
extern int minute;  //minute of the day
extern int year;
extern int month;
extern int day;
extern char active_volume_bin;
extern char never_primed;  //set to 1 if we exit the priming loop because of timeout
extern char print_debug_messages; //set to 1 when we want the debug messages to be sent to the Tx pin.
extern float date;

// ****************************************************************************
// *** Function Prototypes ****************************************************
// ****************************************************************************

void initialization(void);
void ClearWatchDogTimer(void);  // some user groups say using just ClrWdt() is 
//                                 an assembly command that will cause the Compiler 
//                                 not to optimize any function, like Main, that 
//                                 it is a part of and so suggest this wrapper
float checkResetStatus(void);
int longLength(long num);
void longToString(long num, char *numString);
int stringLength(char *string);
void concat(char *dest, const char *src);
void floatToString(float myValue, char *myString);
int readWaterSensor(void);
float readDepthSensor(void); //Only used by Midday depth read which is not used
void initAdc(void);
int readAdc(int channel);
float getHandleAngle();
float batteryLevel(void);
int HasTheHandleMoved(float rest_position);

float degToRad(float degrees);
void delayMs(int ms);
int getLowerBCDAsDecimal(int bcd);
int getUpperBCDAsDecimal(int bcd);
void setInternalRTCC(int sec, int min, int hr, int wkday, int date, int month, int year);
int getTimeHour(void);
long timeStamp(void);
void ResetMsgVariables();
int translate(char digit); //Not used
void RTCCSet(void); //Never Used
void VerifyProperTimeSource(void); // Uses RTCC if it is working VTCC if it is not
int getMinuteOffset();
char BcdToDec(char val);
char DecToBcd(char val);



int noonMessage(void);

int diagnosticMessage(void);
void SoftwareReset(void);

void delaySCL(void);
void midDayDepthRead(void);


void EEProm_Write_Int(int addr, int newData);
int EEProm_Read_Int(int addr);
void EEProm_Read_Float(unsigned int ee_addr, void *obj_p);
void EEProm_Write_Float(unsigned int ee_addr, void *obj_p);
void SaveVolumeToEEProm(void);
void DebugReadEEProm(void);
void ClearEEProm(void);
void initializeVTCC(char sec, char min, char hr, char date, char month);
void updateVTCC(void);
#endif	/* IWPUTILITIES_H */
