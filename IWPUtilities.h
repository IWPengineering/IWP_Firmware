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
extern const float leakSensorVolume; // This is in Liters; pipe dia. = 33mm; rod diam 12 mm; gage length 24mm
extern const int alarmHour; // The weekday and hour (24 hour format) (in BCD) that the alarm will go off
extern const int alarmStartingMinute; // The minimum minute that the alarm will go off
extern const int alarmMinuteMax; // The max number of minutes to offset the alarm (the alarmStartingMinute + a random number between 0 and this number)
extern const int signedNumAdjustADC; // Used to divide the total range of the output of the 10 bit ADC into positive and negative range.
extern const int pulseWidthThreshold; // The value to check the pulse width against (2048)
extern const int networkPulseWidthThreshold; // The value to check the pulse width against (about 20000)
extern const int upstrokeInterval; // The number of milliseconds to delay before reading the upstroke
extern int waterPrimeTimeOut; // Equivalent to 7 seconds (in 50 millisecond intervals); 50 = upstrokeInterval
extern long leakRateTimeOut; // Equivalent to 18 seconds (in 50 millisecond intervals); 50 = upstrokeInterval
//extern long timeBetweenUpstrokes; // 3 seconds (based on upstrokeInterval)
extern const int decimalAccuracy; // Number of decimal places to use when converting floats to strings
extern const float angleThresholdSmall; //number close to zero to determine if handle is moving w/o interpreting accelerometer noise as movement.
extern const float angleThresholdLarge; //total angle movement to accumulate before identifying movement as intentional pumping
extern const float PI;
extern const float upstrokeToMeters;
extern const int minimumAngleDelta;
extern const float batteryLevelConstant;       //This number is found by Vout = (R32 * Vin) / (R32 + R31), Yields Vin = Vout / 0.476
extern int queueCount;
extern int queueLength; //don't forget to change angleQueue to this number also
extern float angleQueue[7];
extern int prevDay;
//extern int prevHour;
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


// ****************************************************************************
// *** Global Variables *******************************************************
// ****************************************************************************
//static char phoneNumber[] = "+233247398396"; // Number for the Black Phone
extern char phoneNumber[]; // Number for Upside Wireless
extern char phoneNumber2[]; // Tony's number
extern char active_volume_bin;
extern char noon_msg_sent;  //set to 1 when noon message has been sent
extern float longestPrime; // total upstroke fo the longest priming event of the day
extern float leakRateLong; // largest leak rate recorded for the day
extern float batteryFloat; // batteryLevel before sends text message commences
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
extern int debugCounter; // DEBUG DEBUG DEBUG DEBUG DEBUG
extern char active_volume_bin;



void initialization(void);
void ClearWatchDogTimer(void);  // some user groups say using just ClrWdt() is 
//                                 an assembly command that will cause the Compiler 
//                                 not to optimize any function, like Main, that 
//                                 it is a part of and so suggest this wrapper
int longLength(long num);
void longToString(long num, char *numString);
int stringLength(char *string);
void concat(char *dest, const char *src);
void floatToString(float myValue, char *myString);
void turnOffSIM();
void turnOnSIM();
void tryToConnectToNetwork();
int connectedToNetwork(void);
void sendDebugMessage(char message[50], float value);
void sendMessage(char message[160]);
void sendTextMessage(char message[160]);
int readWaterSensor(void);
void initAdc(void);
int readAdc(int channel);
float getHandleAngle();
void initializeQueue(float value);
void pushToQueue(float value);
float queueAverage();
float queueDifference();
float batteryLevel(void);

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
void midnightMessage(void);
void noonMessage(void);
void SoftwareReset(void);
void delaySCL(void);
void midDayDepthRead(void);
void sendTimeMessage(void);

void EEProm_Write_Int(int addr, int newData);
int EEProm_Read_Int(int addr);
void EEProm_Read_Float(unsigned int ee_addr, void *obj_p);
void EEProm_Write_Float(unsigned int ee_addr, void *obj_p);
#endif	/* IWPUTILITIES_H */