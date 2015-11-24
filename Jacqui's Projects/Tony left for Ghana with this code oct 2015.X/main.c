/*
 * File: InternalClock.c
 * Author: jy1189
 *
 * Created on April 23, 2015, 11:05 AM
 */

//*****************************************************************************
#include "IWPUtilities.h"
#include "I2C.h"
#include "Pin_Manager.h"
#include <stdio.h>
#include <stdlib.h>
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




// ****************************************************************************
// *** Main Function **********************************************************
// ****************************************************************************
void main(void)
{
	int beenToHandleMovement = 0;
	int beenToVolumeLoop = 0;
	int beenToPrimeLoop = 0;
	int beenToLeakLoop = 0;
	initialization();

	waterPrimeTimeOut /= upstrokeInterval;
	leakRateTimeOut /= upstrokeInterval;
	//timeBetweenUpstrokes /= upstrokeInterval;

	// Do all of these values need to be reset each time around the loop? Or at the end of the day? 06-16-2014
	int handleMovement = 0; // Either 1 or no 0 if the handle moving upward
	int timeOutStatus = 0; // Used to keep track of the water prime timeout
	int hour = 0; // Hour of day
	float angleCurrent = 0; // Stores the current angle of the pump handle
	float anglePrevious = 0; // Stores the last recoreded angle of the pump handle
	float angleDelta = 0; // Stores the difference between the current and previous angles
	float upStroke = 0; // 0 if there is no upstroke, otherwise stores the delta angle
	float upStrokePrime = 0; // Stores the sum of the upstrokes for calculating the prime
	float upStrokeExtract = 0; // Stores the sum of the upstrokes for calculating volume
	float volumeEvent = 0; // Stores the volume extracted
	float leakRatePrevious = 0; // Stores the previous Leak Rate incase if someone stats to pump before leakage can be measured
	//float extractionStartTime = 0; // The time of day (in seconds) when the extraction started
	//float extractionEndTime = 0; // The time of day (in seconds) when the extraction ended
	//float extractionDuration = 0; // The difference between the extraction start and end times
	long leakTimeCounter = 0; // Used to keep track of the leak time timeout
	float upStrokePrimeMeters = 0; // Stores the upstroke in meters
	float leakRate = 0; // Rate at which water is leaking from the rising main
	//	float leakTime = 0; // The number of milliseconds from when the user stops pumping until there is no water (min: 0, max: 10 minutes)
	//	long upStrokeDelayCounter = 0;
	//        int currentHour;
	int currentDay;
	int currentHourDepthSensor;

	float deltaAverage;

	while (1)
	{ //MAIN LOOP; repeats indefinitely
		////////////////////////////////////////////////////////////
		// Idle Handle Monitor Loop
		// 2 axis of the accelerometer are constantly polled for
		// motion in the upward direction by comparing angles
		////////////////////////////////////////////////////////////
		// Get the angle of the pump handle to measure against
		anglePrevious = getHandleAngle();
		float previousAverage = 0;
		// Set the handle movement to 0 (handle is not moving)
		handleMovement = 0;
		// Loop until the handle starts moving
		float angleAccumulated=0;
		while (handleMovement == 0)
		{
                    if(beenToHandleMovement = 0){
                        beenToHandleMovement = 1;
                    }
			currentDay = getDateI2C();
			if ( prevDay != currentDay){ //(prevDay != getDateI2C()){// it's a new day so send midNightMessage();
				batteryFloat = batteryLevel();
				midnightMessage();
			}
			if (depthSensorInUse == 1){ // if the Depth sensor is present
				delayMs(1000);
				int currentDayDepthSensor = BcdToDec(getDateI2C());
				delayMs(1000);
				if ((BcdToDec(getHourI2C() == 12) && (prevDayDepthSensor != currentDayDepthSensor)));
				midDayDepthRead();
			}

			delayMs(10); // Delay for a short time
			float newAngle = getHandleAngle();
			float deltaAngle = newAngle - anglePrevious;

			if(deltaAngle < 0) {
				deltaAngle *= -1;
			}

			anglePrevious = newAngle;
			if (deltaAngle > .5){ // prevents floating accelerometer values when it's not actually moving
				angleAccumulated += deltaAngle;
			}
			// If the angle has changed, set the handleMovement flag
			if (angleAccumulated > 5) //05-30-14 Test for small delta's used to be angleDeltaThreshold
			{
				handleMovement = 1;
			}
		}
		/////////////////////////////////////////////////////////
		// Priming Loop
		// The total amount of upstroke is recorded while the
		// upper water sensor is checked to determine if the
		// pump has been primed
		/////////////////////////////////////////////////////////
		timeOutStatus = 0; // prepares timeoutstatus for new event
		// Get the angle of the pump handle to measure against
		anglePrevious = getHandleAngle();
		upStrokePrime = 0; // gets the variable ready for a new event
		upStroke = 0; // gets variable ready for new event

		// averaging angle Code 9/17/2015
		//initializeQueue(anglePrevious);
		//previousAverage = queueAverage();
		// Averaging angle code
		while ((timeOutStatus < waterPrimeTimeOut) && !readWaterSensor())
		{
                    if(beenToPrimeLoop = 0){
                        beenToPrimeLoop = 1;
                    }

			delayMs(upstrokeInterval);  // delay a short time (10ms)

			// averaging angle Code 9/17/2015
			//pushToQueue(getHandleAngle()); //get Current angle of the pump handle
			//deltaAverage = queueAverage() - previousAverage;
			//previousAverage = queueAverage();


			// end averaging angle Code

			angleCurrent = getHandleAngle(); // Get the current angle of the pump handle
			angleDelta = angleCurrent - anglePrevious; // Calculate the change in angle of the pump handle
			if(angleDelta > 1){
				//if (deltaAverage > 5){ // averaging angle code 9/17/2015
				// upStroke += deltaAverage; // angle Code 9/17/2015
				upStroke += angleDelta;
				upStrokePrime += degToRad(upStroke); // Update the upStrokePrime
				timeOutStatus=0;
				// upstroke and current angle
			}
			else{
				timeOutStatus++;}
			anglePrevious = angleCurrent; // Update the previous angle for the next calculation
		}
		upStrokePrimeMeters = upStrokePrime * upstrokeToMeters;	// Convert to meters

		if (upStrokePrimeMeters > longestPrime) // Updates the longestPrime
		{
			longestPrime = upStrokePrimeMeters;
		}
		///////////////////////////////////////////////////////
// Volume Calculation loop
// Tracks the upStroke for the water being extracted
//(in next loop -->) as well as the time in milliseconds taken for water to leak
///////////////////////////////////////////////////////
                float absoluteAngleThreshold = 1;
                int volumeLoopCounter = 5; // 50ms

                unsigned long extractionDuration = 0;
                float absoluteAngle = 0;
                float totalAbsoluteAngle = absoluteAngleThreshold + 1;
                while(readWaterSensor() && (totalAbsoluteAngle > absoluteAngleThreshold)){

                    totalAbsoluteAngle = 0;

                    int i = 0;
                    while((i < volumeLoopCounter) && readWaterSensor()){
                        angleCurrent = getHandleAngle();
                        angleDelta = angleCurrent - anglePrevious;
                        absoluteAngle=0;
                        if (angleDelta < 0){ // absolute value of angleDelta
                            absoluteAngle+= angleDelta * -1;
                        }
                        else
                        {
                            absoluteAngle += angleDelta;
                        }
                        anglePrevious = angleCurrent;
                        char volumeMessage[20];
                        volumeMessage[0] = 0;
                        if(angleDelta > 0){
                            upStrokeExtract = upStrokeExtract + angleDelta;
                            floatToString(upStrokeExtract, volumeMessage);

                        }
                            totalAbsoluteAngle += absoluteAngle;
                            char totalAbsoluteAngleMessage[20];
                            totalAbsoluteAngleMessage[0]=0;
                            longToString(angleDelta, totalAbsoluteAngleMessage);


                            i++;
                           delayMs(volumeDelay); // Delay for a short time
                           extractionDuration++;
                    }
                }
                    totalAbsoluteAngle = 0;

		///////////////////////////////////////////////////////
		// Leakage Rate loop
		///////////////////////////////////////////////////////
		// Get the angle of the pump handle to measure against
		int leakCondition;
		anglePrevious = getHandleAngle();
		// Used to keep track of how many milliseconds have passed
		long leakDurationCounter = volumeLoopCounter * volumeDelay; // The volume loop has 50 milliseconds of delay before entry
		while (readWaterSensor() && (leakTimeCounter < leakRateTimeOut))
		{
                    if(beenToLeakLoop = 0){
                        beenToLeakLoop = 1;
                    }

			angleCurrent = getHandleAngle(); //Get the current angle of the pump handle
			//Calculate the change in angle of the pump handle
			angleDelta = angleCurrent - anglePrevious;
			// Update the previous angle for the next calculation
			anglePrevious = angleCurrent;
			// If the handle moved more than 2 degrees, we will consider that an
			// intentional pump and break out of the loop (2 is in radians)
			if (angleDelta > angleDeltaThreshold)
			{
				leakCondition=1;
				break;
			}
			if (leakDurationCounter > 20000) // change to 20,000 for real code (was 100 - 10/8/2015 KK)
			{
				leakCondition=2;
				break;
			}
			leakCondition=3;
			delayMs(1);
			leakDurationCounter++;
		}
		switch (leakCondition){
		case 1:
			leakRate = leakRatePrevious;
			break;
		case 2:
			leakRate = 0;
			leakRatePrevious = leakRate;
			break;
		case 3:
			leakRate = leakSensorVolume / ((leakDurationCounter) / 1000.0); // liters/sec
			leakRatePrevious = leakRate;
		}

		if ((leakRate * 3600) > leakRateLong)
		{
			leakRateLong = leakRate * 3600; //reports in L/hr
		}

		upStrokeExtract = degToRad(upStrokeExtract);
		volumeEvent = (MKII * upStrokeExtract);
		volumeEvent -= (leakRate * extractionDuration / 10.0); // Why is this 10?

		hour = BcdToDec(getHourI2C());
		switch (hour / 2)
		{ //organize extaction into 2 hours bins
		case 0:
			volume02 = volume02 + volumeEvent;
			break;
		case 1:
			volume24 = volume24 + volumeEvent;
			break;
		case 2:
			volume46 = volume46 + volumeEvent;
			break;
		case 3:
			volume68 = volume68 + volumeEvent;
			break;
		case 4:
			volume810 = volume810 + volumeEvent;
			break;
		case 5:
			volume1012 = volume1012 + volumeEvent;
			break;
		case 6:
			volume1214 = volume1214 + volumeEvent;
			break;
		case 7:
			volume1416 = volume1416 + volumeEvent;
			break;
		case 8:
			volume1618 = volume1618 + volumeEvent;
			break;
		case 9:
			volume1820 = volume1820 + volumeEvent;
			break;
		case 10:
			volume2022 = volume2022 + volumeEvent;
			break;
		case 11:
			volume2224 = volume2224 + volumeEvent;
			break;
		}
	} // End of main loop
}