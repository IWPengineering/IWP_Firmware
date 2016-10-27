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

// ****************************************************************************
// *** Main Function **********************************************************
// ****************************************************************************
void main(void)
{
	initialization();

	waterPrimeTimeOut /= upstrokeInterval;
	leakRateTimeOut /= upstrokeInterval;
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
	float upStrokePrimeMeters = 0; // Stores the upstroke in meters
	float leakRate = 0; // Rate at which water is leaking from the rising main
	int currentDay;
	int prevDay = getDateI2C();


	while (1)
	{ //MAIN LOOP; repeats indefinitely
		////////////////////////////////////////////////////////////
		// Idle Handle Monitor Loop
		// 2 axis of the accelerometer are constantly polled for
		// motion in the upward direction by comparing angles
		////////////////////////////////////////////////////////////

		anglePrevious = getHandleAngle();                                       // Get the angle of the pump handle to measure against
		float previousAverage = 0;
		handleMovement = 0;                                                     // Set the handle movement to 0 (handle is not moving)
		float angleAccumulated = 0;                                               // Loop until the handle starts moving or if there is water
		while (handleMovement == 0)
		{
			currentDay = getDateI2C();
			if (prevDay != currentDay){                                         // it's a new day so send midNightMessage();
				batteryFloat = batteryLevel();
				midnightMessage();
				prevDay = currentDay;
			}
			delayMs(10);                                                        // Delay for a short time
			float newAngle = getHandleAngle();
			float deltaAngle = newAngle - anglePrevious;
			if(deltaAngle < 0) {
				deltaAngle *= -1;
			}
			anglePrevious = newAngle;
			if (deltaAngle > .5){                                               // prevents floating accelerometer values when it's not actually moving
				angleAccumulated += deltaAngle;
			}

			if (angleAccumulated > 5){                                          // If the angle has changed, set the handleMovement flag
				handleMovement = 1;
			}
		}
		/////////////////////////////////////////////////////////
		// Priming Loop
		// The total amount of upstroke is recorded while the
		// upper water sensor is checked to determine if the
		// pump has been primed
		/////////////////////////////////////////////////////////

		timeOutStatus = 0;                                                      // prepares timeoutstatus for new event
		anglePrevious = getHandleAngle();                                       // Get the angle of the pump handle to measure against
		upStrokePrime = 0;
                float angleThreshold = 0.08;                                               //number close to zero to determine if handle is moving// gets the variable ready for a new event
		upStroke = 0;                                            // gets variable ready for new event
		while ((timeOutStatus < waterPrimeTimeOut) && !readWaterSensor())
		{
                        angleCurrent = getHandleAngle();                                       //gets the latest 10-average angle
			angleDelta = angleCurrent - anglePrevious;                             //determines the amount of handle movement from last reading
			anglePrevious = angleCurrent;                                          //Prepares anglePrevious for the next loop
			if(angleDelta > 0){                                                    //Determines direction of handle movement
				upStrokePrime += angleDelta;                                     //If the valve is moving upward, the movement is added to an accumlation var
			}
			if((angleDelta > (-1 * angleThreshold)) && (angleDelta < angleThreshold)){   //Determines if the handle is at rest
				timeOutStatus++;
			}
			else{
				timeOutStatus = 0;
			}                                                                      //Reset i if handle is moving
			delayMs(10); 
                 }

		upStrokePrimeMeters = upStrokePrime * upstrokeToMeters;	                // Convert to meters
		if (upStrokePrimeMeters > longestPrime){                                // Updates the longestPrime
			longestPrime = upStrokePrimeMeters;
		}
		///////////////////////////////////////////////////////
		// Volume Calculation loop
		// Tracks the upStroke for the water being extracted
		//(in next loop -->) as well as the time in milliseconds taken for water to leak
		///////////////////////////////////////////////////////

		angleThreshold = 0.08;                                               //number close to zero to determine if handle is moving
		int volumeLoopCounter = 15; // 150 ms                                      //number of zero movement cycles before loop ends
		unsigned long extractionDuration = 0;                                      //keeps track of pumping duration
		int i = 0;                                                                   //Index to keep track of no movement cycles
		while(readWaterSensor() && (i < volumeLoopCounter)){                       //if the pump is primed and the handle has not been still for five loops
			angleCurrent = getHandleAngle();                                       //gets the latest 10-average angle
			angleDelta = angleCurrent - anglePrevious;                             //determines the amount of handle movement from last reading
			anglePrevious = angleCurrent;                                          //Prepares anglePrevious for the next loop
			if(angleDelta > 0){                                                    //Determines direction of handle movement
				upStrokeExtract += angleDelta;                                     //If the valve is moving upward, the movement is added to an accumlation var
			}
			if((angleDelta > (-1 * angleThreshold)) && (angleDelta < angleThreshold)){   //Determines if the handle is at rest
				i++;
			}
			else{
				i = 0;
			}                                                                      //Reset i if handle is moving
			extractionDuration++;                                                  // Keep track of elapsed time for leakage calc
			delayMs(volumeDelay);                                                  // Delay for a short time
		}
		///////////////////////////////////////////////////////
		// Leakage Rate loop
		///////////////////////////////////////////////////////
		// Get the angle of the pump handle to measure against
		int leakCondition;
		anglePrevious = getHandleAngle();                                       // Used to keep track of how many milliseconds have passed
		long leakDurationCounter = volumeLoopCounter * volumeDelay;             // The volume loop has 50 milliseconds of delay before entry
		while (readWaterSensor() && (leakDurationCounter < leakRateTimeOut)){
			angleCurrent = getHandleAngle();                                    //Get the current angle of the pump handle
			angleDelta = angleCurrent - anglePrevious;                          //Calculate the change in angle of the pump handle
			anglePrevious = angleCurrent;                                       // Update the previous angle for the next calculation
			// If the handle moved more than 2 degrees, we will consider that an	                                                                // intentional pump and break out of the loop (2 is in radians)
			if (angleDelta > angleDeltaThreshold){
				leakCondition = 1;
				break;
			}
			if (leakDurationCounter > 200000){                                  // change to 20,000 for real code (was 100 - 10/8/2015 KK)
				leakCondition = 2;
				break;
			}
			leakCondition = 3;
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
			leakRateLong = leakRate * 3600;                                     //reports in L/hr
		}
		upStrokeExtract = degToRad(upStrokeExtract);
		volumeEvent = (MKII * upStrokeExtract);
		volumeEvent -= (leakRate * extractionDuration / volumeDelay);
        if(volumeEvent < 0)
        {
            volumeEvent = 0; // we can't pump negative volume
        }

		hour = BcdToDec(getHourI2C());                                          //organize flow into 2 hours bins
		switch (hour / 2)
		{
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
} // End of main program

