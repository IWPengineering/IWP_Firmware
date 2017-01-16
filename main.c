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
#pragma config POSCFREQ = HS // Primary Oscillator Frequency Range Configuration bits (Primary oscillator/external
			     //clock input frequency greater than 8MHz)
#pragma config SOSCSEL = SOSCHP // SOSC Power Selection Configuration bits (Secondary Oscillator configured for high-power operation)
#pragma config FCKSM = CSDCMD // Clock Switching and Monitor Selection (Both Clock Switching and Fail-safe Clock Monitor are disabled)

// FWDT
// Enable the WDT to work both when asleep and awake.  Set the time out to a nominal 131 seconds
#pragma config WDTPS = PS32768          // Watchdog Timer Postscale Select bits (1:32768)
#pragma config FWPSA = PR128            // WDT Prescaler bit (WDT prescaler ratio of 1:128)
#pragma config FWDTEN = ON              // Watchdog Timer Enable bits (WDT enabled in hardware)
// use this if you want to be able to turn WDT on and off 
// #pragma config FWDTEN = SWON            // Watchdog Timer Enable bits (WDT controlled with the SWDTEN bit setting)
#pragma config WINDIS = OFF             // Windowed Watchdog Timer Disable bit (Standard WDT selected(windowed WDT disabled))

// FPOR
#pragma config BOREN = BOR3 // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware, SBOREN bit disabled)
#pragma config LVRCFG = OFF // (Low Voltage regulator is not available)
#pragma config PWRTEN = ON // Power-up Timer Enable bit (PWRT enabled)
#pragma config I2C1SEL = PRI // Alternate I2C1 Pin Mapping bit (Use Default SCL1/SDA1 Pins For I2C1)
//#pragma config BORV = V20 // Brown-out Reset Voltage bits (Brown-out Reset set to lowest voltage (2.0V))
#pragma config MCLRE = ON // MCLR Pin Enable bit (RA5 input pin disabled,MCLR pin enabled)
// FICD
#pragma config ICS = PGx2 // ICD Pin Placement Select bits (EMUC/EMUD share PGC2/PGD2)
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
    
    //                    DEBUG
    // print_debug_messages controls the debug reporting
    //   0 = send message only at noon to upside Wireless
    //   1 = print debug messages to Tx but send only noon message to Upside
    //   2 = print debug messages, send hour message after power up and every hour
    //       to debug phone number.  Still sends noon message to Upside
    
    //   Note: selecting 1 or 2 will change some system timing since it takes 
    //         time to form and send a serial message
    print_debug_messages = 0;
    
    //                     DEBUG
    if(print_debug_messages >= 2){
          hourMessage();
    // DEBUG  
    }
  
    
 
    sendDebugMessage("   \n JUST CAME OUT OF INITIALIZATION ", 0);  //Debug
    while (1)
	{          
        batteryFloat = batteryLevel();
        if (digitalPinStatus(statusPin) == 0) { // if the Fona is off, turn it on so it is awake to be topped off
           turnOnSIM();
        }
 
        //MAIN LOOP; repeats indefinitely
		////////////////////////////////////////////////////////////
		// Idle Handle Monitor Loop
		// 2 axis of the accelerometer are constantly polled for
		// motion in the upward direction by comparing angles
		////////////////////////////////////////////////////////////

		anglePrevious = getHandleAngle();                            // Get the angle of the pump handle to measure against.  
                                                                     // This is our reference when looking for sufficient movement to say the handle is actually moving.  
                                                                     // "moving" is hard coded as 5 degrees.  We should make it a CONSTANT defined in IWPUtilities
		float previousAverage = 0;
		handleMovement = 0;                                          // Set the handle movement to 0 (handle is not moving)
		while (handleMovement == 0)
		{ 
            ClearWatchDogTimer();     // We stay in this loop if no one is pumping so we need to clear the WDT  
            hour = BcdToDec(getHourI2C());

            // should we be asleep to save power?   
            while((batteryFloat < 3.0)&&((hour < 5)||(hour > 19))){
            // If the battery level is too low to work without the sun, go to sleep
            // shut down at an ODD time so that the volume pumped from 4-6PM will be saved to EEProm
            // try to wake up at 5AM just because there is a lot of activity at this time 
            // The WDT settings will put the PIC to sleep for about 131 seconds.  
                if (digitalPinStatus(statusPin) == 1) { // if the Fona is on, shut it off
                    turnOffSIM();             
                }
                Sleep(); 
                sendDebugMessage("Going to sleep ", hour);  //Debug
                
                hour = BcdToDec(getHourI2C()); //still time to sleep? Don't check battery, you are here because it was low
            }
            // Is it time to record volume from previous time bin to EEProm?
            if(hour/2 != active_volume_bin){
                SaveVolumeToEEProm();
                sendDebugMessage("Saving volume to last active bin ", active_volume_bin - 1);  //Debug
            }
            // Is it time to report data from yesterday?        
            if((hour == 12)&&(noon_msg_sent == 0)){             //it's noon send previous day's information 0AM - 12PM
                batteryFloat = batteryLevel();
                noonMessage();
                noon_msg_sent = 1;                             // only want to send it once
                sendDebugMessage("Noon Message sent ", hour);  //Debug
            }
            if(hour != 12){
                noon_msg_sent = 0;
            }
            // If we are debugging at a pump we want to send the noon message every hour
            if((print_debug_messages >= 2)&&(hour > prevHour)){             //it's the next hour and we are debugging at pump
                //phoneNumber = DebugphoneNumber;                            // change phone number to the debug number

                batteryFloat = batteryLevel();
                hourMessage();
                prevHour = hour;                             // only want to send it once
                //phoneNumber = MainphoneNumber;                            // change phone number back to main number

                sendDebugMessage("HOURLY Message sent ", hour);  //Debug
            }
            
            // OK, go ahead and look for handle movement again
			delayMs(upstrokeInterval);                            // Delay for a short time
			float newAngle = getHandleAngle();
			float deltaAngle = newAngle - anglePrevious;
			if(deltaAngle < 0) {
				deltaAngle *= -1;
			}
            
            if(deltaAngle > 5){                         // The needed 5 degree motion should be made a constant in IWPUtilities
				handleMovement = 1;
			}
		}

		/////////////////////////////////////////////////////////
		// Priming Loop
		// The total amount of upstroke is recorded while the
		// upper water sensor is checked to determine if the
		// pump has been primed
		/////////////////////////////////////////////////////////
        sendDebugMessage("\n\n We are in the Priming Loop ", 0);  //Debug
        int i = 0; 
		timeOutStatus = 0;                                            // prepares timeoutstatus for new event
		anglePrevious = getHandleAngle();                             // Get the angle of the pump handle to measure against
		upStrokePrime = 0;
        never_primed = 0;
     
		upStroke = 0;                                                 // gets variable ready for new event
        digitalPinSet(waterPresenceSensorOnOffPin, 1); //turns on the water presence sensor.
		while ((timeOutStatus < waterPrimeTimeOut) && !readWaterSensor())
		{
            angleCurrent = getHandleAngle();                      //gets the latest 10-average angle
			angleDelta = angleCurrent - anglePrevious;            //determines the amount of handle movement from last reading
			anglePrevious = angleCurrent;                         //Prepares anglePrevious for the next loop
			if(angleDelta > 0){                                   //Determines direction of handle movement
				upStrokePrime += angleDelta;                  //If the valve is moving upward, the movement is added to an
                                                             //accumlation var (even if it was smaller than angleThresholdSmall)
			}
            // If they have stopped, pumping we should give up too
			if((angleDelta > (-1 * angleThresholdSmall)) && (angleDelta < angleThresholdSmall)){   //Determines if the handle is at rest
                i++; // we want to stop if the user stops pumping              
			}
			else{
                i=0;   // they are still trying
			} 
            if(i == 100){  // They quit trying for at least 1 second
                never_primed = 1;
                sendDebugMessage("        Stopped trying to prime   ", upStrokePrime);  //Debug
                break;
            }
            timeOutStatus++; // we will wait for up to waterPrimeTimeOut of pumping
			delayMs(upstrokeInterval); 
        }
        if(timeOutStatus >= waterPrimeTimeOut){
            never_primed = 1;          
        }
        sendDebugMessage("The time spent trying to prime = ",timeOutStatus*10);
		upStrokePrimeMeters = upStrokePrime * upstrokeToMeters;	      // Convert to meters
        sendDebugMessage("Up Stroke Prime = ", upStrokePrimeMeters);  //Debug
        sendDebugMessage(" - Longest Prime = ", longestPrime);  //Debug
        sendDebugMessage(" never primed status = ", never_primed);  //Debug
		if (upStrokePrimeMeters > longestPrime){                      // Updates the longestPrime
			longestPrime = upStrokePrimeMeters;
            EEProm_Write_Float(1,&longestPrime);                      // Save to EEProm
            sendDebugMessage("We saved new Up Stroke Prime to EEProm ", 1);  //Debug
		}
		///////////////////////////////////////////////////////
		// Volume Calculation loop
		// Tracks the upStroke for the water being extracted
		//(in next loop -->) as well as the time in milliseconds taken for water to leak
		///////////////////////////////////////////////////////
        sendDebugMessage("\n We are in the Volume Loop ", 0);  //Debug
		int volumeLoopCounter = 15; // 150 ms                           //number of zero movement cycles before loop ends
		unsigned long extractionDurationCounter = 0;                           //keeps track of pumping duration
		i = 0;                                                      //Index to keep track of no movement cycles
		while(readWaterSensor() && (i < volumeLoopCounter)){            //if the pump is primed and the handle has not been 
							                                            //still for "volumeLoopCounter loops
            ClearWatchDogTimer();     // Is unlikely that we will be pumping for 130sec without a stop, but we might
            angleCurrent = getHandleAngle();                        //gets the latest 10-average angle
			angleDelta = angleCurrent - anglePrevious;              //determines the amount of handle movement from last reading
			anglePrevious = angleCurrent;                           //Prepares anglePrevious for the next loop
			if(angleDelta > 0){                                     //Determines direction of handle movement
				upStrokeExtract += angleDelta;                  //If the valve is moving upward, the movement is added to an
										//accumlation var
			}
			if((angleDelta > (-1 * angleThresholdSmall)) && (angleDelta < angleThresholdSmall)){   //Determines if the handle is at rest
				i++;
			}
			else{
				i = 0;
			}                                                             //Reset i if handle is moving
			extractionDurationCounter++;                                         // Keep track of elapsed time for leakage calc
			delayMs(upstrokeInterval);                                         // Delay for a short time
		}
		///////////////////////////////////////////////////////
		// Leakage Rate loop
		///////////////////////////////////////////////////////
        sendDebugMessage("\n We are in the Leak Rate Loop ", 0);  //Debug
		// Get the angle of the pump handle to measure against
		int leakCondition = 3;  // Assume that we are going to be able to calculate a valid leak rate
      
        if(!readWaterSensor()){  // If there is already no water when we get here, something strange is happening, don't calculate leak
            leakCondition = 4;
             sendDebugMessage("There is no water as soon as we get here ", 0);  //Debug
        }
        if(never_primed == 1){
            leakCondition = 4;   // there was never any water
            sendDebugMessage("There never was any water ", 0);  //Debug
        }
        i = 0;  
        anglePrevious = getHandleAngle();                                       // Keep track of how many milliseconds have passed
		long leakDurationCounter = volumeLoopCounter;                            // The volume loop has 150 milliseconds of delay 
                                                                                // if no water or no handle movement before entry.
        while (readWaterSensor()){
			angleCurrent = getHandleAngle();                                //Get the current angle of the pump handle
			angleDelta = angleCurrent - anglePrevious;                      //Calculate the change in angle of the pump handle
            anglePrevious = angleCurrent;                                   // Update the previous angle for the next calculation
											                                                               // intentional pump and break out of the loop (2 is in radians)
			// If the handle starts moving we will abandon calculating a new leak rate
            //  Moving is the same criterion as stopping in volume calculation loop
            if((angleDelta > (-1 * angleThresholdSmall)) && (angleDelta < angleThresholdSmall)){   //Determines if the handle is at rest
				i=0;
            }
			else{
				i++;
 			}             
            if (i >= volumeLoopCounter){
				leakCondition = 1;
				break;
			}
			if (leakDurationCounter >= leakRateTimeOut){                              // (was 100 - 10/8/2015 KK)
				leakCondition = 2; //Jump to condition for no leak if while is broken out of on condition of
                                    //exceeding the leakRateTimeOut wait.
				break;
			}
			delayMs(upstrokeInterval);
			leakDurationCounter++;
		}
        digitalPinSet(waterPresenceSensorOnOffPin, 0); //turns off the water presence sensor.
        
        sendDebugMessage("The Leak condition is ", leakCondition);  //Debug
		switch (leakCondition){
		case 1:
			leakRate = leakRatePrevious; // They started pumping again so can't calculate a new leak rate, use the last one when calculating volume pumped
			break;
		case 2:                          // Waited the max time and water was still there so leak rate = 0
			leakRate = 0;
			leakRatePrevious = leakRate;  
			break;
		case 3:                         // The pump did prime but water leaked out in less than our max time to wait.  So calculate a new value
			leakRate = leakSensorVolume / ((leakDurationCounter * upstrokeInterval) / 1000.0); // liters/sec
			leakRatePrevious = leakRate;    
            break;           
		
        case 4:
            leakRate = leakRatePrevious;  // there was no water at the start of this so we can't calculate a new leak
//            leakRate = 0;  // there was never any water so we can't calculate a new leak rate, let previous value stay the previous value
            break;
        }
        sendDebugMessage("Leak Rate = ", leakRate * 3600);  //Debug
        sendDebugMessage("  - leak Rate Long = ", leakRateLong);  //Debug
		if ((leakRate * 3600) > leakRateLong)
		{
			leakRateLong = leakRate * 3600;                                              //reports in L/hr
            EEProm_Write_Float(0,&leakRateLong);                                        // Save to EEProm
            sendDebugMessage("Saved new longest leak rate to EEProm ", leakRateLong);  //Debug
		}
		upStrokeExtract = degToRad(upStrokeExtract);
		volumeEvent = (MKII * upStrokeExtract);     //[L/rad][rad]=[L] 
		volumeEvent -= (leakRate * ((extractionDurationCounter * upstrokeInterval) / 1000.0)); //[L/s][s]=[L]
        if(volumeEvent < 0)
        {
            volumeEvent = 0; // we can't pump negative volume
        }
		// organize flow into 2 hours bins
        // The hour was read at the start of the handle movement routine       
        sendDebugMessage("Volume Event = ", volumeEvent);  //Debug
        sendDebugMessage("  for time slot ", hour);  //Debug

       
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

