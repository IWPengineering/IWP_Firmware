/*
 * File: Main.c
 * Author: jy1189 - and many more
 * 
 * Created on April 23, 2015, 11:05 AM
 */

//*****************************************************************************
#include "IWPUtilities.h"
#include "I2C.h"
#include "FONAUtilities.h"
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
#pragma config FNOSC = FRC
//#pragma config FNOSC = SOSC // Oscillator Select (Fast RC Oscillator (FRC))
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
    leakRateTimeOut /= upstrokeInterval;
	int handleMovement = 0; // Either 1 or no 0 if the handle moving upward
    
	float angleCurrent = 0; // Stores the current angle of the pump handle
	float anglePrevious = 0; // Stores the last read angle of the pump handle
    float angleAtRest = 0; // Used to store the handle angle when it is not moving
	float angleDelta = 0; // Stores the difference between the current and previous angles
	float upStrokePrime = 0; // Stores the sum of the upstrokes for calculating the prime
	float upStrokeExtract = 0; // Stores the sum of the upstrokes for calculating volume
	float volumeEvent = 0; // Stores the volume extracted
	float leakRatePrevious = 0; // Stores the previous Leak Rate incase if someone stats to pump before leakage can be measured
	float upStrokePrimeMeters = 0; // Stores the upstroke in meters
	float leakRate = 0; // Rate at which water is leaking from the rising main
    
	
    
    //                    DEBUG
    // print_debug_messages controls the debug reporting
    //   0 = send message only at noon to upside Wireless
    //   1 = print debug messages to Tx but send only noon message to Upside (MainphoneNumber[])
    //  ? do we do this one still ? 2 = print debug messages, send hour message after power up and every hour
    //       to debug phone number.  Still sends noon message to Upside
    
    //   Note: selecting 1 or 2 will change some system timing since it takes 
    //         time to form and send a serial message
    print_debug_messages = 0;
    int temp_debug_flag = print_debug_messages;
    
    EEProm_Read_Float(EEpromCodeRevisionNumber,&codeRevisionNumber); //Get the current Diagnostic Status from EEPROM
    
    print_debug_messages = 1;                                        //// We always want to print this out
    sendDebugMessage("   \n JUST CAME OUT OF INITIALIZATION ",-0.1);  //Debug
    sendDebugMessage("The hour is = ", BcdToDec(getHourI2C()));  //Debug
    sendDebugMessage("The minute is = ", BcdToDec(getMinuteI2C()));  //Debug
    sendDebugMessage("The battery is at ",batteryLevel()); //Debug
    sendDebugMessage("The hourly diagnostic reports are at ",diagnostic); //Debug
    sendDebugMessage("The revision number of this code is ",codeRevisionNumber); //Debug
    TimeSinceLastHourCheck = 0;
    print_debug_messages = temp_debug_flag;                          // Go back to setting chosen by user
    
     while (1)
	{
       //batteryFloat = batteryLevel();
        //MAIN LOOP; repeats indefinitely
		////////////////////////////////////////////////////////////
		// Idle Handle Monitor Loop
		// 2 axis of the accelerometer are constantly polled for
		// motion in the upward direction by comparing angles
		////////////////////////////////////////////////////////////
        PORTBbits.RB0 = 0;
		angleAtRest = getHandleAngle();                            // Get the angle of the pump handle to measure against.  
                                                                     // This is our reference when looking for sufficient movement to say the handle is actually moving.  
                                                                     // the "moving" threshold is defined by handleMovementThreshold in IWPUtilities
		handleMovement = 0;                                          // Set the handle movement to 0 (handle is not moving)
		while (handleMovement == 0)
		{ 
           //Check pin connected to debug switch? To see if we have to change tech_at_pump setting.
           digitalPinSet(waterPresenceSensorOnOffPin, 1); //turn on the water presence sensor
           if(readWaterSensor() == 1) { // there is water
               PORTBbits.RB1 = 1;
            } else if(readWaterSensor() == 0){ //there is not water
               PORTBbits.RB1 = 0;
            } else {            // WPS is disconnected
               PORTBbits.RB1 ^= 1;
               delayMs(50);
            }
           
           ClearWatchDogTimer();     // We stay in this loop if no one is pumping so we need to clear the WDT 
           
           // See if the diagnostic PCB is plugged in
//           if(PORTBbits.RB1 == 0){ //The diagnostic PCB is plugged in (pin #5)
           if(tech_at_pump == 1) {
                 CheckIncommingTextMessages(); //See if there are any text messages
                                         // the SIM is powered ON at this point
           }
           else{if(FONAisON){turnOffSIM();}}
           // See if the external RTCC is keeping track of time or if we need to rely on 
           // our less accurate internal timer VTCC
            if(TimeSinceLastHourCheck == 1){ // Check every minute, updated in VTCC interrupt routine
                VerifyProperTimeSource();   
                TimeSinceLastHourCheck = 0; //this gets updated in VTCC interrupt routine
            }  
            // Do hourly tasks
           if(hour != prevHour){
                if(hour/2 != active_volume_bin){
                    SaveVolumeToEEProm();
                    sendDebugMessage("Saving volume to last active bin ", active_volume_bin - 1);
                }
                //Update the Battery level Array used in sleep decision
                BatteryLevelArray[0]=BatteryLevelArray[1];
                BatteryLevelArray[1]=BatteryLevelArray[2];
                BatteryLevelArray[2]=batteryLevel();
                //sendDebugMessage("The send debug message status is ", diagnostic);
                // Read messages sent to the system
                CheckIncommingTextMessages();  // Reads and responds to any messages sent to the system
                // The SIM is ON at this point              
                if(hour == 12){  // If it is noon, save a daily report
                    CreateAndSaveDailyReport();
                }
                // Attempt to Send daily report and if enabled, diagnostic reports
                SendSavedDailyReports();   
                SendHourlyDiagnosticReport();
                turnOffSIM();
                prevHour = hour; // update so we know this is not the start of a new hour
            }       
            // should we be asleep to save power?
           if(((BatteryLevelArray[0]-BatteryLevelArray[2])>BatteryDyingThreshold)||(BatteryLevelArray[2]<=3.2)){
              // keep sleeping until the battery charges up to where it was 2 hrs before we went to sleep
               while(BatteryLevelArray[2]<BatteryLevelArray[0]){ 
            //while(batteryFloat < 3.3){
                // The WDT settings will let the PIC sleep for about 131 seconds.  
                    if (digitalPinStatus(statusPin) == 1) { // if the Fona is on, shut it off
                        turnOffSIM();             
                    }
                    if (sleepHrStatus != 1){ // Record the fact that we went to sleep for diagnostic reporting purposes
                        sleepHrStatus = 1;
                        EEProm_Write_Float(DiagnosticEEPromStart,&sleepHrStatus); 
                    }                
                    Sleep(); 
                    // OK, we just woke up  
                    secondVTCC = secondVTCC + 131;
                    updateVTCC(); //Try to keep VTCC as accurate as possible
                    TimeSinceLastBatteryCheck++; // only check the battery every 11th time you wake up (approx 24min)
                    // check the battery every 24 min
                    if(TimeSinceLastBatteryCheck >= 11){
                        BatteryLevelArray[2]=batteryLevel();
                        TimeSinceLastBatteryCheck = 0;
                    }
                }
           }

            // OK, go ahead and look for handle movement again
           handleMovement = HasTheHandleMoved(angleAtRest);
           delayMs(upstrokeInterval);                            // Delay for a short time
		}

		/////////////////////////////////////////////////////////
		// Priming Loop
		// The total amount of upstroke is recorded while the
		// upper water sensor is checked to determine if the
		// pump has been primed
		/////////////////////////////////////////////////////////
        sendDebugMessage("\n\n We are in the Priming Loop ", -0.1);  //Debug
        //AD1CON1bits.ADON = 1; // Turn on ADC
        int stopped_pumping_index = 0; 
        const float angleThresholdSmallNegative = angleThresholdSmall * -1;
        anglePrevious = getHandleAngle();                             // Get the angle of the pump handle to measure against
        angleAtRest = getHandleAngle();                             // Get the angle of the pump handle to measure against
		upStrokePrime = 0;
        never_primed = 0;

        TMR4 = 0;
        T4CONbits.TON = 1; // Starts 16-bit Timer3
        
        digitalPinSet(waterPresenceSensorOnOffPin, 1); //turns on the water presence sensor.
       PORTBbits.RB0 = 1;   //Turn on pumping led - red
		delayMs(5); //make sure the 555 has had time to turn on
        
        // needed to time the loop for measuring volume of priming
        int loopMinutes = minuteVTCC; // keeps track of loop minutes
        float primeLoopSeconds = secondVTCC - 10; // keeps track of loop seconds, minus 10 to account for end waiting time
        float startTimer = TMR2;
        
        // needed to ensure consistent sampling frequency of 102Hz
        TMR4 = 0; // clear timer
        
        while (readWaterSensor() == 0)
		{
            ClearWatchDogTimer();     // Is unlikely that we will be priming for 130sec without a stop, but we might
            angleCurrent = getHandleAngle();                        //gets the filtered current angle
			angleDelta = angleCurrent - anglePrevious;              //determines the amount of handle movement from last reading
			anglePrevious = angleCurrent;                           //Prepares anglePrevious for the next loop
			if(angleDelta < 0) {  //Determines direction of handle movement
                upStrokePrime += (-1) * angleDelta;                  //If the valve is moving downward, the movement is added to an
										                        //accumlation var
			}
            
            // If they have stopped, pumping we should give up too
			if((angleDelta > angleThresholdSmallNegative) && (angleDelta < angleThresholdSmall)){   //Determines if the handle is at rest
                stopped_pumping_index++; // we want to stop if the user stops pumping              
			} else{
                stopped_pumping_index=0;   // they are still trying
			}
            
            if((stopped_pumping_index) > max_pause_while_pumping){  // They quit trying for at least 10 seconds
                never_primed = 1;
                sendDebugMessage("        Stopped trying to prime   ", upStrokePrime);  //Debug
                break;
            }

            
			while (TMR4 < 153); //fixes the sampling rate at about 102Hz
            TMR4 = 0; //reset the timer before reading WPS

        }
        
        if (readWaterSensor() == 2) {
            never_primed = 2;
        }
        
        primeLoopSeconds = secondVTCC - primeLoopSeconds; // get the number of seconds pumping from VTCC (in increments of 4 seconds)
        primeLoopSeconds += (TMR2 - startTimer) / 15625.0; // get the remainder of seconds (less than the 4 second increment)
        loopMinutes = minuteVTCC - loopMinutes; // get the number of minutes pumping from VTCC

        if (loopMinutes < 0) {
            loopMinutes += 60; // in case the hour incremented and you get negative minutes, make them positive
        }
        
        primeLoopSeconds += loopMinutes * 60; // add minutes to the seconds
        
	
        if(readWaterSensor() == 1) {
            PORTBbits.RB1 = 1;
        } else if(readWaterSensor() == 0){
            PORTBbits.RB1 = 0;
        } else if(readWaterSensor() == 2) {
            PORTBbits.RB1 ^= 1;
            //delayMs(50);
        }

		///////////////////////////////////////////////////////
		// Volume Calculation loop
		// Tracks the upStroke for the water being extracted
		//(in next loop -->) as well as the time in milliseconds taken for water to leak
		///////////////////////////////////////////////////////
        //sendDebugMessage("\n We are in the Volume Loop ", -0.1);  //Debug
        upStrokeExtract = 0;                                                 // gets variable ready for new volume event
		int volumeLoopCounter = 60; // 588 ms                           //number of zero movement cycles before loop ends
		unsigned long extractionDurationCounter = 0;                           //keeps track of pumping duration
		int i = 0;                                                      //Index to keep track of no movement cycles
        anglePrevious = getHandleAngle();

        // needed to time the loop for measuring volume
        loopMinutes = minuteVTCC; // keeps track of loop minutes
        float volumeLoopSeconds = secondVTCC; // keeps track of loop seconds
        startTimer = TMR2;
        
        // needed to ensure consistent sampling frequency of 102Hz
        TMR4 = 0; // clear timer
        if (never_primed != 2) {
		while((readWaterSensor() == 1)&& (i < volumeLoopCounter)){            //if the pump is primed and the handle has not been 

            ClearWatchDogTimer();     // Is unlikely that we will be pumping for 130sec without a stop, but we might

            angleCurrent = getHandleAngle();                        //gets the filtered latest angle
			angleDelta = angleCurrent - anglePrevious;              //determines the amount of handle movement from last reading
			anglePrevious = angleCurrent;                           //Prepares anglePrevious for the next loop
			if(angleDelta < 0) {  //Determines direction of handle movement
                upStrokeExtract += (-1) * angleDelta;                  //If the valve is moving downward, the movement is added to an
										                        //accumlation var
			}
             
			if((angleDelta > angleThresholdSmallNegative) && (angleDelta < angleThresholdSmall)){   //Determines if the handle is at rest
				i++;
			}
			else{
				i = 0;
			}                                                             //Reset i if handle is moving

			extractionDurationCounter++;                                         // Keep track of elapsed time for leakage calc
			
            while (TMR4 < 153); //fixes the sampling rate at about 102Hz
            TMR4 = 0; //reset the timer before reading WPS
		}
        }
        volumeLoopSeconds = secondVTCC - volumeLoopSeconds; // get the number of seconds pumping from VTCC (in increments of 4 seconds)
        volumeLoopSeconds += (TMR2 - startTimer) / 15625.0; // get the remainder of seconds (less than the 4 second increment)
        loopMinutes = minuteVTCC - loopMinutes; // get the number of minutes pumping from VTCC

        if (loopMinutes < 0) {
            loopMinutes += 60; // in case the hour incremented and you get negative minutes, make them positive
        }
        
        volumeLoopSeconds += loopMinutes * 60; // add minutes to the seconds

		///////////////////////////////////////////////////////
		// Leakage Rate loop
		///////////////////////////////////////////////////////
        sendDebugMessage("\n We are in the Leak Rate Loop ", -0.1);  //Debug
		// Get the angle of the pump handle to measure against
		int leakCondition = 3;  // Assume that we are going to be able to calculate a valid leak rate
        
        if(!(readWaterSensor() == 1)){  // If there is already no water when we get here, something strange is happening, don't calculate leak
            leakCondition = 4;
             sendDebugMessage("There is no water as soon as we get here ", -0.1);  //Debug
        }
        if(never_primed == 1 || never_primed == 2){
            leakCondition = 4;   // there was never any water
            sendDebugMessage("There never was any water ", -0.1);  //Debug
        }
        i = 0;  
        anglePrevious = getHandleAngle();                                       // Keep track of how many milliseconds have passed
		long leakDurationCounter = volumeLoopCounter;                            // The volume loop has 150 milliseconds of delay 
        angleAtRest = getHandleAngle();                                                                        // if no water or no handle movement before entry.
        //AD1CON1bits.ADON = 0; // Turn off ADC
        while ((readWaterSensor() == 1)&&(leakCondition == 3)){
            if(HasTheHandleMoved(angleAtRest)){ // if they start pumping, stop calculating leak
                leakCondition = 1;
            }
			if ((leakDurationCounter * upstrokeInterval) >= leakRateTimeOut){  
				leakCondition = 2;  // The leak is slow enough to call it zero 
			}
			delayMs(upstrokeInterval);
			leakDurationCounter++;
		}
        digitalPinSet(waterPresenceSensorOnOffPin, 0); //turns off the water presence sensor.
        
        
        if (upStrokeExtract < 900){  // If someone has not pumped at least 10 liters we don't want to measure leak rate
                                     // this is to prevent a splash from a slug of water hitting the WPS and being interpreted as leak
                                     // when it is just receeding back down the pipe when momentum goes away.
                                     //  upStrokeExtractis in degrees at this point
            leakCondition = 5;
        }
        sendDebugMessage("The Leak condition is ", leakCondition);  //Debug
		switch (leakCondition){
		case 1:
			leakRate = leakRatePrevious; // They started pumping again so can't calculate a new leak rate, use the last one when calculating volume pumped
			break;
		case 2:                          // Waited the max time and water was still there so leak rate = 0
			leakRate = 0;
			leakRatePrevious = leakRate;  
			break;
		case 3:                         // The pump did prime and water leaked out in less than our max time to wait.  So calculate a new value
			leakRate = leakSensorVolume / ((leakDurationCounter * upstrokeInterval) / 1000.0); // liters/sec
			leakRatePrevious = leakRate;    
            break;           
		
        case 4:
            leakRate = leakRatePrevious;  // there was no water at the start of this so we can't calculate a new leak
//            leakRate = 0;  // there was never any water so we can't calculate a new leak rate, let previous value stay the previous value
            break;
        
        case 5:
			leakRate = leakRatePrevious; // Not sure that the rising main was full at the end of pumping so don't calculate a new leak rate
			break;
        }
        
        ///////////////////////////////////////////////////
        /// Save leak rate and report in debug messages ///
        ///////////////////////////////////////////////////
        
        sendDebugMessage("Leak Rate = ", leakRate * 3600);  //Debug
        sendDebugMessage("  - leak Rate Long = ", leakRateLong);  //Debug
		if ((leakRate * 3600) > leakRateLong)
		{
			leakRateLong = leakRate * 3600;                                              //reports in L/hr
            EEProm_Write_Float(0,&leakRateLong);                                        // Save to EEProm
            sendDebugMessage("Saved new longest leak rate to EEProm ", leakRateLong);  //Debug
		}
        
        ///////////////////////////////////////////////////////////
        /// Save prime distance and report in debug messages    ///
        ///////////////////////////////////////////////////////////
        
        if (upStrokePrime == 0) {
            upStrokePrimeMeters = 0; // to avoid divide by zero
        } else {
            upStrokePrime = degToRad(upStrokePrime);    // convert to radians

            float timePerRad = primeLoopSeconds / upStrokePrime;

            if (timePerRad < quadVertex) { // if the time per radian is below this value, the result will be undefined
                timePerRad = quadVertex;    // if above case, set the time per radian to the minimum defined value
            }

            upStrokePrimeMeters = ((-b - sqrt((b*b) - (4 * (a) * (c - (timePerRad))))) / (2*a)) * upStrokePrime; // calculate volume based on quadratic trend line
            upStrokePrimeMeters = (upStrokePrimeMeters / 1000) / 0.000899; // convert to meters based on volume of rising main
        }

		if (upStrokePrimeMeters > longestPrime){                      // Updates the longestPrime
			longestPrime = upStrokePrimeMeters;
            EEProm_Write_Float(1,&longestPrime);                      // Save to EEProm
		}
        sendDebugMessage("Prime Distance ", upStrokePrimeMeters);  //Debug
        
        ///////////////////////////////////////////////////
        /// Save volume and report in debug messages    ///
        ///////////////////////////////////////////////////
        sendDebugMessage("handle movement in degrees ", upStrokeExtract);  //Debug
		upStrokeExtract = degToRad(upStrokeExtract);
        sendDebugMessage("handle movement in radians ", upStrokeExtract);  //Debug 
        
        if (upStrokeExtract == 0) {
            volumeEvent = 0;    // to avoid divide by zero
        } else {
            float timePerRad = volumeLoopSeconds / upStrokeExtract;

            if (timePerRad < quadVertex) { // if the time per radian is below this value, the result will be undefined
                timePerRad = quadVertex;    // if above case, set the time per radian to the minimum defined value
            }

            volumeEvent = ((-b - sqrt((b*b) - (4 * (a) * (c - (timePerRad))))) / (2*a)) * upStrokeExtract; // calculate volume based on quadratic trend line
        }
        
		volumeEvent -= leakRate * volumeLoopSeconds; //[L/s][s]=[L]
        if(volumeEvent < 0)
        {
            volumeEvent = 0; // we can't pump negative volume
        }
		// organize flow into 2 hours bins
        // The hour was read at the start of the handle movement routine       
        sendDebugMessage("Volume Event = ", volumeEvent);  //Debug
        sendDebugMessage("  for time slot ", hour);  //Debug

        sendDebugMessage("Seconds pumping: ", volumeLoopSeconds);  //Testing
       
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

