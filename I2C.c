/*
 * File:   I2C.c
 * Author: KSK0419
 *
 * Created on September 21, 2015, 3:25 PM
 */


#include "xc.h"
#include "I2C.h"
#include "IWPUtilities.h"
#include "Pin_Manager.h"


/////////////////////////////////////////////////////////////////////
////                                                             ////
////                    I2C FUNCTIONS                            ////
////                                                             ////
/////////////////////////////////////////////////////////////////////

/*********************************************************************
 * Function: SoftwareReset()
 * Input: None.
 * Output: None.
 * Overview: Resets software for I2C
 * Note: Contains a timeout loop that will disable then enable I2C
 ********************************************************************/
void SoftwareReset(void) {
    I2C1CONbits.I2CEN = 0; // Disconnect I2C function from I2C pins (on pins 17 an 18)
    configI2c(); // Configure 12C pins as 12C (on pins 17 and 18

    IdleI2C(); // Ensure module is idle
    StartI2C(); // Initiate START condition
    while (I2C1CONbits.SEN)
    { // Wait until START condition is complete
        // do nothing
    }
    int pulsesCreated = 0;
    pinDirectionIO(sclI2CPin, 1); // input
    pinDirectionIO(sdaI2CPin, 1); // input
    if ((digitalPinStatus(sclI2CPin) == 1) && (digitalPinStatus(sdaI2CPin) == 0))
    {
        pinDirectionIO(sdaI2CPin, 0); // make SDA an output
        digitalPinSet(sdaI2CPin, 0); // set SDA
        while ((pulsesCreated < 9) && digitalPinStatus(sdaI2CPin) == 0)
        { //PORTBbits.RB9 == 0){
            delaySCL();
            digitalPinSet(sclI2CPin, 1); //PORTBbits.RB9 = 1; // SCL
            delaySCL();
            digitalPinSet(sdaI2CPin, 0); //PORTBbits.RB9 = 0; // SCL
            delaySCL();
            pulsesCreated++;
        }
    }
    // put pulse code here
    RestartI2C(); // Initiate START condition
    while (I2C1CONbits.RSEN); // Wait until START condition is complete
    StopI2C(); // Initiate STOP condition
    while (I2C1CONbits.PEN); // Wait until STOP condition is complete
}

/*********************************************************************
 * Function: SoftwareResetRKF()
 * Input: None.
 * Output: None.
 * Overview: Resets software for I2C
 * Note: Contains a timeout loop that will disable then enable I2C
 ********************************************************************/
///////////// NOT REWRITTEN YET 3/31/17 
void SoftwareResetRKF(void) {
    
   // First just try turning the I2C interface Off and back On
    I2C1CONbits.I2CEN = 0; // Disconnect I2C function from I2C pins (on pins 17 an 18)
    configI2c(); // Configure 12C pins as 12C (on pins 17 and 18
    
    TMR1 = 0;  //use this to prevent a system hang
    int MaxTime = 10;  //number of Timer1 cycles expected for this whole function.  This is used to
                       //quit trying if things hang.
    
    while ((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT)){}//Wait for bus to be idle 
    I2C1CONbits.SEN = 1; //Generate Start COndition
    while ((TMR1<10)&&(I2C1CONbits.SEN)){ } //Wait for Start COndition
    
    // When the I2C link is idle, both SCL and SDA should be high
    // When the start condition is over, both the SCL and SDA lines should be low
    int pulsesCreated = 0;
    pinDirectionIO(sclI2CPin, 1); // input
    pinDirectionIO(sdaI2CPin, 1); // input
    if ((digitalPinStatus(sclI2CPin) == 1) && (digitalPinStatus(sdaI2CPin) == 0))
    {
        pinDirectionIO(sdaI2CPin, 0); // make SDA an output
        digitalPinSet(sdaI2CPin, 0);  // SDA = 0
        while ((pulsesCreated < 9) && digitalPinStatus(sdaI2CPin) == 0)
        { //PORTBbits.RB9 == 0){
            delaySCL();
            digitalPinSet(sclI2CPin, 1); //PORTBbits.RB9 = 1; // SCL
            delaySCL();
            digitalPinSet(sdaI2CPin, 0); //PORTBbits.RB9 = 0; // SCL
            delaySCL();
            pulsesCreated++;
        }
    }
    // put pulse code here
    RestartI2C(); // Initiate START condition
    while (I2C1CONbits.RSEN); // Wait until START condition is complete
    StopI2C(); // Initiate STOP condition
    while (I2C1CONbits.PEN); // Wait until STOP condition is complete
}

/*********************************************************************
 * Function: IdleI2C()
 * Input: None.
 * Output: None.
 * Overview: Waits for bus to become Idle
 * Note: Contains a timeout loop that will disable then enable I2C
 ********************************************************************/
unsigned int IdleI2C(void) {
    int timeOut = 0;
    while (I2C1STATbits.TRSTAT) //PIC is transmitting.  This is internal and should not hang
    {//Wait for bus Idle
        if (timeOut == 1300)
        { // time out loop incase I2C gets stuck
            SoftwareReset();
            invalid = 0xff;
            return;
        }

        timeOut++;
    }

}

/*********************************************************************
 * Function: StartI2C()
 * Input: None.
 * Output: None.
 * Overview: Generates an I2C Start Condition
 * Note: Contains a timeout loop that will disable then enable I2C
 ********************************************************************/
unsigned int StartI2C(void) {
    //This function generates an I2C start condition and returns status
    //of the Start.
    int timeOut = 0;
    I2C1CONbits.SEN = 1; //Generate Start COndition
    while (I2C1CONbits.SEN) //Wait for Start COndition
    {
        if (timeOut == 1300)
        { // time out loop incase I2C gets stuck
            SoftwareReset();
            invalid = 0xff;
            break;
        }
        timeOut++;
    }
    //return(I2C1STATbits.S); //Optionally return status
}

/*********************************************************************
 * Function: StopI2C()
 * Input: None.
 * Output: None.
 * Overview: Generates a bus stop condition
 * Note: None
 ********************************************************************/
unsigned int StopI2C(void) {
    //This function generates an I2C stop condition and returns status
    //of the Stop.
    int timeOut = 0;

    I2C1CONbits.PEN = 1; //Generate Stop Condition
    while (I2C1CONbits.PEN) //Wait for Stop
    {
        if (timeOut == 1300)
        { // time out loop incase I2C gets stuck
            SoftwareReset();
            invalid = 0xff;
            break;
        }
        timeOut++;
    }
    //return(I2C1STATbits.P); //Optional - return status
}

/*********************************************************************
 * Function: RestartI2C()
 * Input: None.
 * Output: None.
 * Overview: Generates a restart condition and optionally returns status
 * Note: None
 ********************************************************************/
void RestartI2C(void) {
    //This function generates an I2C Restart condition and returns status
    //of the Restart.
    int timeOut = 0;
    I2C1CONbits.RSEN = 1; //Generate Restart
    while (I2C1CONbits.RSEN) //Wait for restart
    {
        if (timeOut == 1300)
        { // time out loop incase I2C gets stuck
            SoftwareReset();
            invalid = 0xff;
            break;
        }
        timeOut++;
    }
    //return(I2C1STATbits.S); //Optional - return status
}

void NackI2C(void) {
    int timeOut = 0;
    I2C1CONbits.ACKDT = 1;
    I2C1CONbits.ACKEN = 1;
    while (I2C1CONbits.ACKEN)
    {
        if (timeOut == 1300)
        { // time out loop incase I2C gets stuck
            SoftwareReset();
            invalid = 0xff;
            break;
        }
        timeOut++;
    }
}

void AckI2C(void) {
    int timeOut = 0;
    I2C1CONbits.ACKDT = 0;
    I2C1CONbits.ACKEN = 1;
    while (I2C1CONbits.ACKEN)
    {
        if (timeOut == 1300)
        { // time out loop incase I2C gets stuck
            SoftwareReset();
            invalid = 0xff;
            break;
        }
        timeOut++;
    }
}
/*********************************************************************
 * Function: configI2c()
 * Input: None
 * Output: None.
 * Overview: Sets desired configuration bits for the I2C interface used to 
 *           communicate with the external RTCC chip. 
 * 
 * Note: None
 ********************************************************************/
void configI2c(void) {
    //From Jake's
    I2C1CONbits.A10M = 0; //Use 7-bit slave addresses
    I2C1CONbits.DISSLW = 1; // Disable Slew rate
    I2C1CONbits.IPMIEN = 0; //should be set to 0 when master
    //IFS1bits.MI2C1IF = 0; // Disable Interupt
    //^From Jake's
    // this is what is in Ghana 1/2017 I2C1BRG = 0x4E; // If Fcy = 8 Mhz this will set the baud to 100 khz
    I2C1BRG = 0x23; // If FNOSC = FRC (Foc = 8Mhz so Fcy = 4 Mhz) this will set the baud to 100 khz
    I2C1CONbits.I2CEN = 1; // Configures I2C pins as I2C (on pins 17 an 18)
}

/*********************************************************************
 * Function: WriteI2C()
 * Input: Byte to write.
 * Output: None.
 * Overview: Writes a byte out to the bus
 * Note: None
 ********************************************************************/
void WriteI2C(unsigned char byte) {
    //This function transmits the byte passed to the function
    int timeOut1 = 0;
    while (I2C1STATbits.TRSTAT)//Wait for bus to be idle -RKF I don't think we should have to wait for this.  If the start condition is finished the line should be ready
    {
        if (timeOut1 == 1300)
        { // time out loop incase I2C gets stuck
            SoftwareReset();
            invalid = 0xff;
            return;
        }
        timeOut1++;

    }
    I2C1TRN = byte; //Load byte to I2C1 Transmit buffer
    int timeOut2 = 0;
    while (I2C1STATbits.TBF) //wait for data transmission
    {
        if (timeOut2 == 1300)
        { // time out loop incase I2C gets stuck
            SoftwareReset();
            invalid = 0xff;
            return;
        }
        timeOut2++;

    }


}

/*********************************************************************
 * Function: ReadI2C()
 * Input: None
 * Output: Returns one Byte from Slave device
 * Overview: Receives Byte & Writes a Nack out to the bus
 * Note: None
 ********************************************************************/
unsigned int ReadI2C(void) {
    int timeOut1 = 0;
    int timeOut2 = 0;
    I2C1CONbits.ACKDT = 1; // Prepares to send NACK
    I2C1CONbits.RCEN = 1; // Gives control of clock to Slave device
    while (!I2C1STATbits.RBF) // Waits for register to fill up
    {
        if (timeOut1 == 1300)
        { // time out loop incase I2C gets stuck
            SoftwareReset();
            invalid = 0xff;
            return 0xff; // invalid
        }
        timeOut1++;

    }
    I2C1CONbits.ACKEN = 1; // Sends NACK or ACK set above
    while (I2C1CONbits.ACKEN) // Waits till ACK is sent (hardware reset)
    {
        if (timeOut2 == 1300)
        { // time out loop incase I2C gets stuck
            SoftwareReset();
            invalid = 0xff;
            return 0xff; //invalid

        }
        timeOut2++;

    }
    return I2C1RCV; // Returns data
}

/*********************************************************************
 * Function: delaySCL()
 * Input: None
 * Output: None
 * Overview: Pulse length for I2C pulse
 * Note: None
 ********************************************************************/
void delaySCL(void) {
    int timeKiller = 0; //don't delete
    int myIndex = 0;
    while (myIndex < 5)
    {
        myIndex++;
    }
}


/////////////////////////////////////////////////////////////////////
////                                                             ////
////                    RTCC FUNCTIONS                           ////
////                                                             ////
/////////////////////////////////////////////////////////////////////

/*********************************************************************
 * Function: readRTCC
 * Input: enum RTCCaddress
 * Output: None
 * Overview: reads from the register specified by the input
 * Note: None
 ********************************************************************/

int readRTCC(enum RTCCregister RTCCregister) {
    unsigned char address;
    unsigned char mask;
    int data;

    switch (RTCCregister) {
        case SEC_REGISTER:
            address = 0x00;
            mask = 0x7F;
            break;

        case MIN_REGISTER:
            address = 0x01;
            mask = 0x7F;
            break;

        case HOUR_REGISTER:
            address = 0x02;
            mask = 0x3F;
            break;

        case WKDAY_REGISTER:
            address = 0x03;
            mask = 0x07;
            break;

        case DATE_REGISTER:
            address = 0x04;
            mask = 0x3F;
            break;

        case MONTH_REGISTER:
            address = 0x05;
            mask = 0x1F;
            break;

        case YEAR_REGISTER:
            address = 0x06;
            mask = 0xFF;
            break;
    }

    configI2c(); // sets up I2C
    StartI2C();
    WriteI2C(0xde); // MCP7490N device address + write command
    IdleI2C();
    WriteI2C(address); // device address for the given register on MCP7490N
    IdleI2C();
    RestartI2C();
    IdleI2C();
    WriteI2C(0xdf); // MCP7490N device address + read command
    IdleI2C();
    data = (int) ReadI2C();
    StopI2C();
    if (invalid == 0xFF)
    {
        invalid = 0;
        data = readRTCC(address);
    }
    data = data & mask; // removes unnecessary bits (mostly control bits and other non-time data)
    return data; // returns the time in address as a BCD number

}

/*********************************************************************
 * Function: turnOffClockOscilator()
 * Input: None
 * Output: None
 * Overview: Turns off RTCC Oscillator MCP7940N so it can be
 * set
 * Note: None
 ********************************************************************/
void turnOffClockOscilator_old(void) {
    // turns off oscilator to prepare to set time
    StartI2C();
    WriteI2C(0xde); //Device Address (RTCC) + Write Command
    IdleI2C();
    WriteI2C(0x00); //address reg. for sec
    IdleI2C();
    WriteI2C(0x00); //Turn off oscillator and sets seconds to 0
    IdleI2C();
    if (invalid == 0xFF)
    {
        invalid = 0;
        turnOffClockOscilator();
    }
    StopI2C();
}

/*********************************************************************
 * Function: turnOffClockOscilator()
 * Input: None
 * Output: None
 * Overview: Turns off RTCC Oscillator MCP7940N so the registers for
 *           time and date can be set
 * Note: None
 ********************************************************************/
int turnOffClockOscilator(void) {
    TMR1 = 0;  //use this to prevent a system hang
    int MaxTime = 10;  //number of Timer1 cycles expected for this whole function.  This is used to
                     //quit trying if things hang.
    int success = 0;  // assume we had a problem
      
    I2C1CONbits.SEN = 1; //Generate Start COndition
    while ((TMR1<10)&&(I2C1CONbits.SEN)){ } //Wait for Start COndition
    
    //Tell External RTCC that we are going to WRITE to it
    I2C1TRN = 0xde; //Load Device Address for RTCC + Write Command into I2C1 Transmit buffer
    while ((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT)){}//Wait for bus to be idle 
      
    //Specify that you are writing to the register for Seconds
    I2C1TRN = 0x00; //address reg. for sec
    while((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT)){} //PIC is transmitting. 

    //Write to the SECONDS register 0 seconds and MSB = 0 which turns off oscillator
    I2C1TRN = 0x00; //Turn off oscillator since MSB = 0
    while((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT)){} //PIC is transmitting. 

    //StopI2C 
    I2C1CONbits.PEN = 1; //Generate Stop Condition
    while ((TMR1<MaxTime)&&(I2C1CONbits.PEN)){ } //Wait for Stop

    if(TMR1 < MaxTime){success = 1;}
    return success;
}

int getSecondI2C(void) //may want to pass char address to it in the future
{
    int sec; // temp var to hold seconds information

    configI2c(); // sets up I2C
    StartI2C();
    WriteI2C(0xde); // MCP7490N device address + write command
    IdleI2C();
    WriteI2C(0x00); // device address for the Seconds register on MCP7490N
    IdleI2C();
    RestartI2C();
    IdleI2C();
    WriteI2C(0xdf); // MCP7490N device address + read command
    IdleI2C();
    sec = (int) ReadI2C();
    StopI2C();
    if (invalid == 0xFF)
    {
        invalid = 0;
        sec = getSecondI2C();
    }
    sec = sec & 0x7f; // removes Oscillator bit
    //sec = BcdToDec(sec); // converts sec to a decimal number
    return sec; // returns the time in sec as a demimal number
}

int getMinuteI2C_old(void) {
    int min; // temp var to hold seconds information
    configI2c(); // sets up I2C
    StartI2C();
    WriteI2C(0xde); // MCP7490N device address + write command
    IdleI2C();
    WriteI2C(0x01); // device address for the minutes register on MCP7490N
    IdleI2C();
    RestartI2C();
    IdleI2C();
    WriteI2C(0xdf); // MCP7490N device address + read command
    IdleI2C();
    min = (int) ReadI2C();
    StopI2C();
    if (invalid == 0xFF)
    {
        invalid = 0;
        min = getMinuteI2C();
    }
    min = min & 0x7f; // removes unused bit
    //min = BcdToDec(min); // converts min to a decimal number
    return min; // returns the time in min as a demimal number
}

int getMinuteI2C(void) {
    int min = 0;
    
    
    int MaxTime = 10;  //number of Timer1 cycles expected for this whole function.  This is used to
                     //quit trying if things hang. (usually takes about 410us)
    
    configI2c(); // sets up I2C
    TMR1 = 0;  
         
    I2C1CONbits.SEN = 1; //Generate Start COndition
    while ((TMR1<MaxTime)&&(I2C1CONbits.SEN)){ } //Wait for Start COndition
    
    //Write I2C Address indicating a WRITE operation
    I2C1TRN = 0xde; //Load Device Address for RTCC + Write Command into I2C1 Transmit buffer
    while ((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT)){}//Wait for bus to be idle 
  // should check for an ACK'
    
     
    //Write I2C - specify the minute register on MCP7490N
    I2C1TRN = 0x01; //address reg. for min
    while((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT));  //PIC is transmitting. 
     // should check for an ACK'
    
    // We want to change over to READING so we need to command a Restart
    I2C1CONbits.RSEN = 1; //Generate Restart
    while ((TMR1 < MaxTime) && (I2C1CONbits.RSEN)); //Wait for restart
    
     //Write I2C Address indicating a READ operation
     I2C1TRN = 0xdf; //Load Device Address for RTCC + Read Command into I2C1 Transmit buffer
     while ((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT)); //Wait for bus to be idle 
  // should check for an ACK'
     
    //Now read the value for the current hour
    I2C1CONbits.ACKDT = 1; // Prepares to send NACK (we only want 1 byte from RTCC)
    I2C1CONbits.RCEN = 1; // Enable receive mode
    while ((TMR1<MaxTime)&&(!I2C1STATbits.RBF));  // Waits for register to fill up

    I2C1CONbits.ACKEN = 1; // Send the NACK set above.  This absence of a ACK' tells slave we don't want any more data
    while ((TMR1<MaxTime) && (I2C1CONbits.ACKEN));  // Waits till ACK is sent (hardware reset)
    min = I2C1RCV & 0x7f; //removes unused bits, expecting BCD in the range of 0 - 59
  
    // Generate a STOP 
    I2C1CONbits.PEN = 1; //Generate Stop Condition
    while ((TMR1<MaxTime)&&(I2C1CONbits.PEN)); //Wait for Stop
    // How do we know that the value we just got makes sense?
    
    
    if((TMR1 > MaxTime)||(min > 0x60)){  //something went wrong (BCD 60)
        min = (minute/10 *16)+(minute % 10); // If the read was unsuccessful, return the last known hour
                                        // Remember the returned value is supposed to be in BCD
    }
    
    return min; 
                
}

int getHourI2C_old(void) {
    int hr; // temp var to hold seconds information
    configI2c(); // sets up I2C
    StartI2C();
    WriteI2C(0xde); // MCP7490N device address + write command
    IdleI2C();
    WriteI2C(0x02); // device address for the hours register on MCP7490N
    IdleI2C();
    RestartI2C();
    IdleI2C();
    WriteI2C(0xdf); // MCP7490N device address + read command
    IdleI2C();
    hr = (int) ReadI2C();
    StopI2C();

    if (invalid == 0xFF)
    {
        invalid = 0;
        hr = getHourI2C_old();
    }
    hr = hr & 0x3f; // removes unused bits
    //hr = BcdToDec(hr); // converts hr to a decimal number
    return hr; // returns the time in hr as a demimal number
}

/*********************************************************************
 * Function: getHourI2C()
 * Input: None
 * Output: None
 * Overview: Reads the current hour from the external RTCC.  If an invalid read
 *           occurs due to things taking too long or the returned value is not in 
 *          the range from 0-23, the value of hour when this was called is returned
 * Note: None
 ********************************************************************/
int getHourI2C(void) {
    int hr = 0;
    
    
    int MaxTime = 10;  //number of Timer1 cycles expected for this whole function.  This is used to
                     //quit trying if things hang. (usually takes about 410us)
    extRtccChecked++;
    
    configI2c(); // sets up I2C
    TMR1 = 0;  
         
    I2C1CONbits.SEN = 1; //Generate Start COndition
    while ((TMR1<MaxTime)&&(I2C1CONbits.SEN)){ } //Wait for Start COndition
    
    //Write I2C Address indicating a WRITE operation
    I2C1TRN = 0xde; //Load Device Address for RTCC + Write Command into I2C1 Transmit buffer
    while ((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT)){}//Wait for bus to be idle 
  // should check for an ACK'
    
     
    //Write I2C - specify the hour register on MCP7490N
    I2C1TRN = 0x02; //address reg. for sec
    while((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT));  //PIC is transmitting. 
     // should check for an ACK'
    
    // We want to change over to READING so we need to command a Restart
    I2C1CONbits.RSEN = 1; //Generate Restart
    while ((TMR1 < MaxTime) && (I2C1CONbits.RSEN)); //Wait for restart
    
     //Write I2C Address indicating a READ operation
     I2C1TRN = 0xdf; //Load Device Address for RTCC + Read Command into I2C1 Transmit buffer
     while ((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT)); //Wait for bus to be idle 
  // should check for an ACK'
     
    //Now read the value for the current hour
    I2C1CONbits.ACKDT = 1; // Prepares to send NACK (we only want 1 byte from RTCC)
    I2C1CONbits.RCEN = 1; // Enable receive mode
    while ((TMR1<MaxTime)&&(!I2C1STATbits.RBF));  // Waits for register to fill up

    I2C1CONbits.ACKEN = 1; // Send the NACK set above.  This absence of a ACK' tells slave we don't want any more data
    while ((TMR1<MaxTime) && (I2C1CONbits.ACKEN));  // Waits till ACK is sent (hardware reset)
    hr = I2C1RCV & 0x3f; //removes unused bits, expecting BCD in the range of 0 - 23
  
    // Generate a STOP 
    I2C1CONbits.PEN = 1; //Generate Stop Condition
    while ((TMR1<MaxTime)&&(I2C1CONbits.PEN)); //Wait for Stop
    // How do we know that the value we just got makes sense?
    
    
    if((TMR1 > MaxTime)||(hr > 0x24)){  //something went wrong (BCD 24)
        hr = (hour/10 *16)+(hour % 10); // If the read was unsuccessful, return the last known hour
                                        // Remember the returned value is supposed to be in BCD
        extRtccHourSet = 0;             // Cleared because RTCC hour didn't update
        sendDebugMessage("The time update failed", 0);
        extRtccTalked++;
        if (extRtccChecked > 60) {
            hr++;
            extRtccManualSet = 1;
        }
    }
    else { // if the max time didn't elapsed, the RTCC talked so set the bit
        sendDebugMessage("The time update succeeded", 0);
    }
    
    
    
    return hr; 
                
}

int getYearI2C_old(void) {
    int yr; // temp var to hold seconds information
    configI2c(); // sets up I2C
    StartI2C();
    WriteI2C(0xde); // MCP7490N device address + write command
    IdleI2C();
    WriteI2C(0x06); // device address for the years register on MCP7490N
    IdleI2C();
    RestartI2C();
    IdleI2C();
    WriteI2C(0xdf); // MCP7490N device address + read command
    IdleI2C();
    yr = (int) ReadI2C();
    StopI2C();
    if (invalid == 0xFF)
    {
        invalid = 0;
        yr = getYearI2C();
    }
    return yr; // returns the time in hr as a demimal number
}

int getYearI2C(void) {
    int yr = 0;
    
    
    int MaxTime = 10;  //number of Timer1 cycles expected for this whole function.  This is used to
                     //quit trying if things hang. (usually takes about 410us)
    
    configI2c(); // sets up I2C
    TMR1 = 0;  
         
    I2C1CONbits.SEN = 1; //Generate Start COndition
    while ((TMR1<MaxTime)&&(I2C1CONbits.SEN)){ } //Wait for Start COndition
    
    //Write I2C Address indicating a WRITE operation
    I2C1TRN = 0xde; //Load Device Address for RTCC + Write Command into I2C1 Transmit buffer
    while ((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT)){}//Wait for bus to be idle 
  // should check for an ACK'
    
     
    //Write I2C - specify the minute register on MCP7490N
    I2C1TRN = 0x06; //address reg. for min
    while((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT));  //PIC is transmitting. 
     // should check for an ACK'
    
    // We want to change over to READING so we need to command a Restart
    I2C1CONbits.RSEN = 1; //Generate Restart
    while ((TMR1 < MaxTime) && (I2C1CONbits.RSEN)); //Wait for restart
    
     //Write I2C Address indicating a READ operation
     I2C1TRN = 0xdf; //Load Device Address for RTCC + Read Command into I2C1 Transmit buffer
     while ((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT)); //Wait for bus to be idle 
  // should check for an ACK'
     
    //Now read the value for the current hour
    I2C1CONbits.ACKDT = 1; // Prepares to send NACK (we only want 1 byte from RTCC)
    I2C1CONbits.RCEN = 1; // Enable receive mode
    while ((TMR1<MaxTime)&&(!I2C1STATbits.RBF));  // Waits for register to fill up

    I2C1CONbits.ACKEN = 1; // Send the NACK set above.  This absence of a ACK' tells slave we don't want any more data
    while ((TMR1<MaxTime) && (I2C1CONbits.ACKEN));  // Waits till ACK is sent (hardware reset)
  
    // Generate a STOP 
    I2C1CONbits.PEN = 1; //Generate Stop Condition
    while ((TMR1<MaxTime)&&(I2C1CONbits.PEN)); //Wait for Stop
    // How do we know that the value we just got makes sense?
    
    
    if(TMR1 > MaxTime){  //something went wrong
        yr = (year/10 *16)+(year % 10); // If the read was unsuccessful, return the last known hour
                                        // Remember the returned value is supposed to be in BCD
    }
    
    return yr; 
                
}


int getMonthI2C_old(void) {
    int mnth; // temp var to hold seconds information
    configI2c(); // sets up I2C
    StartI2C();
    WriteI2C(0xde); // MCP7490N device address + write command
    IdleI2C();
    WriteI2C(0x05); // device address for the years register on MCP7490N
    IdleI2C();
    RestartI2C();
    IdleI2C();
    WriteI2C(0xdf); // MCP7490N device address + read command
    IdleI2C();
    mnth = (int) ReadI2C();
    StopI2C();
    if (invalid == 0xFF)
    {
        invalid = 0;
        mnth = getMonthI2C();
    }
    mnth = mnth & 0x1F;
    return mnth; // returns the time in hr as a demimal number

}

int getMonthI2C(void) {
    int mnth = 0;
    
    
    int MaxTime = 10;  //number of Timer1 cycles expected for this whole function.  This is used to
                     //quit trying if things hang. (usually takes about 410us)
    
    configI2c(); // sets up I2C
    TMR1 = 0;  
         
    I2C1CONbits.SEN = 1; //Generate Start COndition
    while ((TMR1<MaxTime)&&(I2C1CONbits.SEN)){ } //Wait for Start COndition
    
    //Write I2C Address indicating a WRITE operation
    I2C1TRN = 0xde; //Load Device Address for RTCC + Write Command into I2C1 Transmit buffer
    while ((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT)){}//Wait for bus to be idle 
  // should check for an ACK'
    
     
    //Write I2C - specify the minute register on MCP7490N
    I2C1TRN = 0x05; //address reg. for month
    while((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT));  //PIC is transmitting. 
     // should check for an ACK'
    
    // We want to change over to READING so we need to command a Restart
    I2C1CONbits.RSEN = 1; //Generate Restart
    while ((TMR1 < MaxTime) && (I2C1CONbits.RSEN)); //Wait for restart
    
     //Write I2C Address indicating a READ operation
     I2C1TRN = 0xdf; //Load Device Address for RTCC + Read Command into I2C1 Transmit buffer
     while ((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT)); //Wait for bus to be idle 
  // should check for an ACK'
     
    //Now read the value for the current hour
    I2C1CONbits.ACKDT = 1; // Prepares to send NACK (we only want 1 byte from RTCC)
    I2C1CONbits.RCEN = 1; // Enable receive mode
    while ((TMR1<MaxTime)&&(!I2C1STATbits.RBF));  // Waits for register to fill up

    I2C1CONbits.ACKEN = 1; // Send the NACK set above.  This absence of a ACK' tells slave we don't want any more data
    while ((TMR1<MaxTime) && (I2C1CONbits.ACKEN));  // Waits till ACK is sent (hardware reset)
    mnth = I2C1RCV & 0x1f; //removes unused bits, expecting BCD in the range of 0 - 12
  
    // Generate a STOP 
    I2C1CONbits.PEN = 1; //Generate Stop Condition
    while ((TMR1<MaxTime)&&(I2C1CONbits.PEN)); //Wait for Stop
    // How do we know that the value we just got makes sense?
    
    
    if((TMR1 > MaxTime)||(mnth > 0x12)){  //something went wrong (BCD 12)
        mnth = (month/10 *16)+(month % 10); // If the read was unsuccessful, return the last known hour
                                        // Remember the returned value is supposed to be in BCD
    }
    
    return mnth; 
                
}

int getWkdayI2C(void) {
    unsigned char wkday; // temp var to hold seconds information
    configI2c(); // sets up I2C
    StartI2C();
    WriteI2C(0xde); // MCP7490N device address + write command
    IdleI2C();
    WriteI2C(0x03); // device address for the years register on MCP7490N
    IdleI2C();
    RestartI2C();
    IdleI2C();
    WriteI2C(0xdf); // MCP7490N device address + read command
    IdleI2C();
    wkday = (int) ReadI2C();
    StopI2C();
    if (invalid == 0xFF)
    {
        invalid = 0;
        wkday = getWkdayI2C();
    }
    wkday = wkday & 0x07; // converts yr to a decimal number
    return wkday; // returns the time in hr as a demimal number
}

int getDateI2C_old(void) {
    int date; // temp var to hold dat information
    configI2c(); // sets up I2C
    StartI2C();
    WriteI2C(0xde); // MCP7490N device address + write command
    IdleI2C();
    WriteI2C(0x04); // device address for the date register on MCP7490N
    IdleI2C();
    RestartI2C();
    IdleI2C();
    WriteI2C(0xdf); // MCP7490N device address + read command
    IdleI2C();
    date = (int) ReadI2C();
    StopI2C();
    if (invalid == 0xFF)
    {
        invalid = 0;
        date = getDateI2C();
    }
    date = date & 0x3f; // removes unused bits
    //date = BcdToDec(date); // converts yr to a decimal number
    return date; // returns the time in hr as a demimal number
}

int getDateI2C(void) {
    int dt = 0;
    
    
    int MaxTime = 10;  //number of Timer1 cycles expected for this whole function.  This is used to
                     //quit trying if things hang. (usually takes about 410us)
    
    configI2c(); // sets up I2C
    TMR1 = 0;  
         
    I2C1CONbits.SEN = 1; //Generate Start COndition
    while ((TMR1<MaxTime)&&(I2C1CONbits.SEN)){ } //Wait for Start COndition
    
    //Write I2C Address indicating a WRITE operation
    I2C1TRN = 0xde; //Load Device Address for RTCC + Write Command into I2C1 Transmit buffer
    while ((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT)){}//Wait for bus to be idle 
  // should check for an ACK'
    
     
    //Write I2C - specify the minute register on MCP7490N
    I2C1TRN = 0x04; //address reg. for date
    while((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT));  //PIC is transmitting. 
     // should check for an ACK'
    
    // We want to change over to READING so we need to command a Restart
    I2C1CONbits.RSEN = 1; //Generate Restart
    while ((TMR1 < MaxTime) && (I2C1CONbits.RSEN)); //Wait for restart
    
     //Write I2C Address indicating a READ operation
     I2C1TRN = 0xdf; //Load Device Address for RTCC + Read Command into I2C1 Transmit buffer
     while ((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT)); //Wait for bus to be idle 
  // should check for an ACK'
     
    //Now read the value for the current hour
    I2C1CONbits.ACKDT = 1; // Prepares to send NACK (we only want 1 byte from RTCC)
    I2C1CONbits.RCEN = 1; // Enable receive mode
    while ((TMR1<MaxTime)&&(!I2C1STATbits.RBF));  // Waits for register to fill up

    I2C1CONbits.ACKEN = 1; // Send the NACK set above.  This absence of a ACK' tells slave we don't want any more data
    while ((TMR1<MaxTime) && (I2C1CONbits.ACKEN));  // Waits till ACK is sent (hardware reset)
    dt = I2C1RCV & 0x3f; //removes unused bits, expecting BCD in the range of 0 - 31
  
    // Generate a STOP 
    I2C1CONbits.PEN = 1; //Generate Stop Condition
    while ((TMR1<MaxTime)&&(I2C1CONbits.PEN)); //Wait for Stop
    // How do we know that the value we just got makes sense?
    
    
    if((TMR1 > MaxTime)||(dt > 0x31)){  //something went wrong (BCD 60)
        dt = (date/10 *16)+(date % 10); // If the read was unsuccessful, return the last known hour
                                        // Remember the returned value is supposed to be in BCD
    }
    
    return dt; 
                
}

/*********************************************************************
 * Function: setTime()
 * Input: SS MM HH WW DD MM YY
 * Output: Success = 1 if it worked and 0 if not
 * Overview: Sets time for MCP7940N
 * Note: uses DecToBcd and I2C functions.  This is a new version of this function
 *       which does not break the I2C functions into such small parts
 ********************************************************************/
int setTime(char sec, char min, char hr, char wkday, char date, char month, char year){
    int success = 0;  //used to determine if sub operations worked.  Assume this is false
    int MaxTime = 20;  //number of Timer1 cycles expected for this whole function.  This is used to
                     //quit trying if things hang.  Should take about 17
    hour = hr;  //Set the system variable for hour equal to what is being sent to the external RTCC
    int leapYear;
    if (year % 4 == 0)
    {
        leapYear = 1; //Is a leap Year
    } else
    {
        leapYear = 0; //Is not a leap Year
    }
    char BCDsec = DecToBcd(sec); // To BCD
    char BCDmin = DecToBcd(min); // To BCD
    char BCDhr = DecToBcd(hr);
    char BCDwkday = DecToBcd(wkday); // To BCD
    char BCDdate = DecToBcd(date);
    char BCDmonth = DecToBcd(month); // To BCD
    char BCDyear = DecToBcd(year);
   
    BCDhr = BCDhr & 0b10111111; // clearing bit 6 of the hour register sets the hour to 24 hr time
    BCDwkday = BCDwkday | 0b00001000; // the 1 in this bit says the external battery backup supply is enabled.
   
    
    if (leapYear == 0)
    {
        BCDmonth = BCDmonth & 0b11011111; //Not a leap year
    } else
    {
        BCDmonth = BCDmonth | 0b00100000; //Is a leap year
    }
    configI2c();
    success = turnOffClockOscilator();
    //------------------------------------------------------------------------------
    // sets clock
    //------------------------------------------------------------------------------
         
    TMR1 = 0;  //use this to prevent a system hang
      
    I2C1CONbits.SEN = 1; //Generate Start COndition
    while ((TMR1<MaxTime)&&(I2C1CONbits.SEN)){ } //Wait for Start COndition
    
    //Tell External RTCC that we are writing to it
    I2C1TRN = 0xde; //Load Device Address for RTCC + Write Command into I2C1 Transmit buffer
    while ((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT)){}//PIC is transmitting

    //Point to the RTCC register for MINUTES
    I2C1TRN = 0x01; //address register for minutes
    while((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT)){} //PIC is transmitting.  
    
    //Write Data
    // Internal register pointer increments automatically no no need to send register address each time
    I2C1TRN = BCDmin; //Load Minutes  
    while((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT)){} //PIC is transmitting. 
    I2C1TRN = BCDhr; //Load hour
    while((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT)){} //PIC is transmitting. 
    I2C1TRN = BCDwkday; //Load day of the week assumes 1-7, we use 1 = Sunday 
    while((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT)){} //PIC is transmitting. 
    I2C1TRN = BCDdate; //Load day of the month 
    while((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT)){} //PIC is transmitting. 
    I2C1TRN = BCDmonth; //Load month
    while((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT)){} //PIC is transmitting. 
    I2C1TRN = BCDyear; //Load year (20 is assumed) 
    while((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT)){} //PIC is transmitting. 
    
    // Generate a STOP
    I2C1CONbits.PEN = 1; //Generate Stop Condition
    while ((TMR1<MaxTime)&&(I2C1CONbits.PEN)){ } //Wait for Stop
    
    //------------------------------------------------------------------------------
    // Sets seconds and turns on oscillator
    //------------------------------------------------------------------------------
   
   // Start I2C   
    I2C1CONbits.SEN = 1; //Generate Start COndition
    while ((TMR1<MaxTime)&&(I2C1CONbits.SEN)){ } //Wait for Start COndition
    
    //Tell External RTCC that we are writing to it
    I2C1TRN = 0xde; //Load Device Address for RTCC + Write Command into I2C1 Transmit buffer
    while((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT)){} //PIC is transmitting. 

    //Point to the RTCC register for SECONDS
    I2C1TRN = 0x00; //address register for seconds
    while((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT)){} //PIC is transmitting. 
    
    BCDsec = BCDsec | 0x80; // setting the MSB of the seconds register in the MCP7940 turns on the oscillator
    I2C1TRN = BCDsec; //Load seconds and turn oscillator on
    while((TMR1<MaxTime)&&(I2C1STATbits.TRSTAT)){} //PIC is transmitting.
    
    // Generate a STOP
    I2C1CONbits.PEN = 1; //Generate Stop Condition
    while ((TMR1<MaxTime)&&(I2C1CONbits.PEN)){ } //Wait for Stop
    
    if(TMR1 < MaxTime){success = 1;}
    return success;
}
/*********************************************************************
 * Function: setTime()
 * Input: SS MM HH WW DD MM YY
 * Output: None
 * Overview: Sets time for MCP7940N
 * Note: uses DecToBcd and I2C functions
 ********************************************************************/
void setTime_old(char sec, char min, char hr, char wkday, char date, char month, char year) {
    int leapYear;
    if (year % 4 == 0)
    {
        leapYear = 1; //Is a leap Year
    } else
    {
        leapYear = 0; //Is not a leap Year
    }
    char BCDsec = DecToBcd(sec); // To BCD
    char BCDmin = DecToBcd(min); // To BCD
    char BCDhr = DecToBcd(hr);
    char BCDwkday = DecToBcd(wkday); // To BCD
    char BCDdate = DecToBcd(date);
    char BCDmonth = DecToBcd(month); // To BCD
    char BCDyear = DecToBcd(year);
    BCDsec = BCDsec | 0x80; // add turn on oscilator bit
    BCDhr = BCDhr & 0b10111111; // makes 24 hr time
    BCDwkday = BCDwkday | 0b00001000; // the 1 in this bit says the external battery backup supply is enabled.
    // To enable: Flip bits and OR it to turn on (NOT CURRENTLY ENABLED).
    if (leapYear == 0)
    {
        BCDmonth = BCDmonth & 0b11011111; //Not a leap year
    } else
    {
        BCDmonth = BCDmonth | 0b00100000; //Is a leap year
    }
    configI2c();
    turnOffClockOscilator();
    //------------------------------------------------------------------------------
    // sets clock
    //------------------------------------------------------------------------------
    StartI2C();
    WriteI2C(0xDE); //Device Address (RTCC) + Write Command
    IdleI2C();
    WriteI2C(0x01); //Adress for minutes
    IdleI2C();
    WriteI2C(BCDmin); //Load min
    IdleI2C();
    WriteI2C(BCDhr); //Load hour
    IdleI2C();
    WriteI2C(BCDwkday); //Load day of week
    IdleI2C();
    WriteI2C(BCDdate); //Load Date
    IdleI2C();
    WriteI2C(BCDmonth); //Load Month
    IdleI2C();
    WriteI2C(BCDyear); // Load Year
    IdleI2C();
    StopI2C();
    //------------------------------------------------------------------------------
    // Sets seconds and turns on oscilator
    //------------------------------------------------------------------------------
    StartI2C();
    WriteI2C(0xDE); //Device Address (RTCC) + Write Command
    IdleI2C();
    WriteI2C(0x00); //address reg. for sec
    IdleI2C();
    WriteI2C(BCDsec); //Turn on oscillator and sets seconds
    IdleI2C();
    StopI2C();
}