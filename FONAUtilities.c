/*
 * File:   FONAUtilities.c
 * Author: rfish
 *
 * Created on November 15, 2017, 5:17 PM
 */


#include "xc.h"
#include "IWPUtilities.h"
#include "Pin_Manager.h"
#include "I2C.h"
#include "FONAUtilities.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <xc.h>
#include <string.h>
#include <p24FV32KA302.h>

// ****************************************************************************
// This file includes the functions and variables necessary to interact with
// The FONA GSM module for SMS messaging
// ****************************************************************************

// ****************************************************************************
// *** Global Variables *******************************************************
// ****************************************************************************
//char DebugphoneNumber[] = "0548345382"; // Number for the Black Phone - MOVED to kpachelo
//char DebugphoneNumber[] = "0548982327"; // Number for Immanuel programmed in as debug for kpachelo
//char DebugphoneNumber[] = "+17176837803"; // Number for Fish cell phone 
char DebugphoneNumber[] = "+17177784498"; // Upside 
///char DebugphoneNumber[] = "+18458007595"; //Number for Paul Zwert cell phone
//char MainphoneNumber[]="+17177784498"; // Upside Wireless
char MainphoneNumber[]="+17176837803"; // Randy
char SendingPhoneNumber[]="+17177784498"; //this is read from the received SMS message default to mine
//char phoneNumber[] = "+17177784498"; // Number Used to send text message report (daily or hourly)
//char phoneNumber []="+17176837803"; // Randy
char* phoneNumber = MainphoneNumber; // Randy
char FONAmsgStatus[11]; //Message REC/STO, UNREAD/READ, UNSENT/SENT
char ReceiveTextMsg[280]; //This is the string used to buffer up a message read from the FONA
int NumCharInTextMsg = 0; //Keeps track of the number of characters in the received text string
char ReceiveTextMsgFlag = 0; //Set to 1 when a complete text message has been received


//char phoneNumber[] = "+2330548345382"; // Number for the Black Phone
//char phoneNumber[] = "+17177784498"; // Number for Upside Wireless
//char phoneNumber[] = "+233545822291"; // Number for the White Phone Ghana trip 3
//char phoneNumber[] = "+233545823475"; // Number for the Black Phone Ghana trip 3
//char phoneNumber[] = "+19783840645"; // Number for Jake Sargent
// char phoneNumber[] = "+19107094602"; //Number for John Harro
// char phoneNumber[] = "+17176837803"; //Number for Randy Fish
//char phoneNumber2[] = "+17173039306"; // Tony's number
//char phoneNumber[] = "+13018737202"; // Number for Jacqui Young
int noon_msg_sent = 0;  //set to 1 when noon message has been sent
int hour_msg_sent = 0;  //set to 1 when the hourly message has been sent




// ****************************************************************************
// *** FONA Functions *******************************************************
// ****************************************************************************

/*********************************************************************
 * Function: turnOffSIM
 * Input: None
 * Output: NSIM_OFF  this is a 1 if the SIM turned OFF and 0 if not
 * Overview: Turns of the SIM900
 * Note: Pic Dependent
 * TestDate: Not tested as of 03-05-2015
 ********************************************************************/
int turnOffSIM() {
    int SIM_OFF = 0;  // Assume the SIM is not off
    digitalPinSet(simVioPin, 1); //PORTAbits.RA1 = 1; //Tells Fona what logic level to use for UART
    if (digitalPinStatus(statusPin) == 1) { //Checks see if the Fona is ON 1=Yes so turn it off
        digitalPinSet(pwrKeyPin, 0); //PORTBbits.RB6 = 0; //set low pin 15 for 2000ms to turn on Fona
        delayMs(2000);
    }
    if (digitalPinStatus(statusPin) == 0) { //Checks see if the Fona is OFF 0 = OFF so don't do anything
        SIM_OFF = 1;
    }
    digitalPinSet(pwrKeyPin, 1); //PORTBbits.RB6 = 1; // Reset the Power Key so it can be turned on later (pin 15)

    return SIM_OFF;//	while (digitalPinStatus(statusPin) == 1){ //Checks see if the Fona is on pin
 }

/*********************************************************************
 * Function: turnOnSIM
 * Input: None
 * Output: SIM_ON  this is a 1 if the SIM turned on and 0 if not
 * Overview: Turns on SIM900
 * Note: Pic Dependent
 * TestDate: Not tested as of 03-05-2015
 * delayMs(int ms)
 ********************************************************************/
int turnOnSIM() {
    int SIM_ON = 0;  // Assume the SIM is not on
    digitalPinSet(simVioPin, 1); //PORTAbits.RA1 = 1; //Tells Fona what logic level to use for UART
    if (digitalPinStatus(statusPin) == 0) { //Checks see if the Fona is off pin
        digitalPinSet(pwrKeyPin, 0); //PORTBbits.RB6 = 0; //set low pin 15 for 2000ms to turn on Fona
        delayMs(2000);
    }
    if (digitalPinStatus(statusPin) != 0) { //Checks see if the Fona is off pin
        SIM_ON = 1;
    }
    digitalPinSet(pwrKeyPin, 1); //PORTBbits.RB6 = 1; // Reset the Power Key so it can be turned off later (pin 15)

    return SIM_ON;
    //	while (digitalPinStatus(statusPin) == 0) // While STATUS light is not on (SIM900 is off)
    //	{
    //		digitalPinSet(pwrKeyPin, 1); // Hold in PWRKEY button
    //	}
    //
    //	digitalPinSet(pwrKeyPin, 0); // Let go of PWRKEY
}

/*********************************************************************
 * Function: tryToConnectToNetwork
 * Input: None
 * Output: None
 * Overview: This function tests for network status and attempts to connect to the
 * network. If no network is after 7 attempts (between 20sec and 45sec), 
 * a zero is returned indicating that connection to the network failed
 * TestDate: Not tested as of 03-14-2017
 ********************************************************************/
int tryToConnectToNetwork() {
    int success = 0; // assume we were unable to connect to the network
    int networkTimeout = 0; // Stores the number of times we did not have connection
    int networkConnectionCount = 0; // Stores the number of times we have detected a connection
    int keepTrying = 1; // A flag used to keep trying to connect to the network
    while (keepTrying) // Wait until connected to the network, or we tried for 20 seconds
    {
        delayMs(1000); // Delay for 1 second
        // Check for network take the appropriate action
        if (connectedToNetwork()) {
            networkConnectionCount++;
            // 4 consecutive connections means we can exit the loop
            if (networkConnectionCount == 4) {
                keepTrying = 0;
                success = 1;  // we have a network connection
            }
        } else {
            // If we have no network, reset the counter
            networkConnectionCount = 0;
            // Increase the network timeout
            networkTimeout++;
            // Each attempt to connect takes 3-6sec if there is no network.  We will try 7 times
            if (networkTimeout > 7) {
                    keepTrying = 0;
            }
        }
    }
    return success;
}

/*********************************************************************
 * Function: connectedToNetwork
 * Input: None
 * Output: 1 if network connected 0 if not
 * Overview: Measures the time from NETLight High to next High
 *           Spec says this should be 864ms if there is a network
 *           and 3064 if there is not.  We call anything less than 1.28sec 
 *           a valid connection
 *           If there is no network, we are in this routine between 3-6 seconds
 * Note: Timer speed dependent
 * TestDate: Not tested as of 03-05-2015
 ********************************************************************/
int connectedToNetwork(void) //True when there is a network connection
{
  
    // This is function should only be called once we know the FONA is on.  
    // If the FONA is on, the NET light will blink so we should not get stuck here
    // waiting for 1's and 0's.  Just to be safe, leave if you wait too long for
    // the initial high or low
    
    // The timing in this routine assumes that Timer 1 is clocked at 15.625khz

    int success = 0;
    
    // Make sure you start at the beginning of the positive pulse
    TMR1 = 0;
    if (digitalPinStatus(netLightPin) == 1) //(PORTBbits.RB14 == 1)
    { // Wait until the light turns off
        while (digitalPinStatus(netLightPin)) {
            if(TMR1 > 2000){
                return success;   //waited longer than 128ms (high should be 64ms)
            }
        }; //(PORTBbits.RB14) {}; 
    }
    // Wait for rising edge
    TMR1 = 0;
    while ((digitalPinStatus(netLightPin) == 0)) {
         if(TMR1 > 55000){
                return success;   //waited longer than 3.5seconds (low should be 3sec when no network)
            }
    }; //PORTBbits.RB14 == 0) {}; 
    // no need to exit if it takes too long to get a high or low, if we are here, the light is flashing
    TMR1 = 0;  // Get time at start of positive pulse
    // Wait for the pulse to go low
    while (digitalPinStatus(netLightPin)) {
    }; 
    // Wait for the pulse to go high again
    while (digitalPinStatus(netLightPin) == 0) {
    }; 
    if(TMR1 > 20000){ // still looking for network pulsing should be 864ms, we allow up to 1.28sec
        success = 1;  
    }
    
    return success;  // True, when there is a network connection. (pulses slower than 1.28sec)
                     // spec says connection flashes every 864ms and no connection is every 3064ms.
}
void sendDebugMessage(char message[50], float value){
    if(print_debug_messages >= 1){
        char debugMsg[150];
        char debugValueString[20];
        debugMsg[0] = 0;
        concat(debugMsg, message);
        floatToString(value, debugValueString); 
        concat(debugMsg,debugValueString);
        concat(debugMsg, "\n");
        sendMessage(debugMsg);
    }
}
/*********************************************************************
 * Function: sendMessage()
 * Input: String
 * Output: None
 * Overview: Transmits the given characters along serial lines
 * Note: Library, Pic Dependent, sendTextMessage() uses this
 * TestDate: 06-02-2014
 * Note:  4/23/2017.  change this so to use a WHILE loop waiting for UTXBF but 
 *                    put a secondary check using one of the timers so we don't 
 *                    hang if there is a problem with the UART comms
 * 
 *                    Is there any way that the SIM would not get our message?
 *                    how do we know that everything was received since it does 
 *                    not ACK/NACK
 ********************************************************************/
void sendMessage(char message[160]) {
    int stringIndex = 0;
    int delayIndex;
   
    U1STAbits.UTXEN = 1; //enable transmit
    while (stringIndex < stringLength(message)) { // Tom - while not equal null
        if (U1STAbits.UTXBF == 0) {
             U1TXREG = message[stringIndex];
            stringIndex++;
            for (delayIndex = 0; delayIndex < 1000; delayIndex++) {
            }
        } else {
            for (delayIndex = 0; delayIndex < 30000; delayIndex++) { // proabably way longer than we need
            }
        }
    }
}
/*********************************************************************
 * Function: wasMessageSent
 * Input: msgNum - integer from 1 - 30 indicating which message status to check
 * Output: 0 if the last SMS message is still in the FONA waiting to be sent
 *               this is indicated by it being marked as STO UNSENT
 *         1 if the last SMS message was sent by the FONA
 *               this is indicated by it being marked as STO SENT
 * Overview: Reads the contents of message location #1 this is where all 
 *           sent messages are expected to be.  If the message is marked as STO UNSENT
 *           return a 0
 *           If it is marked as STO SENT, clear location #1 and return a 1
 * TestDate: Not Tested
 * Note:  Being Written
 ********************************************************************/
int wasMessageSent(int msgNum){
    int message_sent = 0;
    readSMSMessage(msgNum);
 
    char CmdMatch[]="STO SENT";
    if(strcmp(CmdMatch, FONAmsgStatus)==0){
        //the message was sent
        message_sent = 1;
    }
    else{
        //the message was not sent
    }
        
    
    return message_sent;    
}
/*********************************************************************
 * Function: readMessage()
 * Input: integer between 1-30 indicating which message to read
 * Output: None - the global array ReceiveTextMsg should have the message string in it.
 * Overview: Reads a single text message from the FONA and puts it into 
 *           the string ReceiveTextMsg
 *           ReceiveTextMsgFlag = 1 when a complete message has been received
 * Note: Library, Pic Dependent
 * TestDate: Not Tested
 * Note:  Not yet written
 ********************************************************************/
void readSMSMessage(int msgNum) {
    
    IFS0bits.U1RXIF = 0; // Always reset the interrupt flag
    U1STAbits.OERR = 0;  //clear the overrun error bit to allow new messages to be put in the RXREG FIFO
                         // This clears the RXREG FIFO
    IEC0bits.U1RXIE = 1;  // enable Rx interrupts

    // AT+CMGF=1  //set the mode to text
    NumCharInTextMsg = 0; //Point to the start of the Text Message String
    ReceiveTextMsgFlag = 0; //clear for the next message
    
    sendMessage("AT+CMGF=1\r\n"); //sets to text mode
    while(ReceiveTextMsgFlag<1){  } // Read the echo from the FONA
    ReceiveTextMsgFlag = 0; //clear for the next message
    while(ReceiveTextMsgFlag<1){  } // Read the OK from the FONA
           
    // Send the command to the FONA to read a text message
    // AT+CMGR=1
    
    IFS0bits.U1RXIF = 0; // Always reset the interrupt flag
    U1STAbits.OERR = 0;  //clear the overrun error bit to allow new messages to be put in the RXREG FIFO
                         // This clears the RXREG FIFO
    NumCharInTextMsg = 0; // Point to the start of the Text Message String
    ReceiveTextMsgFlag = 0; //clear for the next message
  // Debug  sendMessage("AT+CPMS=\"SM\"\r\n");
    
    char localMsg[160];
    localMsg[0] = 0;
    char msg_val[3];
    itoa(msg_val, msgNum, 10);
    concat(localMsg,"AT+CMGR=");
    concat(localMsg,msg_val);
    concat(localMsg,"\r\n");   
    sendMessage(localMsg); //Read message at index msgNum
    while(ReceiveTextMsgFlag<1){  } // Read the command echo from the FONA

    
    // There is about 17ms between the end of the echo of the command until 
    // The FONA responds with what you asked for
    // First we will get information about the message followed by a CR
    ReceiveTextMsgFlag = 0; //clear for the next message
    while(ReceiveTextMsgFlag<1){  } // Read the first line from the FONA
    
    // Here is where I'd like to read the phone number that sent the message
    // and the status of the message
    //command echo then +CMGR: "REC READ","+85291234567",,"
    char *MsgPtr;
    int msgLength=strlen(ReceiveTextMsg);
    FONAmsgStatus[0]=0;  //Reset the Fona Message Status array
    MsgPtr = ReceiveTextMsg+7;// Skip over the " in the echo of the original command
    while((*MsgPtr != '\"')&&(MsgPtr < ReceiveTextMsg+msgLength-1)){
        MsgPtr++;
    }
    MsgPtr++;
    while((*MsgPtr !='\"')&&(MsgPtr < ReceiveTextMsg+msgLength-1)){
        //strncpy(FONAmsgStatus, MsgPtr,1);
        strncat(FONAmsgStatus, MsgPtr,1);
        MsgPtr++;
    }
   
   // MsgPtr = ReceiveTextMsg+14; //skip over the + at the start
    while((*MsgPtr != '+')&&(MsgPtr < ReceiveTextMsg+msgLength-1)){
        MsgPtr++;
    }
    strncpy(SendingPhoneNumber,MsgPtr,12);        
    NumCharInTextMsg = 0; //Point to the start of the Text Message String
    ReceiveTextMsgFlag = 0; //clear for the next message
    // Then the message itself is received.  
    while(ReceiveTextMsgFlag<1){  } // Read the second line from the FONA
    // The ReceiveTextMsg array should now have the message
    IEC0bits.U1RXIE = 0;  // disable Rx interrupts
}

/*********************************************************************
 * Function: interpretSMSmessage()
 * Input: None
 * Output: None
 * Overview: Parses the ReceiveTextMsg character array 
 *           Depending upon the message, different actions are taken.
 * Currently Understood Messages
 *      AW_T indicates a time to use to update the RTCC.
 *              AW_T:sec,min,hr,wkday,date,month,year
 * Note: Library
 * TestDate: no tested
 ********************************************************************/
void interpretSMSmessage(void){
    int success = 0;
    char MsgPart[3];
    char CmdMatch[]="AW_T";
    if(strncmp(CmdMatch, ReceiveTextMsg,4)==0){
        strncpy(MsgPart,ReceiveTextMsg+11,2);
        char newhr = atoi(MsgPart); // does it work to convert the 2 string characters to a single decimal value
        setTime(0,45,newhr,5,3,11,17);
        hour = BcdToDec(getHourI2C());
        
        // Now we want to reply to the sender telling it what we just did
        
            // Send off the data

        
        success = turnOnSIM();  // returns 1 if the SIM powered up)
        sendDebugMessage("   \n Turning on the SIM was a ", success);  //Debug
        if(success == 1){ 
       // Try to establish network connection
            success = tryToConnectToNetwork();  // if we fail to connect, don't send the message
            sendDebugMessage("   \n Connect to network was a ", success);  //Debug
            if(success == 1){
            // Send off the data
                phoneNumber = SendingPhoneNumber;
                // Need to make dataMessage
                char localMsg[160];
                localMsg[0] = 0;
                char hour_val[3];
                itoa(hour_val, hour, 10);
                concat(localMsg,"Changed hour to ");
                concat(localMsg, hour_val);
                sendTextMessage(localMsg); 
                phoneNumber = MainphoneNumber;            
            }
        }
        
        
    }    
}
/*********************************************************************
 * Function: sendDebugTextMessage()
 * Input: String
 * Output: None
 * Overview: sends a Text Message to which ever phone number is in the variable 'DebugphoneNumber'
 *           we expect to be in this routine for 15.5sec, however, each character
 *           of each message takes some time that has not yet been calculated
 * Note: Library
 * TestDate: 01-12-2017
 ********************************************************************/
void sendDebugTextMessage(char message[160]) 
{
 //   turnOnSIM();
    delayMs(10000);
    sendMessage("AT+CMGF=1\r\n"); //sets to text mode
    delayMs(250);
    sendMessage("AT+CMGS=\""); //beginning of allowing us to send SMS message
    sendMessage(DebugphoneNumber);
    sendMessage("\"\r\n"); //middle of allowing us to send SMS message
    delayMs(250);
    sendMessage(message);
    delayMs(250);
    sendMessage("\x1A"); // method 2: sending hexidecimal representation
    // of 26 to sendMessage function (line 62)
    // & the end of allowing us to send SMS message
    delayMs(5000); // Give it some time to send the message
 //   turnOffSIM();
}

/*********************************************************************
 * Function: void ClearReceiveTextMessages(int MsgNum, int ClrMode);
 * Inputs: 
 *  MsgNum -  There are up to 30 messages saved on the SIM, specify 1-30
 *  ClrMode - 5 different ways to clear messages
 *  0 = Delete only the SMS message stored at the location MsgNum from the message storage area. 
 *  1 = Ignore the value of MsgNum and delete all SMS messages whose status is 
 *      "received read" from the message storage area.
 *  2 = Ignore the value of MsgNum and delete all SMS messages whose status is 
 *      "received read" or "stored sent" from the message storage area.
 *  3 = Ignore the value of MsgNum and delete all SMS messages whose status is 
 *      "received read", "stored unsent" or "stored sent" from the message storage area.
 *  4 = Ignore the value of MsgNum and delete all SMS messages from the message storage area.
 * 
 * Output: None
 * Overview: sends the command to the FONA board which clears its buffer of 
 *           messages.  This includes messages that have not yet been read
 * Note: Library
 * TestDate: not yet tested
 ********************************************************************/
void ClearReceiveTextMessages(int MsgNum, int ClrMode) 
{
    char MsgNumString[20];
    char ClrModeString[20];
    char MessageString[20];
    longToString(MsgNum, MsgNumString);
    longToString(ClrMode, ClrModeString);
    //AT+CMGF=1  //set the mode to text
    sendMessage("AT+CMGF=1\r\n"); //sets to text mode
    delayMs(250);  // Delay while the FONA replies OK
    //AT+CPMS="SM" //Specifies that we are working with the message storage on the SIM card
    sendMessage("AT+CPMS=\"SM\"\r\n"); 
     delayMs(250);  // Delay while the FONA replies with the number of messages already in storage
    // AT+CMGD=MsgNum,ClrMode  This is the delete command 
         
    concat(MessageString, "AT+CMGD=");
    concat(MessageString, MsgNumString);
    concat(MessageString, ",");
    concat(MessageString, ClrModeString);
    concat(MessageString, "\r\n");
    sendMessage(MessageString); 
     delayMs(250);// Delay while the FONA replies OK
}
/*********************************************************************
 * Function: sendTextMessage()
 * Input: String
 * Output: None
 * Overview: sends a Text Message to which ever phone number is in the variable 'phoneNumber'
 *           we expect to be in this routine for 10.5sec, however, each character
 *           of each message takes some time that has not yet been calculated
 * Note: Library
 * TestDate: 06-02-2014
 ********************************************************************/
void sendTextMessage(char message[160]) // Tested 06-02-2014
{
 //   turnOnSIM();
    // delayMs(10000);  FONA is already ON so no need to wait
    sendMessage("AT+CMGF=1\r\n"); //sets to text mode
    delayMs(250);
    sendMessage("AT+CMGS=\""); //beginning of allowing us to send SMS message
    sendMessage(phoneNumber);
    sendMessage("\"\r\n"); //middle of allowing us to send SMS message
    delayMs(250);
    sendMessage(message);
    delayMs(250);
    sendMessage("\x1A"); // method 2: sending hexidecimal representation
    // of 26 to sendMessage function (line 62)
    // & the end of allowing us to send SMS message
    //
    // we don't turn off the SIM so no need to delay to give it some time to send the message
 //   turnOffSIM();
}



void hourMessage(void) {
    //Message assembly and sending; Use *floatToString() to send:
    // Create storage for the various values to report

    char longestPrimeString[20];
    longestPrimeString[0] = 0;
    char leakRateLongString[20];
    leakRateLongString[0] = 0;
    char batteryFloatString[20];
    batteryFloatString[0] = 0;
    char volume02String[20];
    volume02String[0] = 0;
    char volume24String[20];
    volume24String[0] = 0;
    char volume46String[20];
    volume46String[0] = 0;
    char volume68String[20];
    volume68String[0] = 0;
    char volume810String[20];
    volume810String[0] = 0;
    char volume1012String[20];
    volume1012String[0] = 0;
    char volume1214String[20];
    volume1214String[0] = 0;
    char volume1416String[20];
    volume1416String[0] = 0;
    char volume1618String[20];
    volume1618String[0] = 0;
    char volume1820String[20];
    volume1820String[0] = 0;
    char volume2022String[20];
    volume2022String[0] = 0;
    char volume2224String[20];
    volume2224String[0] = 0;
    // Read values from EEPROM and convert them to strings
    EEProm_Read_Float(0, &leakRateLong);
    floatToString(leakRateLong, leakRateLongString);
    EEProm_Read_Float(1, &longestPrime);
    floatToString(longestPrime, longestPrimeString);
    
    floatToString(batteryFloat, batteryFloatString); //latest battery voltage
    
    EEProm_Read_Float(2, &volume02);  // Read yesterday saved 0-2AM volume, convert to string
    floatToString(volume02, volume02String);
    EEProm_Read_Float(3, &volume24);  // Read yesterday saved 2-4AM volume, convert to string
    floatToString(volume24, volume24String);
    EEProm_Read_Float(4, &volume46);  // Read yesterday saved 4-6AM volume, convert to string
    floatToString(volume46, volume46String);    
    EEProm_Read_Float(5, &volume68);  // Read yesterday saved 6-8AM volume, convert to string
    floatToString(volume68, volume68String);    
    EEProm_Read_Float(6, &volume810);  // Read yesterday saved 8-10AM volume, convert to string
    floatToString(volume810, volume810String);    
    EEProm_Read_Float(7, &volume1012);  // Read yesterday saved 10-12AM volume, convert to string
    floatToString(volume1012, volume1012String);   
    EEProm_Read_Float(8, &volume1214);  // Read yesterday saved 12-14PM volume, convert to string
    floatToString(volume1214, volume1214String);    
    EEProm_Read_Float(9, &volume1416);  // Read yesterday saved 14-16PM volume, convert to string
    floatToString(volume1416, volume1416String);    
    EEProm_Read_Float(10, &volume1618);  // Read yesterday saved 16-18PM volume, convert to string
    floatToString(volume1618, volume1618String);    
    EEProm_Read_Float(11, &volume1820);  // Read yesterday saved 18-20PM volume, convert to string
    floatToString(volume1820, volume1820String);    
    EEProm_Read_Float(12, &volume2022);  // Read yesterday saved 20-22PM volume, convert to string
    floatToString(volume2022, volume2022String);    
    EEProm_Read_Float(13, &volume2224);  // Read yesterday saved 22-24PM volume, convert to string
    floatToString(volume2224, volume2224String);
    
 //   long checkSum = longestPrime + leakRateLong + volume02 + volume24 + volume46 + volume68 + volume810 + volume1012 + volume1214 + volume1416 + volume1618 + volume1820 + volume2022 + volume2224;
 //   char stringCheckSum[20];
 //   floatToString(checkSum, stringCheckSum);
    
    
    // Clear saved leakRateLong and longestPrime
 //   leakRateLong = 0; //Clear local and saved value 
 //   EEProm_Write_Float(0,&leakRateLong); 
 //   longestPrime = 0;//Clear local and saved value
 //   EEProm_Write_Float(1,&longestPrime);
 
    // Move today's 0-12AM values into the yesterday positions
    // There is no need to relocate data from 12-24PM since it has not yet been measured
 //   EEProm_Read_Float(14, &volume02); // Overwrite saved volume with today's value
 //   EEProm_Write_Float(2,&volume02);
 //   EEProm_Read_Float(15, &volume24); // Overwrite saved volume with today's value
 //   EEProm_Write_Float(3,&volume24);
 //   EEProm_Read_Float(16, &volume46); // Overwrite saved volume with today's value
 //   EEProm_Write_Float(4,&volume46);
 //   EEProm_Read_Float(17, &volume68); // Overwrite saved volume with today's value
 //   EEProm_Write_Float(5,&volume68);
 //   EEProm_Read_Float(18, &volume810); // Overwrite saved volume with today's value
 //   EEProm_Write_Float(6,&volume810);
 //   EEProm_Read_Float(19, &volume1012); // Overwrite saved volume with today's value
 //   EEProm_Write_Float(7,&volume1012);
 
    //Clear slots for volume 1214-2224 to make sure they are zero in case there is no power to fill
 //   EEFloatData = 0.01;
 //   EEProm_Write_Float(8, &EEFloatData);
 //   EEProm_Write_Float(9, &EEFloatData);
 //   EEProm_Write_Float(10, &EEFloatData);
 //   EEProm_Write_Float(11, &EEFloatData);
 //   EEProm_Write_Float(12, &EEFloatData);
 //   EEProm_Write_Float(13, &EEFloatData);
 //   EEProm_Write_Float(14, &EEFloatData);
 //   EEProm_Write_Float(15, &EEFloatData);
 //   EEProm_Write_Float(16, &EEFloatData);
 //   EEProm_Write_Float(17, &EEFloatData);
 //   EEProm_Write_Float(18, &EEFloatData);
 //   EEProm_Write_Float(19, &EEFloatData);
    
    
    //will need more formating for JSON 5-30-2014
    char dataMessage[160];
    dataMessage[0] = 0;
    concat(dataMessage, "(\"t\":\"B\",\"d\":(\"l\":");
    concat(dataMessage, leakRateLongString);
    concat(dataMessage, ",\"p\":");
    concat(dataMessage, longestPrimeString);
    concat(dataMessage, ",\"b\":");
    concat(dataMessage, batteryFloatString);
    if (depthSensorInUse == 1) { // if you have a depth sensor
        pinDirectionIO(depthSensorOnOffPin, 0); //makes depth sensor pin an output.
        digitalPinSet(depthSensorOnOffPin, 1); //turns on the depth sensor.
        delayMs(30000); // Wait 30 seconds for the depth sensor to power up
        char maxDepthLevelString[20];
        maxDepthLevelString[0] = 0;
        char minDepthLevelString[20];
        minDepthLevelString[0] = 0;
        float currentDepth = readDepthSensor();
        if (midDayDepth > currentDepth) {
            floatToString(midDayDepth, maxDepthLevelString);
            floatToString(currentDepth, minDepthLevelString);
        } else {
            floatToString(currentDepth, maxDepthLevelString);
            floatToString(midDayDepth, minDepthLevelString);

        }
        concat(dataMessage, ",\"d\":<");
        concat(dataMessage, maxDepthLevelString);
        concat(dataMessage, ",");
        concat(dataMessage, minDepthLevelString);
        concat(dataMessage, ">");

        digitalPinSet(depthSensorOnOffPin, 0); //turns off the depth sensor.
    }
    concat(dataMessage, ",\"v\":<");
    concat(dataMessage, volume02String);
    concat(dataMessage, ",");
    concat(dataMessage, volume24String);
    concat(dataMessage, ",");
    concat(dataMessage, volume46String);
    concat(dataMessage, ",");
    concat(dataMessage, volume68String);
    concat(dataMessage, ",");
    concat(dataMessage, volume810String);
    concat(dataMessage, ",");
    concat(dataMessage, volume1012String);
    concat(dataMessage, ",");
    concat(dataMessage, volume1214String);
    concat(dataMessage, ",");
    concat(dataMessage, volume1416String);
    concat(dataMessage, ",");
    concat(dataMessage, volume1618String);
    concat(dataMessage, ",");
    concat(dataMessage, volume1820String);
    concat(dataMessage, ",");
    concat(dataMessage, volume2022String);
    concat(dataMessage, ",");
    concat(dataMessage, volume2224String);
    concat(dataMessage, ">))");

    turnOnSIM();  
    // Try to establish network connection
    tryToConnectToNetwork();
    delayMs(2000);
    // Send off the data
    sendDebugTextMessage(dataMessage);
    // sendMessage(dataMessage);
    //sendMessage(" \r \n");

    //        prevHour = getHourI2C();
    //        prevDay = getDateI2C();
    // pressReset();
    ////////////////////////////////////////////////
    // Should we put the SIM back to sleep here?
    ////////////////////////////////////////////////
    RTCCSet(); // updates the internal time from the external RTCC if the internal RTCC got off any through out the day

}




/////////////// IN PROCESS //////////////
int noonMessage(void) {
    
    //Message assembly and sending; Use *floatToString() to send:
    // Create storage for the various values to report
    int success = 0;  // variable used to see if various FONA operations worked
                      // which means we either did (1) or did not (0) send the message
    char longestPrimeString[20];
    longestPrimeString[0] = 0;
    char leakRateLongString[20];
    leakRateLongString[0] = 0;
    char batteryFloatString[20];
    batteryFloatString[0] = 0;
    char volume02String[20];
    volume02String[0] = 0;
    char volume24String[20];
    volume24String[0] = 0;
    char volume46String[20];
    volume46String[0] = 0;
    char volume68String[20];
    volume68String[0] = 0;
    char volume810String[20];
    volume810String[0] = 0;
    char volume1012String[20];
    volume1012String[0] = 0;
    char volume1214String[20];
    volume1214String[0] = 0;
    char volume1416String[20];
    volume1416String[0] = 0;
    char volume1618String[20];
    volume1618String[0] = 0;
    char volume1820String[20];
    volume1820String[0] = 0;
    char volume2022String[20];
    volume2022String[0] = 0;
    char volume2224String[20];
    volume2224String[0] = 0;
    // ///////////// Debug
    char debugString[20];
    debugString[0]=0;
    floatToString(debugCounter,debugString);
    /////////////// Debug
    // Read values from EEPROM and convert them to strings
    EEProm_Read_Float(0, &EEFloatData);
    floatToString(EEFloatData, leakRateLongString);
    EEProm_Read_Float(1, &EEFloatData);
    floatToString(EEFloatData, longestPrimeString);
    
    floatToString(batteryFloat, batteryFloatString); //latest battery voltage
    
    EEProm_Read_Float(2, &EEFloatData);  // Read yesterday saved 0-2AM volume, convert to string
    floatToString(EEFloatData, volume02String);
    EEProm_Read_Float(3, &EEFloatData);  // Read yesterday saved 2-4AM volume, convert to string
    floatToString(EEFloatData, volume24String);
    EEProm_Read_Float(4, &EEFloatData);  // Read yesterday saved 4-6AM volume, convert to string
    floatToString(EEFloatData, volume46String);    
    EEProm_Read_Float(5, &EEFloatData);  // Read yesterday saved 6-8AM volume, convert to string
    floatToString(EEFloatData, volume68String);    
    EEProm_Read_Float(6, &EEFloatData);  // Read yesterday saved 8-10AM volume, convert to string
    floatToString(EEFloatData, volume810String);    
    EEProm_Read_Float(7, &EEFloatData);  // Read yesterday saved 10-12AM volume, convert to string
    floatToString(EEFloatData, volume1012String);   
    EEProm_Read_Float(8, &EEFloatData);  // Read yesterday saved 12-14PM volume, convert to string
    floatToString(EEFloatData, volume1214String);    
    EEProm_Read_Float(9, &EEFloatData);  // Read yesterday saved 14-16PM volume, convert to string
    floatToString(EEFloatData, volume1416String);    
    EEProm_Read_Float(10, &EEFloatData);  // Read yesterday saved 16-18PM volume, convert to string
    floatToString(EEFloatData, volume1618String);    
    EEProm_Read_Float(11, &EEFloatData);  // Read yesterday saved 18-20PM volume, convert to string
    floatToString(EEFloatData, volume1820String);    
    EEProm_Read_Float(12, &EEFloatData);  // Read yesterday saved 20-22PM volume, convert to string
    floatToString(EEFloatData, volume2022String);    
    EEProm_Read_Float(13, &EEFloatData);  // Read yesterday saved 22-24PM volume, convert to string
    floatToString(EEFloatData, volume2224String);
    
  //  long checkSum = longestPrime + leakRateLong + volume02 + volume24 + volume46 + volume68 + volume810 + volume1012 + volume1214 + volume1416 + volume1618 + volume1820 + volume2022 + volume2224;
  //  char stringCheckSum[20];
  //  floatToString(checkSum, stringCheckSum);
    
        //will need more formating for JSON 5-30-2014
    char dataMessage[160];
    dataMessage[0] = 0;
  // Debug for Scott  if(hour != 12){
      if(hour == 120){
      concat(dataMessage, "(\"t\":");
      concat(dataMessage,debugString);
      concat(dataMessage,",\"d\",\"d\":(\"l\":");
    }
    else{
        concat(dataMessage, "(\"t\":\"d\",\"d\":(\"l\":");
    }
    
    concat(dataMessage, leakRateLongString);
    concat(dataMessage, ",\"p\":");
    concat(dataMessage, longestPrimeString);
    concat(dataMessage, ",\"b\":");
    concat(dataMessage, batteryFloatString);
    if (depthSensorInUse == 1) { // if you have a depth sensor
        pinDirectionIO(depthSensorOnOffPin, 0); //makes depth sensor pin an output.
        digitalPinSet(depthSensorOnOffPin, 1); //turns on the depth sensor.
        delayMs(30000); // Wait 30 seconds for the depth sensor to power up
        char maxDepthLevelString[20];
        maxDepthLevelString[0] = 0;
        char minDepthLevelString[20];
        minDepthLevelString[0] = 0;
        float currentDepth = readDepthSensor();
        if (midDayDepth > currentDepth) {
            floatToString(midDayDepth, maxDepthLevelString);
            floatToString(currentDepth, minDepthLevelString);
        } else {
            floatToString(currentDepth, maxDepthLevelString);
            floatToString(midDayDepth, minDepthLevelString);

        }
        concat(dataMessage, ",\"d\":<");
        concat(dataMessage, maxDepthLevelString);
        concat(dataMessage, ",");
        concat(dataMessage, minDepthLevelString);
        concat(dataMessage, ">");

        digitalPinSet(depthSensorOnOffPin, 0); //turns off the depth sensor.
    }
    concat(dataMessage, ",\"v\":<");
    concat(dataMessage, volume02String);
    concat(dataMessage, ",");
    concat(dataMessage, volume24String);
    concat(dataMessage, ",");
    concat(dataMessage, volume46String);
    concat(dataMessage, ",");
    concat(dataMessage, volume68String);
    concat(dataMessage, ",");
    concat(dataMessage, volume810String);
    concat(dataMessage, ",");
    concat(dataMessage, volume1012String);
    concat(dataMessage, ",");
    concat(dataMessage, volume1214String);
    concat(dataMessage, ",");
    concat(dataMessage, volume1416String);
    concat(dataMessage, ",");
    concat(dataMessage, volume1618String);
    concat(dataMessage, ",");
    concat(dataMessage, volume1820String);
    concat(dataMessage, ",");
    concat(dataMessage, volume2022String);
    concat(dataMessage, ",");
    concat(dataMessage, volume2224String);
    concat(dataMessage, ">))");

    success = turnOnSIM();  // returns 1 if the SIM powered up)
    sendDebugMessage("   \n Turning on the SIM was a ", success);  //Debug
    if(success == 1){ 
       // Try to establish network connection
        success = tryToConnectToNetwork();  // if we fail to connect, don't send the message
        sendDebugMessage("   \n Connect to network was a ", success);  //Debug
        if(success == 1){
        // Send off the data
            phoneNumber = MainphoneNumber;  
            sendTextMessage(dataMessage);              
        // Now that the message has been sent, we can update our EEPROM
        // Clear RAM and EEPROM associated with message variables
            if(hour == 12){
                ResetMsgVariables();
            }
        }
    }
    
    return success;  // this will be a 1 if we were able to connect to the network.  We assume that we sent the message
   
  
    ////////////////////////////////////////////////
    // Should we put the SIM back to sleep here?
    ////////////////////////////////////////////////
    
    
    
 // Taken out 4/24/17 RKF   RTCCSet(); // updates the internal time from the external RTCC if the internal RTCC got off any through out the day
               // RKF QUESTION - Why do we do this?  I don't think we use the internal RTCC for anything

}


int diagnosticMessage(void) {
    
    //Message assembly and sending; Use *floatToString() to send:
    // Create storage for the various values to report
    int success = 0;  // variable used to see if various FONA operations worked
                      // which means we either did (1) or did not (0) send the message
    char sleepHrStatusString[20];
    sleepHrStatusString[0] = 0;
    char batteryFloatString[20];
    batteryFloatString[0] = 0;
    char timeSinceLastRestartString[20];
    timeSinceLastRestartString[0] = 0;
    char extRtccTalkedString[20];
    extRtccTalkedString[0] = 0;
    
    // Read values from EEPROM and convert them to strings
    EEProm_Read_Float(21, &EEFloatData);
    floatToString(EEFloatData, sleepHrStatusString); //populates the sleepHrStatusString with the value from EEPROM
    
    floatToString(batteryFloat, batteryFloatString); //latest battery voltage
    floatToString(timeSinceLastRestart, timeSinceLastRestartString);
    floatToString(extRtccTalked, extRtccTalkedString);
    
        //will need more formating for JSON 5-30-2014
    char dataMessage[160];
    dataMessage[0] = 0;

    concat(dataMessage, "(\"t\":\"d\",\"d\":(\"s\":");
    concat(dataMessage, sleepHrStatusString);
    concat(dataMessage, ",\"b\":");
    concat(dataMessage, batteryFloatString);
    concat(dataMessage, ",\"r\":");
    concat(dataMessage, timeSinceLastRestartString);
    concat(dataMessage, ",\"c\":");
    concat(dataMessage, extRtccTalkedString);
    

    concat(dataMessage, ">))");

    success = turnOnSIM();  // returns 1 if the SIM powered up)
    sendDebugMessage("   \n Turning on the SIM was a ", success);  //Debug
    if(success == 1){ 
       // Try to establish network connection
        success = tryToConnectToNetwork();  // if we fail to connect, don't send the message
        sendDebugMessage("   \n Connect to network was a ", success);  //Debug
        if(success == 1){
        // Send off the data
            sendTextMessage(dataMessage);              
        // Now that the message has been sent, we can update our EEPROM
        // Clear RAM and EEPROM associated with message variables
            if(hour == 12){
                ResetMsgVariables();
            }
        }
    }
    return success;  // this will be a 1 if we were able to connect to the network.  We assume that we sent the message
}   