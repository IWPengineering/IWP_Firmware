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
//char DebugphoneNumber[] = "+17177784498"; // Upside 

char DebugphoneNumber[] = "+18458007595"; //Number for Paul Zwart cell phone
//char MainphoneNumber[]="+17177784498"; // Upside Wireless
char MainphoneNumber[]="+17176837803"; // Randy
char SendingPhoneNumber[]="+17177784498"; //this is read from the received SMS message default to mine
//char phoneNumber[] = "+17177784498"; // Number Used to send text message report (daily or hourly)
//char phoneNumber []="+17176837803"; // Randy
char* phoneNumber = MainphoneNumber; // Randy
int LeaveOnSIM = 0;  // this is set to 1 when an external message says to not shut off the SIM
char FONAmsgStatus[11]; //Message REC/STO, UNREAD/READ, UNSENT/SENT
char ReceiveTextMsg[160]; //This is the string used to buffer up a message read from the FONA
char SMSMessage[160]; //A string used to hold all SMS message sent with FONA
int NumCharInTextMsg = 0; //Keeps track of the number of characters in the received text string
char ReceiveTextMsgFlag = 0; //Set to 1 when a complete text message has been received
int num_unsent_daily_reports = 0; //this is the number of saved daily reports that have not been sent


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
 * TestDate: 12-22-2017 RKF
 ********************************************************************/
int turnOffSIM() {
    int SIM_OFF = 0;  // Assume the SIM is not off
    if(LeaveOnSIM == 0){ 
        digitalPinSet(simVioPin, 1); //PORTAbits.RA1 = 1; //Tells Fona what logic level to use for UART
        if (digitalPinStatus(statusPin) == 1) { //Checks see if the Fona is ON 1=Yes so turn it off
            digitalPinSet(pwrKeyPin, 0); //PORTBbits.RB6 = 0; //set low pin 15 for 2000ms to turn OFF Fona
            delayMs(2000);
        }
        digitalPinSet(pwrKeyPin, 1); //PORTBbits.RB6 = 1; // Reset the Power Key so it can be turned on later (pin 15)
        // Experiments show the FONA shutting off 7ms BEFORE the KEY is brought back high
        //    Still wait 100ms before checking.
        delayMs(100);

        if (digitalPinStatus(statusPin) == 0) { //Checks see if the Fona is OFF 0 = OFF so don't do anything
            SIM_OFF = 1;
        }
    }
    return SIM_OFF;//	while (digitalPinStatus(statusPin) == 1){ //Checks see if the Fona is on pin
 }

/*********************************************************************
 * Function: turnOnSIM
 * Input: None
 * Output: SIM_ON  this is a 1 if the SIM turned on and 0 if not
 * Overview: Turns on SIM900 - If the PS (Power Status) pin = 0 the SIM is Off
 *                             Strobe the KEY pin low for 2 sec and then go high
 *                            The SIM turns on (PS = 1) after approx. 0.8 - 1.3sec
 * 
 * Note: Pic Dependent
 * TestDate: Not tested as of 12-22-17 RKF
 * delayMs(int ms)
 ********************************************************************/
int turnOnSIM() {
    int SIM_ON = 0;  // Assume the SIM is not on
    digitalPinSet(simVioPin, 1); //PORTAbits.RA1 = 1; //Tells Fona what logic level to use for UART
    if (digitalPinStatus(statusPin) == 0) { //Checks see if the Fona is off pin
        digitalPinSet(pwrKeyPin, 0); //PORTBbits.RB6 = 0; //set low pin 15 for 2000ms to turn on Fona
        delayMs(2000);
    }
    digitalPinSet(pwrKeyPin, 1); //PORTBbits.RB6 = 1; // Reset the Power Key so it can be turned off later (pin 15)
    // Experimental tests showed that it takes 0.8 - 1.3sec for the FONA to turn on
    delayMs(2000);
    if (digitalPinStatus(statusPin) != 0) { //Checks see if the Fona is off pin
        SIM_ON = 1;
    }
    return SIM_ON;
}

/*********************************************************************
 * Function: tryToConnectToNetwork
 * Input: None
 * Output: None
 * Overview: This function tests for network status and attempts to connect to the
 *           network. A connection is there if we see the proper blinking of the
 *           network light 4 times in a row.
 *           If the 4 good consecutive connections are not there, pause for 
 *           1 second and try again up to 7 times. Looking for network with this
 *           approach will take between 20sec and 45sec.  If no network is found, 
 *           a zero is returned indicating that connection to the network failed
 * TestDate: 12/20/2017 RKF
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
        if (CheckNetworkConnection()) {
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
 * Function: CheckNetworkConnection
 * Input: None
 * Output: 1 if network connected 0 if not
 * Overview: Measures the time from NETLight High to next High
 *           Spec:     864ms if Network = NO (high = 64ms low = 800ms)
 *           Measured: 904ms if Network = NO (high = 56ms, low = 848ms)
 * 
 *           Spec:     3064ms if Network = YES (high = 64ms, low = 3000ms)
 *           Measured: 2840ms if Network = YES (high = 52ms, low = 2780ms)
 * 
 *           We call anything less than 1.28sec a valid connection???
 *           If there is no network, we are in this routine between 1.7-6.2 seconds
 * 
 *            * Note: Timer speed dependent
 * TestDate: Not tested as of 03-05-2015
 ********************************************************************/
int CheckNetworkConnection(void) //True when there is a network connection
{
  
    // This is function should only be called once we know the FONA is on.  
    // If the FONA is on, the NET light will blink so we should not get stuck here
    // waiting for 1's and 0's.  Just to be safe, leave if you wait too long for
    // the initial high or low
    
    // The timing in this routine assumes that Timer 1 is clocked at 15.625khz

    int network_status = 0;  // Assume there is no network connection
    
    // Make sure you start at the beginning of the positive pulse
    TMR1 = 0;
    if (digitalPinStatus(netLightPin) == 1) //(PORTBbits.RB14 == 1)
    { // Wait until the light turns off
        while (digitalPinStatus(netLightPin)) {
            if(TMR1 > 2000){
                return network_status;   //waited longer than 128ms (high should be 64ms)
                                  // if this happens, the light is stuck ON
            }
        }
    }
    // Wait for rising edge
    TMR1 = 0;
    while ((digitalPinStatus(netLightPin) == 0)) {
         if(TMR1 > 55000){
                return network_status;   //waited longer than 3.5seconds (low should be 3sec when no network)
                                  // if this happens, the light is stuck OFF
            }
    }
    // We are now right at the start of a cycle and can measure the time
    TMR1 = 0;  // Get time at start of positive pulse
    // Wait for the pulse to go low
    while (digitalPinStatus(netLightPin)) {
         if(TMR1 > 2000){
                return network_status;   //waited longer than 128ms (high should be 64ms)
                                  // if this happens, the light is stuck ON
            }
    } 
    // Wait for the pulse to go high again
    while (digitalPinStatus(netLightPin) == 0) {
        if(TMR1 > 55000){
                return network_status;   //waited longer than 3.5seconds (low should be 3sec when no network)
                                  // if this happens, the light is stuck OFF
            }
    } 
    if(TMR1 > 20000){ // No Network pulsing should be 864ms, If pulse is longer than 1.28 sec, Network = YES
        network_status = 1;  
    }
    
    return network_status;  // True, when there is a network connection. (pulses slower than 1.28sec)
                           
}
void sendDebugMessage(char message[50], float value){

    int SIMwasON = 0;
    if (digitalPinStatus(statusPin) == 1) { // if the Fona is on, turn it off
                                            // the FONA gets messed up if the serial line
                                            // is sending non AT commands when it is on
       turnOffSIM();
       SIMwasON = 1;
    }
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
    if(SIMwasON ==1){
        turnOnSIM(); // if the SIM was on when you got here, turn it back on
    }
}
/*********************************************************************
 * Function: sendMessage()
 * Input: String
 * Output: 1 if the string was sent and 0 if not
 * Overview: Transmits the characters in the string message[] using the
 *           UART.  BAUD rate is assumed to be 9600
 * Note: Library, Pic Dependent, sendTextMessage() uses this
 * TestDate: 1-26-2018
 ********************************************************************/
int sendMessage(char message[160]) {
    int stringIndex = 0;
    int success = 0;
   
    U1STAbits.UTXEN = 1; //enable transmit
    TMR1 = 0;  // transmitting 160char (10bits each) at 9600BAUD should take less than 170ms
               // Assuming a 15.625khz clock for timer 1, that is 2656 clock cycles
    while ((stringIndex < stringLength(message))&&(TMR1 < 2700)) {
        while((U1STAbits.UTXBF == 1)&&(TMR1 < 2700)){ //wait for the buffer to be ready for the next character
                                                      // but don't allow us to get hung if something is wrong
        }
        U1TXREG = message[stringIndex];
        stringIndex++;
    }
    if(stringIndex == stringLength(message)){
        success = 1;
    }
    return success;  // report a failure if we don't send the entire string
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
 * Output: None - two global arrays are modified by this function
 *                ReceiveTextMsg has the message string in it 
 *                SendingPhoneNumber has the phone number of the sender
 * Overview: Reads a single text message from the FONA and puts it into 
 *           the string ReceiveTextMsg
 *           ReceiveTextMsgFlag = 1 when a complete message has been received
 * Note: Library, Pic Dependent
 * TestDate: Not Tested
 ********************************************************************/
void readSMSMessage(int msgNum) {
    int NoMessageThere = 1;  // Assume that there is nothing to read
    
    IFS0bits.U1RXIF = 0; // Always reset the interrupt flag
    U1STAbits.OERR = 0;  //clear the overrun error bit to allow new messages to be put in the RXREG FIFO
                         // This clears the RXREG FIFO
    IEC0bits.U1RXIE = 1;  // enable Rx interrupts
    NumCharInTextMsg = 0; //Point to the start of the Text Message String
    ReceiveTextMsgFlag = 0; //clear for the next message
       // AT+CMGF=1  //set the mode to text 
    sendMessage("AT+CMGF=1\r\n"); //sets to text mode

    TMR1 = 0; // start timer for max 160characters
    while((ReceiveTextMsgFlag<1)&&(TMR1<200)){  } // Read the echo of the AT+CMGF=1 command from the FONA
    ReceiveTextMsgFlag = 0; //clear for the next message
    TMR1 = 0; // start timer for max 160characters
    while((ReceiveTextMsgFlag<1)&&(TMR1<200)){  } // Read the OK from the FONA
           
    // Send the command to the FONA to read a text message
    // AT+CMGR=1
    
    IFS0bits.U1RXIF = 0; // Always reset the interrupt flag
    U1STAbits.OERR = 0;  //clear the overrun error bit to allow new messages to be put in the RXREG FIFO
                         // This clears the RXREG FIFO
    NumCharInTextMsg = 0; // Point to the start of the Text Message String
    ReceiveTextMsgFlag = 0; //clear for the next message
    
    char localMsg[160];
    localMsg[0] = 0;
    char msg_val[3];
    itoa(msg_val, msgNum, 10);
    concat(localMsg,"AT+CMGR=");
    concat(localMsg,msg_val);
    concat(localMsg,"\r\n");   
    sendMessage(localMsg); //send command to Read message at index msgNum
    TMR1 = 0; // start timer for max 160characters
    while((ReceiveTextMsgFlag<1)&&(TMR1<200)){  } // Read the command echo from the FONA

    
    // There is about 17ms between the end of the echo of the command until 
    // The FONA responds with what you asked for
    // First we will get information about the message followed by a CR
    ReceiveTextMsgFlag = 0; //clear for the next message
    TMR1 = 0; // start timer for max 160characters
    while((ReceiveTextMsgFlag<1)&&(TMR1<200)){  } // Read the first line from the FONA
    
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
    TMR1 = 0; // start timer for max 160characters
    while((ReceiveTextMsgFlag<1)&&(TMR1<200)){  } // Read the second line from the FONA
    // The ReceiveTextMsg array should now have the message
    IEC0bits.U1RXIE = 0;  // disable Rx interrupts
}

/*********************************************************************
 * Function: interpretSMSmessage()
 * Input: None - must be called after the readSMSMessage
 *                ReceiveTextMsg has the message string in it 
 *                SendingPhoneNumber has the phone number of the sender
 * Output: None
 * Overview: Parses the ReceiveTextMsg character array 
 *           Depending upon the message, different actions are taken.
 * Currently Understood Messages
 *      AW_C indicates changes to the RTCC date,month,hour.
 *              AW_T:sec,min,hr,wkday,date,month,year
 *              AW_C:date,month,delta_hour  ie.  March 12th add 6hrs to hour AW_C:12,03,06
 * Note: Library
 * TestDate: no tested
 ********************************************************************/
void interpretSMSmessage(void){
    char CmdMatch[]="AW_C";
    if(strncmp(CmdMatch, ReceiveTextMsg,4)==0){
      updateClockCalendar();
    }    
}
/*********************************************************************
 * Function: updateClockCalendar()
 * Input: None - must be called after the readSMSMessage
 *                ReceiveTextMsg has the message string in it 
 *                SendingPhoneNumber has the phone number of the sender
 * Output: None
 * Overview: An SMS message was received (in the string ReceiveTextMsg)
 *           with the AW_C prefix indicating that
 *           the date, month or hour needs to be changed
 *              AW_C:date,month,delta_hour  
 *              AW_C:12,03,06  March 12th add 6hrs to hour 
 * 
 *           The cell phone number that sent the message is expected to already
 *           be in the string SendingPhoneNumber
 * Note: Library
 * TestDate: not tested
 ********************************************************************/
void updateClockCalendar(){
     char MsgPart[3];
     int success = 0;
     // Get the new date
     strncpy(MsgPart,ReceiveTextMsg+5,2);
     char newDate = atoi(MsgPart);      
     // Get the new month
     strncpy(MsgPart,ReceiveTextMsg+8,2);
     char newMonth = atoi(MsgPart);
     // Get the change to the hour
     strncpy(MsgPart,ReceiveTextMsg+11,2);
     char Delta_hour = atoi(MsgPart);
   
    // Update the settings for the external RTCC
    hour = Delta_hour + BcdToDec(getHourI2C());
    if(hour > 23){ // If we want to change 9AM to 7AM we will ask for a change of +22
        hour = hour - 24;
    }
    int year = BcdToDec(getYearI2C());
    int wkday = BcdToDec(getWkdayI2C());
    setTime(0,0,hour,wkday,newDate,newMonth,year);//   (sec, min, hr, wkday, date, month, year)
    
    // Update the settings for the internal RTCC
    setInternalRTCC(0, 0, hour, wkday, newDate, newMonth, year);
    internalHour = hour;
               
    // Now we want to reply to the sender telling it what we just did
    success = turnOnSIM();  // returns 1 if the SIM powered up)
    if(success == 1){
        // Try to establish network connection
        success = tryToConnectToNetwork();  // if we fail to connect, don't send the message
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
            sendTextMessage(localMsg);   //note, this now returns 1 if successfully sent to FONA
            phoneNumber = MainphoneNumber;
        }
    }
    turnOffSIM();
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
    MessageString[0]=0; //Clear the message string    
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
 * Output: 1 if the SMS message was sent to the FONA.  This does not mean it was actually transmitted
 * Overview: sends an SMS Text Message to which ever phone number is in the variable 'phoneNumber'
 *           
 * Note: Library
 * TestDate: Needs to be retested
 ********************************************************************/
int sendTextMessage(char message[160]) 
{
    int success = 0;
 //   The SIM should have been turned on prior to getting here;
    
    success = sendMessage("AT+CMGF=1\r\n"); //sets to text mode
    delayMs(250);
    success = sendMessage("AT+CMGS=\""); //beginning of allowing us to send SMS message
    success = sendMessage(phoneNumber);
    success = sendMessage("\"\r\n"); //middle of allowing us to send SMS message
    delayMs(250);
    success = sendMessage(message);
    delayMs(250);
    success = sendMessage("\x1A"); // this terminates an AT SMS command
    
    return success;
}

 
/*********************************************************************
 * Function: CreateNoonMessage(int)
 * Input: EEProm slot number for the start of data saved for this daily report
 * Output: None
 * Overview: Gathers the data needed for the daily report and puts it into a text string SMSMessage
 *           The maximum length of a SMS message is 160 characters
 *           battery limited to 5 characters
 *           priming, leak and volume amounts limited to 7 characters each
 *           if EEProm has a NaN, it is reported as 22222
 *           3 decimal places of accuracy maintained for values between 0.001 - 999.999
 *           For 1000 - 32000 we only report integers.  Maximum integer is 32000
 * Note: 
 * TestDate: Tested 3-13-08
 ********************************************************************/
/*
 * Notice that there is a " at the start but not the end of this string????
 * New format: ("t":"d","d":("l":0,"p":0,"b":3.968,"v":<0,0,0,0,0,0,0,0,0,0,0,0>),?c?:MMDDHH)
 *                    MM- Month and DD-Date are the date the information was collected
 *                    HH ? hour is the time that the system believes the message is being sent.
 */
void CreateNoonMessage(int effective_address){
    int vptr;
    char LocalString[20];  
    SMSMessage[0] = 0; //reset SMS message array to be empty
    LocalString[0] = 0;
    
    concat(SMSMessage, "(\"t\":\"d\",\"d\":(\"l\":");
    EEProm_Read_Float(effective_address, &EEFloatData); // Get Longest Leak Rate
    // Limit report to 7 characters within integer range
    if((EEFloatData == 0)||(EEFloatData < 0)||(EEFloatData > 0)){// check for NaN
        if(EEFloatData>32000){ // maximum positive integer is 32767 (7FFF)
            EEFloatData = 32000;
        }
        if(EEFloatData>999.999){
            EEFloatData = (int)EEFloatData; //truncates to integer
        }
    }
    else{
        EEFloatData = 22222; // this will be understood to be NaN
    }
    floatToString(EEFloatData, LocalString);
    concat(SMSMessage, LocalString);
    concat(SMSMessage, ",\"p\":");
    EEProm_Read_Float(effective_address+1, &EEFloatData); // Get Longest Prime
    // Limit report to 7 characters within integer range
    if((EEFloatData == 0)||(EEFloatData < 0)||(EEFloatData > 0)){// check for NaN
        if(EEFloatData>32000){ // maximum positive integer is 32767 (7FFF)
            EEFloatData = 32000;
        }
        if(EEFloatData>999.999){
            EEFloatData = (int)EEFloatData; //truncates to integer
        }
    }
    else{
        EEFloatData = 22222; // this will be understood to be NaN
    }
    
    floatToString(EEFloatData, LocalString);
    concat(SMSMessage, LocalString);
    concat(SMSMessage, ",\"b\":");
    EEProm_Read_Float(effective_address+2, &EEFloatData); // Get battery voltage
    // Limit report to 7 characters within integer range
    if((EEFloatData == 0)||(EEFloatData < 0)||(EEFloatData > 0)){// check for NaN
        // Limit report to 5 characters
        if(EEFloatData>9.999){
            EEFloatData = 9.999;
        }
    }
    else{
        EEFloatData = 22222; // this will be understood to be NaN
    }
    
    floatToString(EEFloatData, LocalString);
    concat(SMSMessage, LocalString);
    concat(SMSMessage, ",\"v\":<");
    for(vptr = 3; vptr < 15; vptr++){
        EEProm_Read_Float(effective_address+vptr, &EEFloatData); // Get Next Volume
    // Limit report to 7 characters within integer range
    if((EEFloatData == 0)||(EEFloatData < 0)||(EEFloatData > 0)){// check for NaN
        if(EEFloatData>32000){ // maximum positive integer is 32767 (7FFF)
            EEFloatData = 32000;
        }
        if(EEFloatData>999.999){
            EEFloatData = (int)EEFloatData; //truncates to integer
        }
    }
    else{
        EEFloatData = 22222; // this will be understood to be NaN
    }
        floatToString(EEFloatData, LocalString);
        concat(SMSMessage, LocalString);
        if(vptr < 14){
            concat(SMSMessage, ",");
        }
        else{
            concat(SMSMessage, ">))");
        }
    }
    concat(SMSMessage, ",\"c\":");
    EEProm_Read_Float(effective_address+15, &EEFloatData); // Get saved date 
    if((EEFloatData == 0)||(EEFloatData < 0)||(EEFloatData > 0)){// check for NaN
        EEFloatData = (EEFloatData*100)+ hour; //Add current hour
    }
    else{
        EEFloatData = 131300 + hour;
    }
    floatToString(EEFloatData, LocalString);
    concat(SMSMessage, LocalString);
    concat(SMSMessage, ")");
        
}

/*********************************************************************
 * Function: ReadSIMresponse(char expected_reply[10])
 * Input: Array containing the string expected when the message sent 
 *        to the SIM was successful
 * Output: 1 if the expected reply was received
 * Overview: When an AT command is sent to the FONA it replies.  Usually this is
 *           OK or some other indication of success.  If there was a problem
 *           it returns ERROR.  This routine looks for the response and checks
 *           to see if it was the one expected when things are working properly
 * Note:     There is usually a CR-LF RESPONSE CR-LF.  This routine expects 
 *           the bracketing CR-LF 
 *           Experiments show this taking between 11.71 - 11.72sec when the response is ERROR
 *           we will wait up to 20seconds
 * TestDate: 
 ********************************************************************/
int ReadSIMresponse(char expected_reply[10]){
    int GoodResponse = 0;  // Assume the reply was not the one expected
    IFS0bits.U1RXIF = 0; // Always reset the interrupt flag
    U1STAbits.OERR = 0;  //clear the overrun error bit to allow new messages to be put in the RXREG FIFO
                         // This clears the RXREG FIFO
    IEC0bits.U1RXIE = 1;  // enable Rx interrupts
    NumCharInTextMsg = 0; //Point to the start of the Text Message String
    ReceiveTextMsgFlag = 0; //clear for the next message
    ReceiveTextMsg[0]=0;  //Reset the receive text message array
    
    TMR1 = 0;
    int response_ctr = 0;
    //while(ReceiveTextMsgFlag<2){}
    while((ReceiveTextMsgFlag<2)&&(response_ctr < 10)){// Only wait up to 20 seconds
        if(TMR1 > 31250){//this is 2 seconds
            TMR1 = 0;
            response_ctr++;
        }
    } 
    if(response_ctr < 10){ // we got something
        IEC0bits.U1RXIE = 0;  // disable Rx interrupts
          
        // Here is where I'd like to read the response to see if it was an ERROR or not
        char *MsgPtr;
        int msgLength=strlen(ReceiveTextMsg);
        FONAmsgStatus[0]=0;  //Reset the Fona Message Status array
        MsgPtr = ReceiveTextMsg;
        while((*MsgPtr < 0x30)&&(MsgPtr < ReceiveTextMsg+msgLength-1)){ // Skip line feeds etc  and anything like + and get to the reply text
            MsgPtr++;
        }
        while((*MsgPtr > 0x20)&&(MsgPtr < ReceiveTextMsg+msgLength-1)){// copy the text into the FONAmsgStatus string
            strncat(FONAmsgStatus, MsgPtr,1);
            MsgPtr++;
        }
        msgLength=strlen(expected_reply);
        char ErrCmdMatch[]="ERROR";
        if(strncmp(expected_reply, FONAmsgStatus,msgLength)==0){
            GoodResponse = 1;
        }
        else if(strncmp(ErrCmdMatch, FONAmsgStatus,5)==0){
            GoodResponse = 0;
        }
        else{
            GoodResponse = 0; //something is wrong we should get expected or Error
        }
    }
    else{
        GoodResponse = 0;
    }// we did not hear back from the FONA
    return GoodResponse;    
}


/*********************************************************************
 * Function: void CreateAndSaveDailyReport(void)
 * Input: None
 * Output: None
 * Overview: At noon, collect the information saved during the previous day
 *           Leak,Prime,Battery,12Volume
 *           Save these to the end of the circular daily report EEPROM buffer
 *           the last entry should be a 4 digit number indicating the 
 *           MonthDate for the previous day; which is the date stamp for the data
  * TestDate: 
 ********************************************************************/
void CreateAndSaveDailyReport(void){
    int num_saved_messages;
    int message_position;
    int effective_address;
    int vptr;
    float date;
    // Read EEPROM address, right now 21, which contains the number of messages already saved
    EEProm_Read_Float(DailyReportEEPromStart, &EEFloatData);
    num_saved_messages = EEFloatData;
    num_saved_messages++; //we are adding to the queue
    if(num_saved_messages > 10){
        num_saved_messages = 6;
    }
    EEFloatData = num_saved_messages;  //Update the number of messages in the queue
    EEProm_Write_Float(DailyReportEEPromStart,&EEFloatData);
    // Find the first available address to store this daily report
    if(num_saved_messages > 5){
        message_position = num_saved_messages-5;  //If this is the 24th message since 
                                                   //we were able to send things, it 
                                                   //belongs in position 4 of the circular buffer
    }
    else{message_position = num_saved_messages;
    }
    effective_address = ((message_position - 1)*16)+DailyReportEEPromStart+1;

  /*                  EEPROM STORAGE
 * EEProm#		    EEProm#		         EEProm#	
0	leakRateLong	9	Volume01416	     18	Volume1810
1	longestPrime	10	Volume01618	     19	Volume11012
2	Volume002	    11	Volume01820	     20	Restart Status
3	Volume024	    12	Volume02022		
4	Volume046	    13	Volume02224		
5	Volume068	    14	Volume102		
6	Volume0810	    15	Volume124		
7	Volume01012	    16	Volume146		
8	Volume01214	    17	Volume168		

 Volume01012 = Yesterday(0)10AM-12AM
 Volume124 = Today(1) 2AM-4AM
 */
    EEProm_Read_Float(0, &EEFloatData); // Longest Leak Rate
    EEProm_Write_Float(effective_address,&EEFloatData);
    effective_address++;

    EEProm_Read_Float(1, &EEFloatData); // Longest Prime
    EEProm_Write_Float(effective_address,&EEFloatData);
    effective_address++;
    
    EEFloatData = batteryLevel(); //latest battery voltage
    EEProm_Write_Float(effective_address,&EEFloatData);
    effective_address++;
    
    for(vptr = 2; vptr < 14; vptr++){
        EEProm_Read_Float(vptr, &EEFloatData); // Get Next Volume
        EEProm_Write_Float(effective_address,&EEFloatData);
        effective_address++;
    }
  // add the date stuff
    date = 100*BcdToDec(getMonthI2C());
    date = date + BcdToDec(getDateI2C());
    EEFloatData = date;
    EEProm_Write_Float(effective_address,&EEFloatData);

    // Now that the daily report information has been saved
    // we can move today's data to its new locations and
    // Clear RAM and EEPROM associated with message variables
    ResetMsgVariables();
      
}
/*********************************************************************
 * Function: int SendSavedDailyReports(void)
 * Input: none
 * Output: the number of daily reports still waiting to be sent
 * Note:  Sends saved daily reports to the MainphoneNumber.  Messages are sent
 *        newest first.  As long as the network is available and messages are
 *        being sent, older saved messages not able to be sent before are sent 
 * 
 *        Once all messages are sent, received messages are read.  If they are
 *        messages from friends (AW) the appropriate action is taken.  If not, they
 *        are just deleted  
 * TestDate: 
 ********************************************************************/
int SendSavedDailyReports(void){
    int ready = 0; 
    int num_saved_messages;
    int message_position;
    int effective_address;  //EEProm position.  We assume each position is a float and start at 0
    // Turn on the FONA
    sendDebugMessage("Trying to send a daily or diagnostic message", 1);
    ready = turnOnSIM();
    ready = tryToConnectToNetwork(); // This will try 7 times to connect to the network
    
      
    // Send a daily report if we have network connection and there are messages to send
    EEProm_Read_Float(DailyReportEEPromStart, &EEFloatData);// Read EEPROM address, as of now 21, which contains the number of messages already saved
    num_saved_messages = EEFloatData;
    num_unsent_daily_reports = num_saved_messages;  //as long as there have been 5 or less saved messages, these are the same
    if(num_saved_messages > 5){
            num_unsent_daily_reports = 5;  //This is the maximum number of messages saved in our daily report buffer
    }
    while((num_unsent_daily_reports > 0 )&&(ready == 1)){
        // Find the EEPROM address of the start of the next daily report data
        if(num_saved_messages > 5){
            message_position = num_saved_messages-5;  //If this is the 7th message since 
                                                       //we were able to send things, it 
                                                       //belongs in position 2 of the circular buffer
        }
        else{message_position = num_saved_messages;}
        effective_address = ((message_position - 1)*16)+DailyReportEEPromStart+1;
        // Create the message including adding the hour
        CreateNoonMessage(effective_address);  //Gather the data into the array SMSMessage
        // send the message and check for the reply that it was sent
        phoneNumber = MainphoneNumber;  // Make sure we are sending to the proper destination
        ready = sendTextMessage(SMSMessage);   
        // Check to see if the FONA replies with ERROR or not
        char CmdMatch[]="CMGS:";  // we only look for letters in reply so exclude leading +
        ready = ReadSIMresponse(CmdMatch);
        if(ready){    
            num_saved_messages--;
            num_unsent_daily_reports--;
        }
        else{
            break; // we were not able to send this message so stop trying until next hour
        }
        ready = CheckNetworkConnection(); // make sure we still have a network connection
    }
    
        // If hourly diagnostic messages are enabled and we are still ready, create and send the message
    while (diagnostic == 1){
        numberTries++;
        if(numberTries > 30) {
            break;
        }
        ready = CheckNetworkConnection();
        if(ready == 1) {
            phoneNumber = DebugphoneNumber;

            batteryFloat = batteryLevel();               
            //Put the phone number back to Upside 

            createDiagnosticMessage();
            
            while(1){
                ready = sendTextMessage(SMSMessage);
                // Check to see if the FONA replies with ERROR or not
                char CmdMatch[]="CMGS:";  // we only look for letters in reply so exclude leading +
                ready = ReadSIMresponse(CmdMatch);
                if (ready == 1) {
                    break;
                }
            }
            // NOTE:  We should not try to send messages on the serial port while the FONA is ON
                //sendDebugMessage("  \n The attempt to send the hourly diagnostic message to the FONA was a ", ready);  //Debug
            timeSinceLastRestart++; // if first time in loop this hour, increase the hour since last restart by one
            extRtccTalked = 0; // reset the external clock talked bit
            sleepHrStatus = 0; // reset the slept during that hour
            EEProm_Write_Float(DiagnosticEEPromStart,&sleepHrStatus);                      // Save to EEProm
            extRtccManualSet = 0; 
            phoneNumber = MainphoneNumber;  // Make sure we are sending to the proper destination
            break;
        }
    }
    numberTries = 0;
    // after we are done sending update the number of messages still waiting to be sent
    // if there is no problem with the network, this will be zero
    EEFloatData = num_saved_messages;  //Update the number of messages in the queue
    EEProm_Write_Float(DailyReportEEPromStart,&EEFloatData);
    //Read received messages and act on them
    /// BEING TESTED  readSMSMessage(int msgNum); //still not used 
    /// BEING TESTED  interpretSMSmessage(void); //still not used
    //  BEING TESTED   ClearReceiveTextMessages(int MsgNum, int ClrMode); 
    //Turn off the FONA
    turnOffSIM();
    
    return num_unsent_daily_reports;
    

 // Taken out 4/24/17 RKF   RTCCSet(); // updates the internal time from the external RTCC if the internal RTCC got off any through out the day
               // RKF QUESTION - Why do we do this?  I don't think we use the internal RTCC for anything

}


void createDiagnosticMessage(void) {
    char LocalString[20]; 
    float LocalFloat = hour;
    SMSMessage[0] = 0; //reset SMS message array to be empty
    LocalString[0] = 0;
  
    concat(SMSMessage, "(\"t\":\"d\",\"d\":(\"s\":");
    EEProm_Read_Float(DiagnosticEEPromStart, &EEFloatData);
    floatToString(EEFloatData, LocalString); //populates the sleepHrStatusString with the value from EEPROM
    concat(SMSMessage, LocalString);
    concat(SMSMessage, ",\"b\":");
    floatToString(batteryFloat, LocalString); //latest battery voltage
    concat(SMSMessage, LocalString);
    concat(SMSMessage, ",\"r\":");
    floatToString(timeSinceLastRestart, LocalString); // hours since the system restarted
    concat(SMSMessage, LocalString);
    concat(SMSMessage, ",\"c\":");
    floatToString(extRtccTalked, LocalString); // if the external rtcc responded in the last hour
    concat(SMSMessage, LocalString);
    concat(SMSMessage, ",\"t\":");
    floatToString(LocalFloat, LocalString); // what it thinks the hour is
    concat(SMSMessage, LocalString);
    concat(SMSMessage, ",\"n\":");
    floatToString(numberTries, LocalString); // number of tries to connect
    concat(SMSMessage, LocalString);
    concat(SMSMessage, ",\"e\":");
    floatToString(extRtccManualSet, LocalString); // external RTCC was manually set forward
    concat(SMSMessage, LocalString);
    
    concat(SMSMessage, ">))");
}   

