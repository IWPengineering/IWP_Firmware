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
char origDebugphoneNumber[] = "+17176837803"; // Number for Fish cell phone 
char DebugphoneNumber[15]; //Defined during initialization 
//char DebugphoneNumber[] = "+17177784498"; // Upside 
//char origDebugphoneNumber[]="+17176837704"; //Number for Sue Fish Cell
//char DebugphoneNumber[]="+254787620369"; //Number for Paul Zwart cell phone; this number is changed and default is the mainphonenumber
char origMainphoneNumber[]="+17177784498"; // Upside Wireless
//char origMainphoneNumber[]="+17176837803"; // Randy
char MainphoneNumber[15]; //Defined during initialization
//char origMainphoneNumber[]="+17176837704"; //Sue
char origCountryCode[] = "+254"; // This is Kenya 
char CountryCode[6];
char SendingPhoneNumber[]="+254787620369"; //this is read from the received SMS message default to mine
//char phoneNumber[] = "+17177784498"; // Number Used to send text message report (daily or hourly)
//char phoneNumber []="+17176837803"; // Randy
char* phoneNumber = MainphoneNumber; // Randy
char CountryCode[] = "+254"; // This is Kenya 
int LeaveOnSIM = 0;  // this is set to 1 when an external message says to not shut off the SIM
char FONAmsgStatus[11]; //Message REC/STO, UNREAD/READ, UNSENT/SENT
char SignalStrength[3]; //hold the values of the signal strength
char ReceiveTextMsg[160]; //This is the string used to buffer up a message read from the FONA
char SMSMessage[160]; //A string used to hold all SMS message sent with FONA
int NumCharInTextMsg = 0; //Keeps track of the number of characters in the received text string
char ReceiveTextMsgFlag = 0; //Set to 1 when a complete text message has been received
int num_unsent_daily_reports = 0; //this is the number of saved daily reports that have not been sent
int longest_wait = 4225; // amount of time needed to receive 260 (100header, 160msg) characters at 9600 BAUD
int diagPCBpluggedIn = 0; // Used to keep track of whether diagnostic PCB is plugged in or not
int MaxSMSmsgSize = 30;  // number of slots available on SIM to store text messages
int FONAisON = 0; //Keeps track of whether the FONA has been turned on


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
 * Output: SIM_OFF  this is a 1 if the SIM turned OFF and 0 if not
 * Overview: Powers Down (turns off) the SIM900
 * Note: Pic Dependent
 * TestDate: 12-22-2017 RKF
 ********************************************************************/
int turnOffSIM() {
    int SIM_OFF = 0;  // Assume the SIM is not off
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
    FONAisON = !SIM_OFF; // Makes FONAisON = 0 when FONA is OFF
    return SIM_OFF;//	while (digitalPinStatus(statusPin) == 1){ //Checks see if the Fona is on pin
 }
/*********************************************************************
 * Function: turnOnSIM
 * Input: None
 * Output: SIM_ON  this is a 1 if the SIM turned on and 0 if not
 * Overview: Turns on SIM900 - If the SIM is already on, just return
 *                             If it is off, the PS (Power Status) pin = 0,
 *                             Strobe the KEY pin low for 2 sec and then go high
 *                             The SIM should turn on (PS = 1) after approx. 0.8 - 1.3sec
 *                             wait for 2sec and check to see if it turned on or not
 * 
 * Note: Pic Dependent
 * TestDate: 7/6/2018 RKF
 ********************************************************************/
int turnOnSIM() {
    int SIM_ON = 0;  // Assume the SIM is not on
    digitalPinSet(simVioPin, 1); //PORTAbits.RA1 = 1; //Tells Fona what logic level to use for UART
    if (digitalPinStatus(statusPin)) { //Checks see if the Fona is already on
        SIM_ON = 1;
    }
    else{
        digitalPinSet(pwrKeyPin, 0); //PORTBbits.RB6 = 0; //set low pin 15 for 2000ms to turn on Fona
        delayMs(2000);
        digitalPinSet(pwrKeyPin, 1); //PORTBbits.RB6 = 1; // Reset the Power Key so it can be turned off later (pin 15)
        // Experimental tests showed that it takes 0.8 - 1.3sec for the FONA to turn on
        delayMs(2000);
        if (digitalPinStatus(statusPin)) { // See if the FONA turned on
            SIM_ON = 1;
        }
    }
    FONAisON = SIM_ON;
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
        //if (CheckNetworkConnection()) {
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
    ClearWatchDogTimer(); // In case we were here for 45sec and the calling routine did not allow for that
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
 *           We call anything with a low less than 1.28sec No Network Connection
 *           If there IS network, we are in this routine between 1.28 to 3seconds
 *           If there is NO Network, we are in this routine between up to 1.8seconds
 * 
 *            * Note: Timer speed dependent
 * TestDate: 6/27/2018 RKF
 ********************************************************************/
int CheckNetworkConnection(void) //True when there is a network connection
{
    // This function should only be called once we know the FONA is on.  
    // If the FONA is on, the NET light will blink so we should not get stuck here
    // waiting for 1's and 0's.  Just to be safe, leave if you wait too long for
    // a high or low
    
    // The timing in this routine assumes that Timer 1 is clocked at 15.625khz

    int network_status = 0;  // Assume there is no network connection
    int LightIsFlashing = 1; //Assume the light is blinking

    if (digitalPinStatus(netLightPin) == 1){ //(PORTBbits.RB14 == 1)
    // If the light is ON when we get here, we know we don't have network if it
    // goes low and stays low for more than 1.15 seconds (TMR1 = 18000).  In this
    // case we are here for max 1.28sec when there IS a network connection and less if not   
        
        // Wait until the light turns off
        TMR1 = 0;
        while ((digitalPinStatus(netLightPin))&&(LightIsFlashing)) {
            if(TMR1 > 2000){
                LightIsFlashing = 0; // stop if the light is stuck ON longer than 128ms 
            }
        }
        // Wait for next rising edge or stop if we have network
        TMR1 = 0;
        while ((digitalPinStatus(netLightPin) == 0)&&(LightIsFlashing)&& (!network_status)) {
            if(TMR1 > 18000){
                network_status = 1; 
            }
        }    
    }
    else {
    // If the light is OFF when we get here, we need to wait till it goes ON
    //      if it was OFF for more than 1.15 seconds we know we have network
    //      if not, we need to wait for the next low and see if it lasts more than
    //      1.15 seconds.   
    //      If we enter at the start of a low, we will be here for 3seconds.  
    //      If more than 1.15 seconds from the end of the low, we are here for less than 3 seconds
    //      If we enter just less than 1.15 sec from end of first low, we are here for 1.15 + 0.064 + 1.15 = 2.36sec 
        
        // Wait for next rising edge
        TMR1 = 0;
        while ((digitalPinStatus(netLightPin) == 0)&&(LightIsFlashing)) {
            if(TMR1 > 55000){
                LightIsFlashing = 0;  // stop if the line is stuck OFF longer than 3.5 sec max should be 3sec when no network 
            }
        }
        if((LightIsFlashing)&&(TMR1 > 18000)){
            network_status = 1;
        }
        // If needed wait for the next Low
        if(!network_status){
            TMR1 = 0;         
            while ((digitalPinStatus(netLightPin))&&(LightIsFlashing)) {
                if(TMR1 > 2000){
                    LightIsFlashing = 0; // stop if the light is stuck ON longer than 128ms 
                }
            }
            TMR1 = 0;
            while ((digitalPinStatus(netLightPin) == 0)&&(LightIsFlashing)&& (!network_status)) {
                if(TMR1 > 18000){
                    network_status = 1; 
                }
            }   
        }
    }
  
    return network_status;  // True, when there is a network connection. (low lasts longer than 1.1sec)                      

}


void sendDebugMessage(char message[50], float value){
    int msg_length = stringLength(message);
    int SIMwasON = 0;
    if (digitalPinStatus(statusPin) == 1) { // if the Fona is on, turn it off
                                            // the FONA gets messed up if the serial line
                                            // is sending non-AT commands when it is on
       turnOffSIM();
       SIMwasON = 1;
    }
    if(print_debug_messages >= 1){
        char debugMsg[150];
        char debugValueString[20];
        debugMsg[0] = 0;
        concat(debugMsg, message); // Text part of message
        if(value != -0.1){ // -0.1 is used when there is no valid value to report and we just want the text
            floatToString(value, debugValueString); 
            concat(debugMsg,debugValueString);
        }
        concat(debugMsg, "\n");
        sendMessage(debugMsg);
        // we should wait for the debug message to be sent before turning on the SIM
        // the sendMessage function should only return once all but the last char or two
        // of the message was sent but we will wait enough for the whole message
        // need to wait about 1ms/character
        delayMs(msg_length+3);
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
 * TestDate: 6-28-2018
 ********************************************************************/
int sendMessage(char message[200]) {
    int stringIndex = 0;
    int success = 0;
    int num_msg_char = 0;
    int MaxWaitTime = 0;
    num_msg_char = stringLength(message);
    MaxWaitTime = num_msg_char*24; //(#char *10bits/char)*(15625/9600)*1.5 (50% safety factor
    U1STAbits.UTXEN = 1; //enable transmit
    TMR1 = 0;  // transmitting 160char (10bits each) at 9600BAUD should take less than 170ms
               // Assuming a 15.625khz clock for timer 1, that is 2656 clock cycles
    while ((stringIndex < num_msg_char)&&(TMR1 < MaxWaitTime)) {
        while((U1STAbits.UTXBF == 1)&&(TMR1 < MaxWaitTime)){ //wait for the buffer to be ready for the next character
                                                      // but don't allow us to get hung if something is wrong
        }
        U1TXREG = message[stringIndex];
        stringIndex++;
    }
    if(stringIndex == num_msg_char){
        success = 1;
    }
    return success;  // report a failure if we don't send the entire string
}
/*********************************************************************
 * Function: readMessage()
 * Input: integer between 1-30 indicating which message to read
 *        Assumes that the SIM was already turned on
 * Output: None - three global arrays are modified by this function
 *                ReceiveTextMsg has the message string in it 
 *                SendingPhoneNumber has the phone number of the sender
 *                FONAmsgStatus contains string indicating if the message slot 
 *                    was read/unread, unsent/sent etc.
 * Overview: Reads a single text message from the FONA and puts it into 
 *           the string ReceiveTextMsg
 *           ReceiveTextMsgFlag = 1 when a complete message has been received
 * Note: Library, Pic Dependent
 * TestDate: Not Tested
 ********************************************************************/
void readSMSMessage(int msgNum) {
    int success = 0;
    success = SetFONAtoTextMode();
    
    if(success == 1){
    // Send the command to the FONA to read a text message
    // AT+CMGR=msg_number_to_read
    
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
        TMR1 = 0; // start timer for max 260characters (100command header + 160 message) ? what about the 25ms gap between echo and response?
        while(TMR1<longest_wait){  }
     
        // Here is where I'd like to extract information if there is something there
        FONAmsgStatus[0]=0;  //Reset the Fona Message Status array
        SendingPhoneNumber[0]=0; //Reset the SendingPhoneNumber
        int msgLength=strlen(ReceiveTextMsg);
        if(msgLength > 22){ // if less, there was no message, we either just got OK or ERROR
        //read the status of the message and the phone number that sent it
        //command echo then +CMGR: "REC READ","+85291234567",,"
            char *MsgPtr;
            MsgPtr = ReceiveTextMsg;
            // Read whether the message is REC READ, REC UNREAD etc
            while((*MsgPtr != '\"')&&(MsgPtr < ReceiveTextMsg+msgLength-1)){
                MsgPtr++;
            }
            MsgPtr++;
            while((*MsgPtr !='\"')&&(MsgPtr < ReceiveTextMsg+msgLength-1)){
                strncat(FONAmsgStatus, MsgPtr,1);
                MsgPtr++;
            }
            //Now get the sending phone number
            while((*MsgPtr != '+')&&(MsgPtr < ReceiveTextMsg+msgLength-1)){
                MsgPtr++;
            }
            while((*MsgPtr !='\"')&&(MsgPtr < ReceiveTextMsg+msgLength-1)){
                strncat(SendingPhoneNumber, MsgPtr,1);
                MsgPtr++;
            }   
            UpdateSendingPhoneNumber(); // If the caller is local, strip off the country code and replace with 0
            //Now extract the actual message
            char *MsgPtr2;
            MsgPtr2 = ReceiveTextMsg;
            while((*MsgPtr != '\n')&&(MsgPtr < ReceiveTextMsg+msgLength-1)){
                MsgPtr++;
            }
            MsgPtr++;
            while((*MsgPtr !='\n')&&(MsgPtr < ReceiveTextMsg+msgLength-1)){
                *MsgPtr2 = *MsgPtr;
                MsgPtr++;
                MsgPtr2++;
            } 
            *MsgPtr2 = 0;
        }    
    }
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
 *      AWI   indicates that a person at the pump wants information
 *      AWC indicates changes to the RTCC date,month,hour.
 *              AWC date,month,delta_hour  ie.  March 12th add 6hrs to hour AWC 12,03,06
 *      AWD enable/disable hourly diagnostic messages
 *              AWD 0 (disable diagnostic messages)
 *              AWD 1 (enable hourly diagnostic messages). When diagnostic messages are 
 *                    enabled, messages are sent to DebugphoneNumber
 *              AWD 2 +############ enable hourly diagnostic messages and 
 *                  change DebugphoneNumber to the phone provided
 *      AWCC +### change the country code to the number provided 
 *      AWPN    +########### change the phone number used for daily reports
 *      
 * 
 * Note: Library
 * TestDate: no tested
 ********************************************************************/
void interpretSMSmessage(void){
    char CmdMatch[]="AWC";
    if(strncmp(CmdMatch, ReceiveTextMsg,3)==0){
      updateClockCalendar();
    }  
    strncpy(CmdMatch,"AWD",3);
    if(strncmp(CmdMatch, ReceiveTextMsg,3)==0){
      enableDiagnosticTextMessages();
    } 
    strncpy(CmdMatch,"AWCC",4);
    if(strncmp(CmdMatch, ReceiveTextMsg,4)==0){
      // Change the stored country code 
        ChangeCountryCode();
    } 
    strncpy(CmdMatch,"AWI",3);
    if(strncmp(CmdMatch, ReceiveTextMsg,3)==0){
        // Report basic information about pump status
        OneTimeStatusReport();
    }
    strncpy(CmdMatch,"AWPN",4);
    if(strncmp(CmdMatch, ReceiveTextMsg,4)==0){
        // Change the phone number used for daily reports
        ChangeDailyReportPhoneNumber();
    }
}

/*********************************************************************
 * Function: updateClockCalendar()
 * Input: None - must be called after the readSMSMessage
 *                ReceiveTextMsg has the message string in it 
 *                SendingPhoneNumber has the phone number of the sender
 * Output: None
 * Overview: An SMS message was received (in the string ReceiveTextMsg)
 *           with the AWC prefix indicating that
 *           the date, month, hour or minute needs to be changed.  Depending upon
 *           the number of parameters sent, they are interpreted differently
 *              AWC delta_hour
 *              AWC delta_hour delta_minute
 *              AWC date,month,delta_hour
 *              AWC date, month, delta_hour, delta_minute  
 *              AWC:12,03,06  March 12th add 6hrs to hour 
 *              AWC 12 3 6 is also accepted
 * 
 *           The cell phone number that sent the message is expected to already
 *           be in the string SendingPhoneNumber
 * Note: Library
 * TestDate: 7/26/2018 RKF
 ********************************************************************/
void updateClockCalendar(){
    char MsgPart[3];
    int success = 0;
    int ext_success = 0;  //see if you were able to change the external RTCC
    int info_provided = 1; //assume all fields are reasonable
    int local_val[4];
    int val_offset = 0;
    char delta_hour = 0;
    char delta_min = 0;
    char newDate = BcdToDec(getDateI2C()); // we may overwrite this
    char newMonth = BcdToDec(getMonthI2C());// we may overwrite this
    char new_hour = hourVTCC;
    char new_min = minuteVTCC; // we may overwrite this
    
    char *MsgPtr;
    int msgLength=strlen(ReceiveTextMsg);
    MsgPtr = ReceiveTextMsg;
    // Extract the information provided.  This can be from 1 to 4 values
    while((MsgPtr < ReceiveTextMsg+msgLength-1)&&(val_offset < 4)){
        // Skip the next command value
        while((*MsgPtr != 0x2d)&&!((*MsgPtr > 0x2f)&&(*MsgPtr < 0x3a))&&(MsgPtr < ReceiveTextMsg+msgLength-1)){ // skip non-numbers except - (neg sign)
            MsgPtr++;
        } 
        // Get the value
        MsgPart[0]=0;
        while(((*MsgPtr == 0x2d)||((*MsgPtr > 0x2f)&&(*MsgPtr < 0x3a)))&&(MsgPtr < ReceiveTextMsg+msgLength-1)){ // accept numbers and - (neg sign)
            strncat(MsgPart,MsgPtr,1);
            MsgPtr++;
        }
        if(MsgPtr < ReceiveTextMsg+msgLength-1){//Make sure there will always be a character after the last value, new line?
            local_val[val_offset] = atoi(MsgPart);
            val_offset++;
        }
    }
    // Now work with what was received
    if(val_offset == 1){// all we have is a delta hour
        delta_hour = local_val[0];
    }
    if(val_offset == 2){// we have delta hour and delta minute
        delta_hour = local_val[0];
        delta_min = local_val[1];
    }
    if(val_offset == 3){// we have date, month delta hour
        newDate = local_val[0];
        newMonth = local_val[1];
        delta_hour = local_val[2];
    }
    if(val_offset == 4){// we have date, month, delta hour, delta min
        newDate = local_val[0];
        newMonth = local_val[1];
        delta_hour = local_val[2];
        delta_min = local_val[3];
    }
    // See if the information provided is valid
    if((delta_hour < -23)||(delta_hour > 23)||(delta_min < -59)||(delta_min > 59)){
        info_provided = 0; // The value in the delta hour field is incorrect
    }
    else{
        new_min = delta_min + new_min;
            if(new_min >= 60){
                new_min = new_min - 60;
                new_hour++;
            }
            if(new_min < 0){
                new_min = new_min + 60;
                new_hour--;
            }
        new_hour = delta_hour + new_hour;
        if(new_hour >= 24){new_hour = new_hour - 24;}
        if(new_hour < 0){new_hour = new_hour + 24;}
    }
    if((val_offset == 3)||(val_offset == 4)){
        if((newDate > 31)||(newDate < 1)||(newMonth > 12)||(newMonth < 1)){
            info_provided = 0; // the date or month field is incorrect
        }
    }
    // Time to make the requested changes
    if(info_provided){
        hour = new_hour;
        min = new_min;
        if(hour > 23){ // In case someone adds time that wraps over
            hour = hour - 24;
        }
        int year = BcdToDec(getYearI2C());
        int wkday = 1; //just always assume its the 1st day of the week
        ext_success = setTime(0,min,hour,wkday,newDate,newMonth,year);//   (sec, min, hr, wkday, date, month, year)
    
        // Update the settings for the internal RTCC
        initializeVTCC(0, min, hour, newDate, newMonth);
    }
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
            char ClkCal_val[3];
            if(info_provided){
                concat(localMsg,"Changed Date-Month Hour:Minute to \n");
                itoa(ClkCal_val,newDate,10);
                concat(localMsg, ClkCal_val);
                concat(localMsg,"-");
                itoa(ClkCal_val,newMonth,10);
                concat(localMsg, ClkCal_val);
                concat(localMsg," ");
                itoa(ClkCal_val,new_hour,10);
                concat(localMsg, ClkCal_val);
                concat(localMsg,":");
                itoa(ClkCal_val,new_min,10);
                concat(localMsg, ClkCal_val);                
            }
            else{
                concat(localMsg,"No Change made, some part of information provided was incorrect ");
            }
            
            sendTextMessage(localMsg);   //note, this now returns 1 if successfully sent to FONA
            // I think I  need to wait until it is sent.  See the Daily Report code.
            char CmdMatch[]="CMGS:";  // we only look for letters in reply so exclude leading +
            ReadSIMresponse(CmdMatch); // this looks for the response from the FONA that the message has been received
            phoneNumber = MainphoneNumber;
        }
    }
  // don't to this.  Let calling routine take care of it  turnOffSIM();
    // Should we wait for the message to be sent before trying to work with the FONA?
    delayMs(40);
} 
/*********************************************************************
 * Function: void ChangeDailyReportPhoneNumber()
 * Input: None - must be called after the readSMSMessage
 *                ReceiveTextMsg has the message string in it 
 * Output: None
 * Overview: An SMS message was received (in the string ReceiveTextMsg)
 *           with the AWPN prefix indicating that
 *           the phone number to be sent the daily report,MainphoneNumber[], should be changed.
 *           If the system restarts the original number in EEPROM will again be used.
 *           This function does not overwrite the EEPROM
  * Note: Library
 * TestDate: not tested
 ********************************************************************/
void ChangeDailyReportPhoneNumber(){
    char *MsgPtr;
    int msgLength=strlen(ReceiveTextMsg);
    MsgPtr = ReceiveTextMsg;
    MainphoneNumber[0]=0; //reset Main Phone Number String

    // Skip to New Main Number
    while((*MsgPtr != 0x2b)&&!((*MsgPtr > 0x2f)&&(*MsgPtr < 0x3a))&&(MsgPtr < ReceiveTextMsg+msgLength-1)){ // skip non-numbers except + 
        MsgPtr++;
    }
    if(*MsgPtr != '+'){
        strncat(MainphoneNumber,"+",1);
    }
    while(((*MsgPtr == 0x2b)||((*MsgPtr > 0x2f)&&(*MsgPtr < 0x3a)))&&(MsgPtr < ReceiveTextMsg+msgLength-1)){ // accept numbers and +
        strncat(MainphoneNumber,MsgPtr,1);
        MsgPtr++;
    }
    PhonenumberToEEPROM(EEpromMainphoneNumber,MainphoneNumber);// Save this new number to EEPROM
      // Now we want to reply to the sender telling it what we just did
    int success = 0;
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
            concat(localMsg,"Changed Main Reporting Phone Number to ");
            concat(localMsg,MainphoneNumber);
            sendTextMessage(localMsg);   //note, this now returns 1 if successfully sent to FONA
            // I think I  need to wait until it is sent.  See the Daily Report code.
            char CmdMatch[]="CMGS:";  // we only look for letters in reply so exclude leading +
            ReadSIMresponse(CmdMatch); // this looks for the response from the FONA that the message has been received
            phoneNumber = MainphoneNumber;
        }
    }
    // Should we wait for the message to be sent before trying to work with the FONA?
    delayMs(40);
}
/*********************************************************************
 * Function: enableDiagnosticTextMessages()
 * Input: None - must be called after the readSMSMessage
 *                ReceiveTextMsg has the message string in it 
 *                SendingPhoneNumber has the phone number of the sender
 * Output: None
 * Overview: An SMS message was received (in the string ReceiveTextMsg)
 *           with the AWD prefix indicating that
 *           the hourly diagnostic messages should either be 
 *              0 = disabled
 *              1 = enabled
 *              2 (phone number) = enabled and recipient phone number changed to
 *                                 the number provided.
  * Note: Library
 * TestDate: not tested
 ********************************************************************/
void enableDiagnosticTextMessages(){
    int success;
    char *MsgPtr;
    char MsgPart[3];
    char localMsg[160];
    localMsg[0] = 0;
    MsgPtr = ReceiveTextMsg;
    int DiagnosticMode = 0;
    int msgLength=strlen(ReceiveTextMsg);
    // Skip over to the desired mode (0,1,2)
    while(((*MsgPtr < 0x30)||(*MsgPtr > 0x39))&&(MsgPtr < ReceiveTextMsg+msgLength-1)){ // skip non-numbers
        MsgPtr++;
    }
    // Get the Diagnostic Reporting Mode
    MsgPart[0]=0;
    while(((*MsgPtr > 0x2f)&&(*MsgPtr < 0x3a))&&(MsgPtr < ReceiveTextMsg+msgLength-1)){ // accept numbers
        strncat(MsgPart,MsgPtr,1);
        MsgPtr++;
    }
    DiagnosticMode = atoi(MsgPart);
    if(DiagnosticMode == 0){
        diagnostic = 0; // Disable hourly diagnostic reporting
        concat(localMsg,"Hourly Diagnostic Messages Have Been DISABLED ");
    }
    if(DiagnosticMode == 2){
        // Change the Diagnostic Phone Number
        DebugphoneNumber[0]=0;  //Reset the phone number string
         // Skip to Phone Number
        while((*MsgPtr != 0x2b)&&!((*MsgPtr > 0x2f)&&(*MsgPtr < 0x3a))&&(MsgPtr < ReceiveTextMsg+msgLength-1)){ // skip non-numbers except + 
            MsgPtr++;
        }
        if(*MsgPtr != '+'){
            strncat(DebugphoneNumber,"+",1);
        }
        while(((*MsgPtr == 0x2b)||((*MsgPtr > 0x2f)&&(*MsgPtr < 0x3a)))&&(MsgPtr < ReceiveTextMsg+msgLength-1)){ // accept numbers and +
            strncat(DebugphoneNumber,MsgPtr,1);
            MsgPtr++;
        }
        PhonenumberToEEPROM(EEpromDebugphoneNumber,DebugphoneNumber); // Save new debug phone number to EEPROM   
    }  
    if(DiagnosticMode > 0){ // We are either just enabling or enabling and changing the phone number
        diagnostic = 1; // Enable hourly diagnostic reporting
        concat(localMsg,"Hourly Diagnostic Messages Have Been ENABLED to be sent to ");
        concat(localMsg,DebugphoneNumber);
    }
    EEProm_Write_Float(EEpromDiagStatus,&diagnostic); //Save the new Diagnostic Status to EEPROM  
   
    // Now we want to reply to the sender telling it what we just did
    success = turnOnSIM();  // returns 1 if the SIM powered up)
    if(success == 1){
        // Try to establish network connection
        success = tryToConnectToNetwork();  // if we fail to connect, don't send the message
        if(success == 1){
        // Send off the data
            phoneNumber = SendingPhoneNumber;
                      
            sendTextMessage(localMsg);   //note, this now returns 1 if successfully sent to FONA
            // I think I  need to wait until it is sent.  See the Daily Report code.
            char CmdMatch[]="CMGS:";  // we only look for letters in reply so exclude leading +
            ReadSIMresponse(CmdMatch); // this looks for the response from the FONA that the message has been received
            phoneNumber = MainphoneNumber;
        }
    }
    // Should we wait for the message to be sent before trying to work with the FONA?
    delayMs(40);
    
}
/*********************************************************************
 * Function: void ChangeCountryCode()
 * Input: None - must be called after the readSMSMessage
 *                ReceiveTextMsg has the message string in it 
 *                SendingPhoneNumber has the phone number of the sender
 * Output: None
 * Overview: The received message should look like AWPN +254 where 254 is understood
 *           to be the country code where the pump is installed.
 *           This function changes the variable CountryCode so that other
 *           routines can use this information to know if they are responding
 *           locally or internationally.
 * TestDate: June 7 2018
 ********************************************************************/
void ChangeCountryCode(){
    char *MsgPtr;
    int msgLength=strlen(ReceiveTextMsg);
    MsgPtr = ReceiveTextMsg;
    CountryCode[0]=0; //reset Country Code String

    // Skip to Country Code
    while((*MsgPtr != 0x2b)&&!((*MsgPtr > 0x2f)&&(*MsgPtr < 0x3a))&&(MsgPtr < ReceiveTextMsg+msgLength-1)){ // skip non-numbers except + 
        MsgPtr++;
    }
    if(*MsgPtr != '+'){
        strncat(CountryCode,"+",1);
    }
    while(((*MsgPtr == 0x2b)||((*MsgPtr > 0x2f)&&(*MsgPtr < 0x3a)))&&(MsgPtr < ReceiveTextMsg+msgLength-1)){ // accept numbers and +
        strncat(CountryCode,MsgPtr,1);
        MsgPtr++;
    }
    // Save this new value in EEPROM in case we have a reset
    PhonenumberToEEPROM(EEpromCountryCode,CountryCode); 
      // Now we want to reply to the sender telling it what we just did
    int success = 0;
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
            concat(localMsg,"Changed Country Code to ");
            concat(localMsg,CountryCode);
            sendTextMessage(localMsg);   //note, this now returns 1 if successfully sent to FONA
            // I think I  need to wait until it is sent.  See the Daily Report code.
            char CmdMatch[]="CMGS:";  // we only look for letters in reply so exclude leading +
            ReadSIMresponse(CmdMatch); // this looks for the response from the FONA that the message has been received
            phoneNumber = MainphoneNumber;
        }
    }
    // Should we wait for the message to be sent before trying to work with the FONA?
    delayMs(40);
    
}
/*********************************************************************
 * Function: void UpdateSendingPhoneNumber()
 * Input: None - must be called after the readSMSMessage
 *                ReceiveTextMsg has the message string in it 
 *                SendingPhoneNumber has the phone number of the sender
 * Output: None
 * Overview: If the sending phone number is from the same country, we want to 
 *           replace the +CountryCode with a 0 before replying
 *           If the CountryCode = 1, indicating that the system is in the USA,
 *           the +1 should stay
 * TestDate: June 7 2018
 ********************************************************************/
void UpdateSendingPhoneNumber(){
    int LengthCntryCode = strlen(CountryCode);
    int LengthSendingPhoneNumber = strlen(SendingPhoneNumber);
    if(strncmp("+1",CountryCode,LengthCntryCode)!=0){ // We are not calling from the USA
        if(strncmp(CountryCode, SendingPhoneNumber,LengthCntryCode)==0){
            // replace +country_code with 0
            SendingPhoneNumber[0]='0';
            int i;
            for (i=1;i<(LengthSendingPhoneNumber-LengthCntryCode+1);i++){
                SendingPhoneNumber[i]=SendingPhoneNumber[i+LengthCntryCode-1];
            }
            SendingPhoneNumber[i]=0;        
        }
    }
    
}
/*********************************************************************
 * Function: void PhonenumberToEEPROM(int EEoffset,char PhoneNumber)
 * Input: EEoffset - this is the location expected for the floats 
 *                   used to save the phone number value
 *        PhoneNumber - the string which holds the phone number to be saved
 * 
 * Output: None
 * Overview: A phone number (which is assumed to have a + in the first position)
 *           is broken into a 7 character string and a second string long enough
 *           to accommodate the remaining characters is the passed phone number.
 *           the length of the string is limited to know more than 7 characters
 *           so that its mantissa will fit in a float.
 *           The upper part of the string is saved at EEPROMbase + EEoffset
 *           The lower part is saved at EEPROMbase + EEoffset + 1
 * TestDate: 8/11/2018 RKF
 ********************************************************************/
void PhonenumberToEEPROM(int EEoffset,char *PhoneNumber){
        char LocalPhoneNumber[8];
        char *MsgPtr=PhoneNumber+1;
        char LengthOfPhoneNumber = stringLength(PhoneNumber);
        float PhoneNum;
        if(LengthOfPhoneNumber < 7){ //This is what we do when we are saving the Country Code
            strncpy(LocalPhoneNumber,MsgPtr,LengthOfPhoneNumber);
            PhoneNum = atof(LocalPhoneNumber);
            EEProm_Write_Float(EEoffset,&PhoneNum); 
        }
        else{
            strncpy(LocalPhoneNumber,MsgPtr,7);
            PhoneNum = atof(LocalPhoneNumber);
            EEProm_Write_Float(EEoffset,&PhoneNum);
            MsgPtr=PhoneNumber+8;
            strncpy(LocalPhoneNumber,MsgPtr,LengthOfPhoneNumber-7);
            PhoneNum = atof(LocalPhoneNumber);
            EEProm_Write_Float(EEoffset+1,&PhoneNum);
        }
}

/*********************************************************************
 * Function: void EEPROMtoPhonenumber(int EEoffset, char *DynamicPhoneNumber)
 * Input: EEoffset - this is the location expected for the floats 
 *                   from which to recover saved phone number value
 *        DynamicPhoneNumber - the string which should be set from the values in EEPROM
 * 
 * Output: None
 * Overview: A phone number is expected to be saved in two locations in EEPROM
 *           these values are recovered and converted back to a single string.
 *           the + is added to the start of the phone number
 * TestDate: 8/11/2018 RKF
 ********************************************************************/
void EEPROMtoPhonenumber(int EEoffset, char *DynamicPhoneNumber){
    char PhoneNumber[8];    
    EEProm_Read_Float(EEoffset, &EEFloatData);
    floatToString(EEFloatData, PhoneNumber);
    DynamicPhoneNumber[0]=0;
    concat(DynamicPhoneNumber,"+");
    concat(DynamicPhoneNumber, PhoneNumber);
    if((stringLength(PhoneNumber))< 7){ }// Just the one float CountryCode
    else{
        EEProm_Read_Float(EEoffset+1, &EEFloatData);
        floatToString(EEFloatData, PhoneNumber);
        concat(DynamicPhoneNumber, PhoneNumber);
    }
}
/*********************************************************************
 * Function: OneTimeStatusReport()
 * Input: None - must be called after the readSMSMessage
 *                ReceiveTextMsg has the message string in it 
 *                SendingPhoneNumber has the phone number of the sender
 * Output: None
 * Overview: An SMS message was received (in the string ReceiveTextMsg)
 *           with the AWI prefix indicating that
 *           system status information should be sent to the phone number
 *           sending the message.
 * 
 *           Battery: 3.7
 *           Network: 5 (number reported by SIM) bad/weak/good
 *           Date: XX
 *           Month: XXX
 *           Time: HR:MIN
 * 
 * Note: Library
 * TestDate: not tested
 ********************************************************************/
void OneTimeStatusReport(){
    int success = 0;
    char reportValueString[20];
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
            concat(localMsg,"Battery: ");
            floatToString(batteryLevel(), reportValueString); 
            concat(localMsg,reportValueString);
            concat(localMsg,"\n Network: ");
            readFonaSignalStrength();
            concat(localMsg,SignalStrength); //change this to interpreting as bad,weak,good
            concat(localMsg,"\n Date: ");
            floatToString(BcdToDec(getDateI2C()),reportValueString);
            concat(localMsg,reportValueString);
            concat(localMsg,"\n Month: ");
            floatToString(BcdToDec(getMonthI2C()),reportValueString);
            concat(localMsg,reportValueString);
            concat(localMsg,"\n Time: ");
            //other places I add 1313 to hour
            floatToString(hour,reportValueString);
            concat(localMsg,reportValueString);
            concat(localMsg,":");
            floatToString(minuteVTCC,reportValueString);
            concat(localMsg,reportValueString);
            
            
            sendTextMessage(localMsg);   //note, this now returns 1 if successfully sent to FONA
            // I think I  need to wait until it is sent.  See the Daily Report code.
            char CmdMatch[]="CMGS:";  // we only look for letters in reply so exclude leading +
            ReadSIMresponse(CmdMatch); // this looks for the response from the FONA that the message has been received
            phoneNumber = MainphoneNumber;
        }
    }
    // Should we wait for the message to be sent before trying to work with the FONA?
    delayMs(40);
    
}
/*********************************************************************
 * Function: int SetFONAtoTextMode(void);
 * Inputs: None
 * Output: 1 if the command to go to Text mode received a response of OK
 *         0 if there was no response or not OK
 * Overview: Call this before sending any other AT commands to make sure 
 *           the FONA is in text mode.  This waits for the echo
 *           of the command and an OK, or for a timeout waiting for a reply
 * Note: Library
 * TestDate: 6/23/2018
 ********************************************************************/
int SetFONAtoTextMode(void){
    int success = 0; 
    char CmdMatch[]="OK";
    IFS0bits.U1RXIF = 0; // Always reset the interrupt flag
    U1STAbits.OERR = 0;  //clear the overrun error bit to allow new messages to be put in the RXREG FIFO
                         // This clears the RXREG FIFO
    IEC0bits.U1RXIE = 1;  // enable Rx interrupts
    NumCharInTextMsg = 0; //Point to the start of the Text Message String
    ReceiveTextMsgFlag = 0; //clear for the next message
       // AT+CMGF=1  //set the mode to text 
    sendMessage("AT+CMGF=1\r\n"); //sets to text mode

    TMR1 = 0; // start timer for max 260 (100 command echo + 160 message) characters
    while(TMR1<longest_wait){  }
    char *MsgPtr;
    int msgLength=strlen(ReceiveTextMsg);
    MsgPtr = ReceiveTextMsg;// See if message contains the expected reply
    while(MsgPtr < ReceiveTextMsg+msgLength-1){
        if(strncmp(CmdMatch, MsgPtr,2)==0){
            success = 1;
        }
        MsgPtr++;
    }    
    return success;

}
/*********************************************************************
 * Function: int AreThereTextMessagesToRead(void);
 * Inputs: None
 * Output: 0 if there are no messages waiting
 *         # number of unread messages
 *         Also changes a Global variable MaxSMSmsgSize to reflect the total number of 
 *         SMS slots available to hold messages.  This differs depending upon the SIM carrier
 * Overview: sends the command to the FONA board to see how many of the 
 *           30 or more message slots have something in them.  We don't know where
 *           these are, we just know how many
 * Note: Library
 * TestDate: not yet tested
 ********************************************************************/
int AreThereTextMessagesToRead(void) 
{
    //int longest_wait = 2650;
    int number_of_unread_messages = 0;
    int success = 0;
    success = SetFONAtoTextMode();
    if(success ==1){
         //AT+CPMS="SM" //Specifies that we are working with the message storage on the SIM card
        IFS0bits.U1RXIF = 0; // Always reset the interrupt flag
        U1STAbits.OERR = 0;  //clear the overrun error bit to allow new messages to be put in the RXREG FIFO
                         // This clears the RXREG FIFO
        NumCharInTextMsg = 0; // Point to the start of the Text Message String
        ReceiveTextMsgFlag = 0; //clear for the next message
    
        sendMessage("AT+CPMS=\"SM\"\r\n"); 
        TMR1 = 0; // start timer for max 160characters
        while(TMR1<longest_wait){  } // Read the command echo and the reply
    
        // The response is now in the array ReceiveTextMst
        // The response looks like  +CPMS: 0,30,0,30,0,30 meaning there are 0 messages in the 30 slots
        char *MsgPtr;
        char unread_msg[3];
        unread_msg[0]=0;
        int msgLength=strlen(ReceiveTextMsg);
        MsgPtr = ReceiveTextMsg;
        // Get the number of messages
        while(!((*MsgPtr > 0x2f)&&(*MsgPtr < 0x3a))&&(MsgPtr < ReceiveTextMsg+msgLength-1)){ // skip non-numbers 
            MsgPtr++;
        }
        while((((*MsgPtr > 0x2f)&&(*MsgPtr < 0x3a)))&&(MsgPtr < ReceiveTextMsg+msgLength-1)){ // accept numbers
            strncat(unread_msg, MsgPtr,1); // read the number of waiting messages
            MsgPtr++;
        }
        number_of_unread_messages = atoi(unread_msg);
        //Now see how many message slots there are (this varies with SIM card)
        unread_msg[0]=0;
        MsgPtr++;
        while(!((*MsgPtr > 0x2f)&&(*MsgPtr < 0x3a))&&(MsgPtr < ReceiveTextMsg+msgLength-1)){ // skip non-numbers 
            MsgPtr++;
        }
        while((((*MsgPtr > 0x2f)&&(*MsgPtr < 0x3a)))&&(MsgPtr < ReceiveTextMsg+msgLength-1)){ // accept numbers
            strncat(unread_msg, MsgPtr,1); // read the number of waiting messages
            MsgPtr++;
        }
        MaxSMSmsgSize = atoi(unread_msg);
    } 
    return number_of_unread_messages; 
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
 * Output: The number of messages prior to clearing this one
 * Overview: sends the command to the FONA board which clears its buffer of 
 *           messages.  This includes messages that have not yet been read
 * Note: Library
 * TestDate: not yet tested
 ********************************************************************/
int ClearReceiveTextMessages(int MsgNum, int ClrMode) 
{
    int number_of_unread_messages = 0;
    char MsgNumString[20];
    char ClrModeString[20];
    char MessageString[20];
    int success = 0;
    longToString(MsgNum, MsgNumString);
    longToString(ClrMode, ClrModeString);
        
    success = SetFONAtoTextMode();
    if(success == 1){
        //AT+CPMS="SM" //Specifies that we are working with the message storage on the SIM card
        IFS0bits.U1RXIF = 0; // Always reset the interrupt flag
        U1STAbits.OERR = 0;  //clear the overrun error bit to allow new messages to be put in the RXREG FIFO
                             // This clears the RXREG FIFO
        NumCharInTextMsg = 0; // Point to the start of the Text Message String
        ReceiveTextMsgFlag = 0; //clear for the next message
        sendMessage("AT+CPMS=\"SM\"\r\n"); 
        //We don't use the reply yet but wait for it to finish
        TMR1 = 0; // start timer for max 260characters (100command header + 160 message) ? what about the 25ms gap between echo and response?
        while(TMR1<longest_wait){  }
     
        // AT+CMGD=MsgNum,ClrMode  This is the delete command 
        ReceiveTextMsgFlag = 0; //clear for the next message
        NumCharInTextMsg = 0; // Point to the start of the Text Message String
        MessageString[0]=0; //Clear the message string    
        concat(MessageString, "AT+CMGD=");
        concat(MessageString, MsgNumString);
        concat(MessageString, ",");
        concat(MessageString, ClrModeString);
        concat(MessageString, "\r\n");
        sendMessage(MessageString); 
        delayMs(250);// Delay while the FONA replies OK
        sendMessage("\x1A"); // this terminates an AT SMS command
    
        // We should have just deleted all READ messages.
        // Use the AT+CPMS="SM" command to see how many UNREAD messages remain
        number_of_unread_messages = AreThereTextMessagesToRead();
        //IFS0bits.U1RXIF = 0; // Always reset the interrupt flag
        //U1STAbits.OERR = 0;  //clear the overrun error bit to allow new messages to be put in the RXREG FIFO
                             // This clears the RXREG FIFO
        //NumCharInTextMsg = 0; // Point to the start of the Text Message String
        //ReceiveTextMsgFlag = 0; //clear for the next message
        //sendMessage("AT+CPMS=\"SM\"\r\n"); 
        //TMR1 = 0; // start timer for max 260characters (100command header + 160 message) 
        //while(TMR1<longest_wait){  }
        // The response is now in the array ReceiveTextMsg
        // The response looks like  +CPMS: 0,30,0,30,0,30 meaning there are 0 messages in the 30 slots
        //char *MsgPtr;
        //int msgLength=strlen(ReceiveTextMsg);
        //MsgPtr = ReceiveTextMsg;// Skip over the +CPMS: part
        //while(((*MsgPtr < 0x30)||(*MsgPtr > 0x39))&&(MsgPtr < ReceiveTextMsg+msgLength-1)){ // skip non-numbers
        //    MsgPtr++;
       // }
        // Get the number of messages
        //char unread_msg[3];
        //int i = 0;
        //unread_msg[0]=0;
        //while(((*MsgPtr > 0x2f)&&(*MsgPtr < 0x3a))&&(MsgPtr < ReceiveTextMsg+msgLength-1)&&(i<3)){ // accept numbers
        //    strncat(unread_msg,MsgPtr,1);
        //    MsgPtr++;
        //    i++;
        //}
        //number_of_unread_messages = atoi(unread_msg);
        // End of see how many    
    }
    
    return number_of_unread_messages;
}
/*********************************************************************
 * Function: sendTextMessage()
 * Input: String
 * Output: 1 if the SMS message was sent to the FONA.  This does not mean it was actually transmitted
 * Overview: sends an SMS Text Message to which ever phone number is in the variable 'phoneNumber'
 *           
 * Note: Library
 * TestDate: 6/28/2018
 ********************************************************************/
int sendTextMessage(char message[160]) 
{
    int success = 0;
 //   The SIM should have been turned on prior to getting here;
    
    
    success = SetFONAtoTextMode(); //sets to text mode
    if(success){
        success = sendMessage("AT+CMGS=\""); //beginning of allowing us to send SMS message
        success = sendMessage(phoneNumber);
        success = sendMessage("\"\r\n"); //middle of allowing us to send SMS message
        delayMs(250);
        success = sendMessage(message);
        delayMs(250);
        success = sendMessage("\x1A"); // this terminates an AT SMS command
    }
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
        EEFloatData = 131300 + hour; // if NaN saved for date,month insert date = 13 month = 13
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
 * Function: int SendSavedDailyReports2(void)
 * Input: none - Assumes the FONA is already ON
 * Output: the number of daily reports still waiting to be sent
 * Note:  Sends saved daily reports to the MainphoneNumber.  Messages are sent
 *        newest first.  As long as the network is available and messages are
 *        being sent, older saved messages not able to be sent before are sent 
 * 
 * TestDate: 
 ********************************************************************/
int SendSavedDailyReports(void){
    int ready = 0; 
    int num_saved_messages;
    int message_position;
    int effective_address;  //EEProm position.  We assume each position is a float and start at 0
      
    // Send a daily report if we have network connection and there are messages to send
    EEProm_Read_Float(DailyReportEEPromStart, &EEFloatData);// Read EEPROM address, as of now 21, which contains the number of messages already saved
    num_saved_messages = EEFloatData;
    num_unsent_daily_reports = num_saved_messages;  //as long as there have been 5 or less saved messages, these are the same
    if(num_saved_messages > 5){
            num_unsent_daily_reports = 5;  //This is the maximum number of messages saved in our daily report buffer
    }
    if(num_saved_messages > 0){
        ready = tryToConnectToNetwork(); // This will try 7 times to connect to the network   
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
    }
    // after we are done sending, update the number of messages still waiting to be sent
    // if there is no problem with the network, this will be zero
    EEFloatData = num_saved_messages;  //Update the number of messages in the queue
    EEProm_Write_Float(DailyReportEEPromStart,&EEFloatData);
    return num_unsent_daily_reports;  
}
/*********************************************************************
 * Function: void SendHourlyDiagnosticReport(void)
 * Input: none (assumes that the FONA is turned on)
 * Output: none
 * Overview: Creates a JSON formatted text message relaying
 *          s	sleep hour status ? 0/1, 1 if the system went to sleep since the last report
 *          b	battery voltage at the time the message is sent
 *          r	Number of hours since the last system restart
 *          p	0 - 0 if no reset occurred, if there was a restart, the value is 
 *              the contents of the RCON register.  See Theory of Operation for RCON bits meaning
 *          c	Did the RTCC communicate fine since the last report (or is something hung so it is not responding). 1 = good RTCC communication
 *          t	The hour that the system believes it is when the message is sent (24hr time format)
 *          n	# of times it tried to connect to the network to send the report
 *          x	Is the current time being controlled by the external RTCC or the internal VTCC? 0 = RTCC, 1 = VTCC
 *          w	cell signal strength.  See Theory of Operation for comments on the signal strength reading
 * 
 *  Sends the diagnostic message to the DebugphoneNumber if the network is available.
 *      if the network is not available, the diagnostic message is not saved  
 * TestDate: 
 ********************************************************************/
void SendHourlyDiagnosticReport(void){
    int ready = 0; 
    int numberTries = 0;
    
    if(diagnostic){ // If hourly diagnostic messages are enabled create and send the message
        // Try up to 30 times to get a network connection
        while((!ready)&&(numberTries < 31)){
            ready = CheckNetworkConnection(); 
            numberTries++;            
        }
        if((ready)&&(numberTries < 31)){
            phoneNumber = DebugphoneNumber;
            //batteryFloat = batteryLevel();               
            createDiagnosticMessage();
            sendTextMessage(SMSMessage);
            // Check to see if the FONA replies with ERROR or not. We really do 
            // this to make sure the FONA has time to send the message before
            // we shut it off
            char CmdMatch[]="CMGS:";  // we only look for letters in reply so exclude leading +
            ready = ReadSIMresponse(CmdMatch);
            extRtccTalked = 0; // reset the external clock talked bit
            sleepHrStatus = 0; // reset the slept during that hour
            EEProm_Write_Float(DiagnosticEEPromStart,&sleepHrStatus); // Save to EEProm
            resetCause = 0; // reset the reset cause bits
            EEProm_Write_Float(DiagnosticEEPromStart + 1,&resetCause); // Save to EEProm
        }
    }
    timeSinceLastRestart++; // increase the # of hours since the last system restart (cleared during initialization)

    phoneNumber = MainphoneNumber; //Put the phone number back to Upside 
}
/*********************************************************************
 * Function: void readFonaSignalStrength(void)
 * Input: none
 * Output: none
 * Overview:  Asks the FONA what the current cellular signal strength is and then put the value
 *           in an array. The value saved is a value corresponding to the signal strength, not the signal
 *           strength itself.
 *           2-9 = Marginal (some say you can do SMS at 5, voice at 10 and data at 20)
 *           10-14 = OK
 *           15-19 = Good
 *           20-30 = Excellent
 * TestDate: 6/28/2018 RKF
 ********************************************************************/

void readFonaSignalStrength(void) {
    strncpy(SignalStrength,"0",1); //Set the signal strength to a default of 0
    int success = 0;
    success = SetFONAtoTextMode();
    
    if(success == 1){
        IFS0bits.U1RXIF = 0; // Always reset the interrupt flag
        U1STAbits.OERR = 0;  //clear the overrun error bit to allow new messages to be put in the RXREG FIFO
                         // This clears the RXREG FIFO
        NumCharInTextMsg = 0; // Point to the start of the Text Message String
        ReceiveTextMsgFlag = 0; //clear for the next message
        ReceiveTextMsg[0]=0;  //Reset the receive text message array
        sendMessage("AT+CSQ\r\n"); //Request network signal strength
        
        TMR1 = 0; // start timer for max 260characters (100command header + 160 message) this should also cover the  approx 25ms gap between echo and response
        while(TMR1<longest_wait){  }
        IEC0bits.U1RXIE = 0;  // disable Rx interrupts
        
        // Here is where I'd like to extract information if there is something there
        char *MsgPtr;
        MsgPtr = ReceiveTextMsg; //set the pointer to the response
        int msgLength=strlen(ReceiveTextMsg);
        while((*MsgPtr != ':')&&(MsgPtr < ReceiveTextMsg+msgLength-1)){ //advance pointer to the colon
            MsgPtr++;
        }
        MsgPtr++;
        if(MsgPtr<ReceiveTextMsg+msgLength-1){
            SignalStrength[0]=0; // clear the signal strength array
        }
        while((*MsgPtr != ',')&&(MsgPtr < ReceiveTextMsg+msgLength-1)){ //when we reach a comma, we have read over the signal strength and should stop reading
            strncat(SignalStrength, MsgPtr, 1); //save signal strength number in array
            MsgPtr++;
        }
    }
}

/*********************************************************************
 * Function: void createDiagnosticMessage(void)
 * Input: none
 * Output: none
 * Overview:  Creates the message to be sent in the hourly diagnostic message. 
 * TestDate: 
 ********************************************************************/

void createDiagnosticMessage(void) {
    char LocalString[20]; 
    SMSMessage[0] = 0; //reset SMS message array to be empty
    LocalString[0] = 0;
    
    concat(SMSMessage, "(\"t\":\"d\",\"d\":(\"s\":");
    EEProm_Read_Float(DiagnosticEEPromStart, &EEFloatData);
    floatToString(EEFloatData, LocalString); //populates the sleepHrStatusString with the value from EEPROM
    concat(SMSMessage, LocalString);
    concat(SMSMessage, ",\"b\":");
    //batteryFloat = batteryLevel();  
    //floatToString(batteryFloat, LocalString); //latest battery voltage
    floatToString(batteryLevel(), LocalString); //latest battery voltage
    concat(SMSMessage, LocalString);
    concat(SMSMessage, ",\"r\":");
    floatToString(timeSinceLastRestart, LocalString); // hours since the system restarted
    concat(SMSMessage, LocalString);
    concat(SMSMessage, ",\"p\":");
    floatToString(resetCause, LocalString); //0 if no reset occurred, else the RCON register bit number that is set is returned
    concat(SMSMessage, LocalString);
    concat(SMSMessage, ",\"c\":");
    floatToString(extRtccTalked, LocalString); // if the external rtcc responded the last time the hour was checked
    concat(SMSMessage, LocalString);
    concat(SMSMessage, ",\"t\":");
    //floatToString(LocalFloat, LocalString); // what it thinks the hour is (LocalFloat = hour)
    floatToString(hour, LocalString); // what it thinks the hour is (LocalFloat = hour)
    concat(SMSMessage, LocalString);
    concat(SMSMessage, ",\"n\":");
    floatToString(numberTries, LocalString); // number of tries to connect
    concat(SMSMessage, LocalString);
    concat(SMSMessage, ",\"x\":");
    floatToString(extRTCCset, LocalString); // To keep track if the VTCC time was used to set the external RTCC
    concat(SMSMessage, LocalString);
    concat(SMSMessage, ",\"w\":");
    readFonaSignalStrength();
    concat(SMSMessage, SignalStrength); // The cellular signal
    
    concat(SMSMessage, ">))");
}   

/*********************************************************************
 * Function: void checkDiagnosticStatus(void)
 * Input: none
 * Output: none
 * Overview:  Check whether diagnostic messages are enabled
 * TestDate: 
 ********************************************************************/

//void checkDiagnosticStatus(void){
//    float LocalFloatUpper;
//    float LocalFloatLower;
//    char LocalStringUpper[15]; 
//    char LocalStringLower[15]; 
//    
//    EEProm_Read_Float(DiagnosticEEPromStart + 2, &EEFloatData); // lower byte of the phone number
//    LocalFloatLower = EEFloatData;
//    EEProm_Read_Float(DiagnosticEEPromStart + 3, &EEFloatData); // upper byte of the phone number
//    LocalFloatUpper = EEFloatData;
//    if ((LocalFloatLower != 0) || (LocalFloatUpper != 0)) {
//        diagnostic = 1;
//        floatToString(LocalFloatLower, LocalStringLower);
//        floatToString(LocalFloatUpper, LocalStringUpper);
//        concat(DebugphoneNumber, LocalStringUpper);
//        concat(DebugphoneNumber, LocalStringLower);
//    }
//    else {
//        diagnostic = 0;
//    }
//}

/*********************************************************************
 * Function: void CheckIncommingTextMessages(void)
 * Input: none
 * Output: none
 * Overview:  This is called if the external Diagnostic PCB is 
 *            plugged in and each hour to see if there have been any messages sent 
 *            to the system which require action. 
 * TestDate: 
 ********************************************************************/
void CheckIncommingTextMessages(void){
    if(!FONAisON){
        turnOnSIM();
        delayMs(10000); // Give FONA time to get messages from the network
    }
    if(FONAisON){
        int msg_remaining = 0;
        // Try for up to three minutes
        int num_tries = 0;
        while((!msg_remaining)&&(num_tries < 36)){
            msg_remaining = AreThereTextMessagesToRead();
            delayMs(5000); //wait for 5 seconds
            num_tries++;    
            ClearWatchDogTimer(); 
        }
        //msg_remaining = AreThereTextMessagesToRead();
        if(msg_remaining){ //Only read messages if there are any to read
            int msgNum = 1;
            while((msgNum <= MaxSMSmsgSize)&&(msg_remaining)){
                readSMSMessage(msgNum);
                interpretSMSmessage();
                msg_remaining = ClearReceiveTextMessages(msgNum, 2); // Clear the message just read from the message storage area                    
                msgNum++;
            }
        }
    }
        
}
