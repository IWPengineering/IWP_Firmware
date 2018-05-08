
/*
 * File:   FONAUtilities.h
 * Author: rfish
 *
 * Created on November 15, 2017, 5:17 PM
 */


// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef FONAUTILITIES_H
#define	FONAUTILITIES_H

#include <xc.h> // include processor files - each processor file is guarded.  

// ****************************************************************************
// *** Constants **************************************************************
// ****************************************************************************



// ****************************************************************************
// *** Global Variables *******************************************************
// ****************************************************************************
extern char MainphoneNumber[]; // Upside Wireless
extern char DebugphoneNumber[]; // Number for the Black Phone
extern char SendingPhoneNumber[]; //Number for the phone that sent the system a message
//extern char* phoneNumber; // Number Used to send text message report (daily or hourly)
extern char* phoneNumber; // Number Used to send text message report (daily or hourly)
extern int LeaveOnSIM;  // this is set to 1 when an external message says to not shut off the SIM
extern char FONAmsgStatus[]; //A string indicating if the message has been sent/read
extern char SignalStrength[]; //hold the values of the signal strength
extern char SMSMessage[]; //A string used to hold all SMS message sent with FONA
extern char ReceiveTextMsg[];  //String used to hold text messages received from FONA
extern int NumCharInTextMsg; //Number of characters in the received text message 
extern char ReceiveTextMsgFlag; // Set to 1 when a complete text message has been received
extern int noon_msg_sent;  //set to 1 when noon message has been sent
extern int hour_msg_sent;  //set to 1 when hourly message has been sent
extern int num_unsent_daily_reports; //this is the number of saved daily reports that have not been sent

// ****************************************************************************
// *** Function Prototypes ****************************************************
// ****************************************************************************
int turnOffSIM();
int turnOnSIM();
int tryToConnectToNetwork(); //tries to connect to the network for up to 45 seconds
int CheckNetworkConnection(void); //checks to see if the network has been acquired
int ReadSIMresponse(char expected_reply[10]); //called after an AT command is sent to the FONA, looks for a defined response
int sendMessage (char message[160]); //uses UART to send the string in message[]
int sendTextMessage(char message[160]); //transmits UART characters necessary to send an SMS message using AT protocol
void sendDebugMessage(char message[50], float value);
int wasMessageSent(int msgNum); //this is being written and may not be needed
void readSMSMessage(int msgNum); //Called as a part of the hourly tasks.  Reads msgNum msg and puts it in ReceiveTextMsg
void interpretSMSmessage(void); //Right now only understands the AW_C command to change the clock/calendar 
void updateClockCalendar(void); //take action when an AW command calls for a change to the clock calendar
void enableDiagnosticTextMessages(void); //Enables (1) or disable (0) hourly diagnostic messages
void sendDebugTextMessage(char message[160]); //I don't think this is ever used
void ClearReceiveTextMessages(int MsgNum, int ClrMode); 
void CreateNoonMessage(int);
void CreateAndSaveDailyReport(void);
int SendSavedDailyReports(void);
void createDiagnosticMessage(void);
void checkDiagnosticStatus(void);
void readFonaSignalStrength(void); // Asks the FONA for the strength of the network signal




#endif	/* FONAUTILITIES_H*/


