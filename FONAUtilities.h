
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
extern char MainphoneNumber[15]; // Upside Wireless
extern char DebugphoneNumber[15]; // Number for the Black Phone
extern char SendingPhoneNumber[15]; //Number for the phone that sent the system a message
extern char CountryCode[]; // The country code for the country where the pump is installed
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
extern int longest_wait; // the maximum ticks of TMR1 that we wait for a text message
extern int diagPCBpluggedIn; // Used to keep track of whether diagnostic PCB is plugged in or not
extern int MaxSMSmsgSize;  // number of slots available on SIM to store text messages
extern int FONAisON; //Keeps track of whether the FONA has been turned on

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
void sendDebugMessage(char message[50], float value); //sends message meant to be read by diagnostic equipment not to be sent by FONA
void readSMSMessage(int msgNum); //Called as a part of the hourly tasks.  Reads msgNum msg and puts it in ReceiveTextMsg
int SetFONAtoTextMode(void); // Used before other AT commands to put FONA in TEXT mode
void interpretSMSmessage(void); //Decide which AW command was received 
void updateClockCalendar(void); //take action when an AW command calls for a change to the clock calendar
void enableDiagnosticTextMessages(void); //Enables (1) or disable (0) hourly diagnostic messages
int ClearReceiveTextMessages(int MsgNum, int ClrMode); 
void CreateNoonMessage(int);
void CreateAndSaveDailyReport(void);
int SendSavedDailyReports(void);
void SendHourlyDiagnosticReport(void);
void createDiagnosticMessage(void);
void checkDiagnosticStatus(void); //NOT CURRENTLY USED
void readFonaSignalStrength(void); // Asks the FONA for the strength of the network signal
void OneTimeStatusReport(void); // reports various system status information one time to requesting phone number
int AreThereTextMessagesToRead(void); //Checks # of waiting messages in FONA and total number of message slots for this SIM card
void CheckIncommingTextMessages(void); // This routine is called to read and process text messages which have been received
void ChangeCountryCode(void); // Changes the saved variable Country Code
void UpdateSendingPhoneNumber(void); //If the sending number is local, remove country code
void ChangeDailyReportPhoneNumber(void); //Changes the number used for daily reports MainphoneNumber[]




#endif	/* FONAUTILITIES_H*/


