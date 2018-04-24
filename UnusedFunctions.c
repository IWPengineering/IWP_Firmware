/*
 * File:   UnusedFunctions.c
 * Author: rfish
 *
 * Created on November 16, 2017, 7:27 PM
 */


#include "xc.h"
#include "IWPUtilities.h"
#include "Pin_Manager.h"
#include "FONAUtilities.h"
#include "I2C.h"

const int networkPulseWidthThreshold = 0x4E20; // The value to check the pulse width against (about 20000)
int queueCount = 0;
//const int queueLength = 7; //don't forget to change angleQueue to this number also
#define queueLength        7
float angleQueue[queueLength]; // Now we don't have to remember to change it anymore. Just change it once.


void sendTimeMessage(void) {
    char timeHourMessage[20];
    timeHourMessage[0] = 0;
    char timeMinuteMessage[20];
    timeMinuteMessage[0] = 0;
    char timeSecondMessage[20];
    timeSecondMessage[0]=0;
    char timeMessage[160];
    timeMessage[0] = 0;
    char timeWeekMessage[20];
    timeWeekMessage[0] = 0;
    char timeDayMessage[20];
    timeDayMessage[0] = 0;
    char timeMonthMessage[20];
    timeMonthMessage[0]=0;
    char timeYearMessage[20];
    timeYearMessage[0] = 0;


    longToString(BcdToDec(getTimeI2C(0x02, 0x3f, 23)), timeHourMessage);
    longToString(BcdToDec(getTimeI2C(0x01, 0x7f, 59)), timeMinuteMessage);
    longToString(BcdToDec(getSecondI2C()), timeSecondMessage);
    longToString(BcdToDec(getTimeI2C(0x06, 0xff, 99)), timeYearMessage);
    longToString(BcdToDec(getWkdayI2C()), timeWeekMessage);
    longToString(BcdToDec(getTimeI2C(0x04, 0x3f, 31)), timeDayMessage);
    longToString(BcdToDec(getTimeI2C(0x05, 0x1f, 12)), timeMonthMessage);

    concat(timeMessage, timeHourMessage);
    concat(timeMessage, ":");
    concat(timeMessage, timeMinuteMessage);
    concat(timeMessage, ":");
    concat(timeMessage, timeSecondMessage);
    concat(timeMessage, "   ");
    concat(timeMessage, "WeekDay: ");
    concat(timeMessage, timeWeekMessage);
    concat(timeMessage, "  ");
    concat(timeMessage, timeMonthMessage);
    concat(timeMessage, "/");
    concat(timeMessage, timeDayMessage);
    concat(timeMessage, "/");
    concat(timeMessage, timeYearMessage);
    concat(timeMessage, " \r\n");
    sendMessage(timeMessage);
}

/*********************************************************************
 * Function: initializeQueue()
 * Input: float
 * Output: None
 * Overview: Set all values in the queue to the intial value
 * Note: Library
 * TestDate: 06-20-2014
 ********************************************************************/
void initializeQueue(float value) {
    int i = 0;
    for (i = 0; i < queueLength; i++) {
        angleQueue[i] = value;
    }
}

/*********************************************************************
 * Function: pushToQueue()
 * Input: float
 * Output: None
 * Overview: Shift values down one
 * Note: Library
 * TestDate: 06-20-2014
 ********************************************************************/
void pushToQueue(float value) {
    int i = 0;
    for (i = 0; i < queueLength - 1; i++) {
        angleQueue[i] = angleQueue[i + i];
    }
    // Insert the value at the end of the queue
    angleQueue[queueLength - 1] = value;
}

/*********************************************************************
 * Function: queueAverage()
 * Input: None
 * Output: float
 * Overview: Takes the average of the queue
 * Note: Library
 * TestDate: NOT TESTED
 ********************************************************************/
float queueAverage() {
    float sum = 0;
    int i = 0;
    // Sum up all the values in the queue
    for (i = 0; i < queueLength; i++) {
        sum += angleQueue[i];
    }
    // Returns the average after converting queueLength to a float
    return sum / (queueLength * 1.0);
}

/*********************************************************************
 * Function: queueDifference()
 * Input: None
 * Output: float
 * Overview: Returns the difference between the last and first numbers in the queue
 * Note: Library
 * TestDate: NOT TESTED
 ********************************************************************/
float queueDifference() {
    return angleQueue[queueLength - 1] - angleQueue[0];
}
