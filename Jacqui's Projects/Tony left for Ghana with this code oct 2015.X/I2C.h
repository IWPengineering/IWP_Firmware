/* Microchip Technology Inc. and its subsidiaries.  You may use this software
 * and any derivatives exclusively with Microchip products.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
 * TERMS.
 */

/*
 * File: I2C.h
 * Author: Ken
 * Comments: Parsing out the I2C functiosn from IWPUtilities.h
 * Revision history:
 */

// This is a guard condition so that contents of this file are not included
// more than once.
#ifndef I2C_H
#define	I2C_H

#include <xc.h> // include processor files - each processor file is guarded.
#include "IWPUtilities.h"

/*
 Public Definitions
 */

/*
 Public Constants
 */

/*
 Private Constants
 */

/*
 Public Variables
 */

/*
 Private Variables
 */

/*
 Public Functions
 */
unsigned int IdleI2C(void);
unsigned int StartI2C(void);
unsigned int StopI2C(void);
void RestartI2C(void);
void NackI2C(void);
void AckI2C(void);
void configI2c(void);
void WriteI2C(unsigned char byte);
unsigned int ReadI2C(void);
void turnOffClockOscilator(void);
int getSecondI2C(void);
int getMinuteI2C(void);
int getHourI2C(void);
int getYearI2C(void);
int getMonthI2C(void);
int getWkdayI2C(void);
int getDateI2C(void);
void setTime(char sec, char min, char hr, char wkday,
        char date, char month, char year);

/*
 Private Functions
 */

/*
 Structs
 */

// Comment a function and leverage automatic documentation with slash star star
/**
    <p><b>Function prototype:</b></p>

    <p><b>Summary:</b></p>

    <p><b>Description:</b></p>

    <p><b>Precondition:</b></p>

    <p><b>Parameters:</b></p>

    <p><b>Returns:</b></p>

    <p><b>Example:</b></p>
    <code>

    </code>

    <p><b>Remarks:</b></p>
 */
// TODO Insert declarations or function prototypes (right here) to leverage
// live documentation

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C
    // linkage so the functions can be used by the c code.

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

