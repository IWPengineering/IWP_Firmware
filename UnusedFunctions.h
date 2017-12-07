/*
 * File:   UnusedFunctions.c
 * Author: rfish
 *
 * Created on November 16, 2017, 7:27 PM
 */


// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef UNUSEDFUNCTIONS_H
#define	UNUSEDFUNCTIONS_H

#include <xc.h> // include processor files - each processor file is guarded.  

// TODO Insert appropriate #include <>

// ****************************************************************************
// *** Constants **************************************************************
// ****************************************************************************

extern const int networkPulseWidthThreshold; // The value to check the pulse width against (about 20000)


void sendTimeMessage(void);
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



#endif	/* UNUSEDFUNCTIONS_h */

