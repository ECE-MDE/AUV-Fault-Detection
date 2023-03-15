//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides communication protocols for the Digi Radio 
//==============================================================================

#ifndef DIGI_H
#define DIGI_H

typedef struct Tracking
{
    double lat;     //deg
    double lon;     //deg
    float v;        //m/s
    float course;   //deg
    int now;        //s
    int age;        //s
} Tracking;


#endif // DIGI_H
