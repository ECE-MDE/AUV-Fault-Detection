//==============================================================================
// Description: Calculates the density of water
//==============================================================================

#ifndef DENSITY_CALCULATOR_H
#define DENSITY_CALCULATOR_H

// Refer to the following:
// https://link.springer.com/content/pdf/bbm%3A978-3-319-18908-6%2F1.pdf


#include <math.h>
#include <stdio.h>

//==============================================================================
//                            FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        VALIDITY_CHECK 
// Description: Checks if conditions are valid for density calculation
// Arguments:   - cond: Conductivity in mS / cm
//              - temp: Temperature in Celsius
//              - salinity: PSU
// Returns:     true if valid
//------------------------------------------------------------------------------
bool VALIDITY_CHECK(double cond, double temp, double salinity)
{

    if (cond < 0)
        return false;

    if (temp < 0 || temp > 40)
        return false;

    if (salinity < 0 || salinity > 42)
        return false;

    return true;
}
//------------------------------------------------------------------------------
// Name:        SALINITY_CALCULATOR
// Description: Calculates the salinity
// Arguments:   - cond: Conductivity in mS / cm
//              - temp: Temperature in Celsius
//              - press_dbar: Pressure in dbar
// Returns:     Double that represents the salinity in psu
//              (psu and ptt are nearly equivalent)
//------------------------------------------------------------------------------
double SALINITY_CALCULATOR(double cond, double temp, double press_dbar)
{
    double conductivity_datum = 42.914;
    double a0 = 0.008;
    double a1 = -0.1692;
    double a2 = 25.3851;
    double a3 = 14.0941;
    double a4 = -7.0261;
    double a5 = 2.7081;

    double b0 = 0.0005;
    double b1 = -0.0056;
    double b2 = -0.0066;
    double b3 = -0.0375;
    double b4 = 0.0636;
    double b5 = -0.0144;

    double c0 = 0.6766097;
    double c1 = 0.0200564;
    double c2 = 0.0001104259;
    double c3 = -6.9698 * 1.0e-7;
    double c4 = 1.0031 * 1.0e-9;

    double d1 = 0.03426;
    double d2 = 0.0004464;
    double d3 = 0.4215;
    double d4 = -0.003107;

    double e1 = 0.0000207;
    double e2 = -6.37 * 1.0e-10;
    double e3 = 3.989 * 1.0e-15;

    double k = 0.0162;
    double T68 = temp * 1.00024;
    double littleRt = c0 + T68 * (c1 + T68 * (c2 + T68 * (c3 + T68 * c4)));
    double cRatio = cond / conductivity_datum;
    double rp_numerator = press_dbar * (e1 + press_dbar * (e2 + press_dbar *
        e3));
    double rp_denominator = (1 + d1 * T68 + d2 * T68 *T68 + d3 * cRatio + d4 *
        T68 * cRatio);
    double rp = 1 + rp_numerator / rp_denominator;
    double bigRt = cRatio / (rp * littleRt);
    double sqrtBigRt = sqrt(bigRt);

    double s = a0 + sqrtBigRt * (a1 + sqrtBigRt * (a2 + sqrtBigRt * (a3 +
        sqrtBigRt * (a4 + sqrtBigRt * a5))));
    double deltaS = (T68 - 15) / (1 + k * (T68 - 15)) * (b0 + sqrtBigRt * (b1 +
        sqrtBigRt * (b2 + sqrtBigRt * (b3 + sqrtBigRt * (b4 + sqrtBigRt * b5)))
        )); 

    return s + deltaS;
}

//------------------------------------------------------------------------------
// Name:        SMOW_CALCULATOR
// Description: Calculates the Standard Mean Ocean Water density
// Arguments:   - temp: Temperature in Celsius 
// Returns:     Double that represents the SMOW
//------------------------------------------------------------------------------
double SMOW_CALCULATOR(double temp)
{
    double a0 = 999.842594;
    double a1 = 6.793953 * pow(10.0, -2.0);
    double a2 = -9.095290 * pow(10.0, -3.0);
    double a3 = 1.001685 * pow(10.0, -4.0);
    double a4 = -1.120083 * pow(10.0, -6.0);
    double a5 = 6.536332 * pow(10.0, -9.0);

    return a0 + a1 * temp + a2 * pow(temp, 2.0) + a3 * pow(temp, 3.0) + 
        a4 * pow(temp, 4.0) + a5 * pow(temp, 5.0);
}

//------------------------------------------------------------------------------
// Name:        RHO_ST0 
// Description: Calculates the density of ocean water at 0 pressure
// Arguments:   - sal: salinity in PSU
//              - temp: Temperature in Celsius 
// Returns:     Double that represents the density at 0 pressure
//------------------------------------------------------------------------------
double RHO_ST0(double sal, double temp)
{
    double b0 = 8.2449 * pow(10.0, -1.0);
    double b1 = -4.0899 * pow(10.0, -3.0);
    double b2 = 7.6438 * pow(10.0, -5.0);
    double b3 = -8.2467 * pow(10.0, -7.0);
    double b4 = 5.3875 * pow(10.0, -9.0);

    double c0 = -5.7246 * pow(10.0, -3.0);
    double c1 = 1.0227 * pow(10.0, -4.0);
    double c2 = -1.6546 * pow(10.0, -6.0);

    double d0 = 4.8314 * pow(10.0, -4.0);

    double B1 = b0 + b1 * temp + b2 * pow(temp, 2.0) + b3 * pow(temp, 3.0) + 
        b4 * pow(temp, 4.0);
    double C1 = c0 + c1 * temp + c2 * pow(temp, 2.0);

    return SMOW_CALCULATOR(temp) + B1 * sal + C1 * pow(sal, 1.5) +
        d0 * pow(sal, 2.0);
}

//------------------------------------------------------------------------------
// Name:        K_ST0
// Description: Calculates compressibility constant at 0 pressure
// Arguments:   - sal: Salinity in PSU
//              - temp: Temperature in Celsius 
// Returns:     Compressibility constant at 0 pressure
//------------------------------------------------------------------------------
double K_ST0(double sal, double temp)
{
    double e0 = 19652.210000;
    double e1 = 148.420600;
    double e2 = -2.327105;
    double e3 = 1.360477 * pow(10.0, -2.0);
    double e4 = -5.155288 * pow(10.0, -5.0);

    double f0 = 54.674600;
    double f1 = -0.603459;
    double f2 = 1.099870 * pow(10.0, -2.0);
    double f3 = -6.167000 * pow(10.0, -5.0);

    double g0 = 7.9440 * pow(10.0, -2.0);
    double g1 = 1.6483 * pow(10.0, -2.0);
    double g2 = -5.3009 * pow(10.0, -4.0);

    double Kw = e0 + e1 * temp + e2 * pow(temp, 2.0) + e3 * pow(temp, 3.0) + e4
        * pow(temp, 4.0);
    double F1 = f0 + f1 * temp + f2 * pow(temp, 2.0) + f3 * pow(temp, 3.0);
    double G1 = g0 + g1 * temp + g2 * pow(temp, 2.0);

    return Kw + F1 * sal + G1 * pow(sal, 1.5);
}

//------------------------------------------------------------------------------
// Name:        K_STP
// Description: Calculates final compressibility constant
// Arguments:   - sal: Salinity in PSU
//              - temp: Temperature in Celsius 
//              - pressure: Pressure of water measured in Bar
// Returns:     Final compressibility constant 
//------------------------------------------------------------------------------
double K_STP(double sal, double temp, double pressure)
{
    double h0 = 3.23990;
    double h1 = 1.43713 * pow(10.0, -3.0);
    double h2 = 1.16092 * pow(10.0, -4.0);
    double h3 = -5.77905 * pow(10.0, -7.0);

    double i0 = 2.28380 * pow(10.0, -3.0);
    double i1 = -1.09810 * pow(10.0, -5.0);
    double i2 = -1.60780 * pow(10.0, -6.0);

    double j0 = 1.91075 * pow(10.0, -4.0);

    double Aw = h0 + h1 * temp + h2 * pow(temp, 2.0) + h3 * pow(temp, 3.0);
    double A1 = Aw + (i0 + i1 * temp + i2 * pow(temp, 2.0)) * sal + j0 *
        pow(sal, 1.5);

    double k0 = 8.50935 * pow(10.0, -5.0);
    double k1 = -6.12293 * pow(10.0, -6.0);
    double k2 = 5.27870 * pow(10.0, -8.0);

    double m0 = -9.9348 * pow(10.0, -7.0);
    double m1 = 2.0816 * pow(10.0, -8.0);
    double m2 = 9.1697 * pow(10.0, -10.0);

    double Bw = k0 + k1 * temp + k2 * pow(temp, 2.0);
    double B2 = Bw + (m0 + m1 * temp + m2 * pow(temp, 2.0)) * sal;

    return K_ST0(sal, temp) + ( A1 * pressure ) + (B2 * pressure
        * pressure);
}

//------------------------------------------------------------------------------
// Name:        DENSITY_CALCULATOR
// Description: Calculates the double
// Arguments:   - cond: Salinity in ppm
//              - temp: Temperature in Celsius 
//              - press_dbar: Pressure of water measured in dBar
// Returns:     Double that represents the density of water in 
//------------------------------------------------------------------------------
double CALCULATE_DENSITY(double cond, double temp, double press_dbar)
{
    double p_b = press_dbar/1.0e1;  //convert dBar to bar
    double sal = SALINITY_CALCULATOR(cond, temp, press_dbar);
    double rho_St0 = RHO_ST0(sal, temp);
    double C = 1 - (p_b / K_STP(sal, temp, p_b));

    return rho_St0 / C;
}

#endif // DENSITY_CALCULATOR_H
