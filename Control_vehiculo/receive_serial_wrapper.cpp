/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>
#ifndef MATLAB_MEX_FILE

#include <Arduino.h>

volatile float av = 0;
volatile float aw = 0;
volatile unsigned char am = 0;

#endif
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Output function
 *
 */
extern "C" void receive_serial_Outputs_wrapper(real32_T *v,
			real32_T *w,
			uint8_T *m,
			const real_T *xD)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
/* This sample sets the output equal to the input
      y0[0] = u0[0]; 
 For complex signals use: y0[0].re = u0[0].re; 
      y0[0].im = u0[0].im;
      y1[0].re = u1[0].re;
      y1[0].im = u1[0].im;
 */
    if (xD[0] == 1)
    {
#ifndef MATLAB_MEX_FILE

        if (Serial.available() > 0)
        {
            String str = Serial.readStringUntil('\n');
            String data1 = "";
            String data2 = "";
            String data3 = "";
            int var = 0;
            for (int i = 0; i < str.length(); i++)
            {
                if ((var == 0) and (str[i] == 'v'))
                {
                    var = 1;
                }
                else if (var == 1)
                {
                    if (str[i] == 'w')
                    {
                        var = 2;
                    }
                    else
                    {
                        data1 = data1 + str[i];
                    }
                }
                else if (var == 2)
                {
                    if (str[i] == 'm')
                    {
                        var = 3;
                    }
                    else
                    {
                        data2 = data2 + str[i];
                    }
                }
                else if (var == 3)
                {
                    if (str[i] == ' ')
                    {
                        var = 4;
                    }
                    else
                    {
                        data3 = data3 + str[i];
                    }
                }
            }
            v[0] = data1.toFloat();
            w[0] = data2.toFloat();
            m[0] = data3.toInt();
            av = v[0];
            aw = w[0];
            am = m[0];
        }
        else
        {
            v[0] = av;
            w[0] = aw;
            m[0] = am;
        }

#endif
    }
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
 * Updates function
 *
 */
extern "C" void receive_serial_Update_wrapper(real32_T *v,
			real32_T *w,
			uint8_T *m,
			real_T *xD)
{
/* %%%-SFUNWIZ_wrapper_Update_Changes_BEGIN --- EDIT HERE TO _END */
/*
 * Code example
 *   xD[0] = u0[0];
 */
    if (xD[0] != 1)
    {
#ifndef MATLAB_MEX_FILE

        Serial.begin(115200);
        xD[0] = 1;

#endif
    }
/* %%%-SFUNWIZ_wrapper_Update_Changes_END --- EDIT HERE TO _BEGIN */
}

