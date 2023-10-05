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
#include <can.h>
#include <SPI.h>
#include <SPI.cpp>
#include <mcp2515.h>
#include <mcp2515.cpp>

 

struct can_frame canMsg;
MCP2515 mcp2515(53);

 

/// Value Limits ///
#define P_MIN -95.500f
#define P_MAX 95.500f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

 

// Matrix components for pack_cmd()
#define IP_IN 0
#define IV_IN 1
#define IKP_IN 2
#define IKD_IN 3
#define IT_IN 4

 

//Matrix components for unpack_reply()
#define SLAVE_ID 0
#define P_OUT 1
#define V_OUT 2
#define T_OUT 3

 

//Motor adress and number of motors
#define N_MOTORES 6

 

//typedef unsigned long __u32;
//typedef __u32 canid_t;
canid_t can_idv[]={0x01,0x02,0x03,0x04,0x05,0x06};


//Set values
volatile unsigned char MODE = 0;
volatile unsigned char mode_ant = 0;
volatile float in[N_MOTORES][5];
volatile float out[N_MOTORES][4];

 

// Measured values
float p_out=0.0f;
float v_out=0.0f;
float t_out=0.0f;

 

// Volatile values
//volatile 
volatile bool STATE = false;



//FUNCTIONS
void EnterMotorMode(canid_t can_idv)
{
  struct can_frame canMsg1;
  canMsg1.can_id  = can_idv;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = 0xFF;
  canMsg1.data[1] = 0xFF;
  canMsg1.data[2] = 0xFF;
  canMsg1.data[3] = 0xFF;
  canMsg1.data[4] = 0xFF;
  canMsg1.data[5] = 0xFF;
  canMsg1.data[6] = 0xFF;
  canMsg1.data[7] = 0xFC;
  mcp2515.sendMessage(&canMsg1);
}

 
void ExitMotorMode(canid_t can_idv)
{
  struct can_frame canMsg1;
  canMsg1.can_id  = can_idv;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = 0xFF;
  canMsg1.data[1] = 0xFF;
  canMsg1.data[2] = 0xFF;
  canMsg1.data[3] = 0xFF;
  canMsg1.data[4] = 0xFF;
  canMsg1.data[5] = 0xFF;
  canMsg1.data[6] = 0xFF;
  canMsg1.data[7] = 0xFD;
  mcp2515.sendMessage(&canMsg1);
}

 
void ZeroPosition(canid_t can_idv)
{
  struct can_frame canMsg1;
  canMsg1.can_id  = can_idv;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = 0xFF;
  canMsg1.data[1] = 0xFF;
  canMsg1.data[2] = 0xFF;
  canMsg1.data[3] = 0xFF;
  canMsg1.data[4] = 0xFF;
  canMsg1.data[5] = 0xFF;
  canMsg1.data[6] = 0xFF;
  canMsg1.data[7] = 0xFE;
  mcp2515.sendMessage(&canMsg1);
}


unsigned int float_to_uint( float x,float x_min, float x_max, int bits){
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int pgg = 0;
  if (bits == 12){
    pgg = (unsigned int) ((x-offset)*4095.0/span);
  }
  if(bits == 16){
    pgg = (unsigned int) ((x-offset)*65535.0/span);
  }
  return pgg;
}


float uint_to_float( unsigned int x_int, float x_min,float x_max, int bits){
  float span = x_max - x_min;
  float offset = x_min;
  float pgg = 0;
  if (bits == 12){
    pgg = ((float)x_int)*span/4095.0+ offset;
  }
  if(bits == 16){
   pgg = ((float)x_int)*span/65535.0+ offset;
  }
  return pgg;
}


void pack_cmd(float in[N_MOTORES][5])
{
/// CAN Command Packet Structure ///
/// 16 bit position command, between -4*pi and 4*pi
/// 12 bit velocity command, between -30 and + 30 rad/s
/// 12 bit kp, between 0 and 500 N-m/rad
/// 12 bit kd, between 0 and 100 N-m*s/rad
/// 12 bit feed forward torque, between -18 and 18 N-m
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]]
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]
  struct can_frame canMsg1;
     /// limit data to be within bounds ///
     
    for(int motor=0; motor<=N_MOTORES-1; motor++){
    
    float p_des = constrain(in[motor][IP_IN], P_MIN, P_MAX);                   
    float v_des = constrain(in[motor][IV_IN], V_MIN, V_MAX);
    float kp = constrain(in[motor][IKP_IN], KP_MIN, KP_MAX);
    float kd = constrain(in[motor][IKD_IN], KD_MIN, KD_MAX);
    float t_ff = constrain(in[motor][IT_IN], T_MIN, T_MAX);
  
     /// convert floats to unsigned ints ///
   unsigned  int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);            
   unsigned  int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
   unsigned  int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
   unsigned  int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
   unsigned  int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

 

   int id = motor;
     /// pack ints into the can buffer ///
     canMsg1.can_id = can_idv[id];
     canMsg1.can_dlc = 8;
     canMsg1.data[0] = p_int>>8;                                       
     canMsg1.data[1] = p_int&0xFF;
     canMsg1.data[2] = v_int>>4;
     canMsg1.data[3] = ((v_int&0xF)<<4)|(kp_int>>8);
     canMsg1.data[4] = kp_int&0xFF;
     canMsg1.data[5] = kd_int>>4;
     canMsg1.data[6] = ((kd_int&0xF)<<4)|(t_int>>8);
     canMsg1.data[7] = t_int&0xff;
     mcp2515.sendMessage(&canMsg1);
    }
}


void unpack_reply(){
  /// CAN Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 5 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]

    /// Save lectures from bus can ///
    unsigned int id = canMsg.data[0];
    unsigned int p_int = (canMsg.data[1]<<8)|canMsg.data[2];
    unsigned int v_int = (canMsg.data[3]<<4)|(canMsg.data[4]>>4);
    unsigned int i_int = ((canMsg.data[4]&0xF)<<8)|canMsg.data[5];
    
    /// convert ints to floats ///
    p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
    v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
    t_out = uint_to_float(i_int, -T_MAX, T_MAX, 12);


    /// Save into the matrix //
    out[id-1][SLAVE_ID]= id;
    out[id-1][P_OUT] = p_out;
    out[id-1][V_OUT] = v_out;
    out[id-1][T_OUT] = t_out; 
    
}


void Send_Receive(float in[N_MOTORES][5], bool MODE)
{
  
  if(MODE==true && STATE==false){
    for(int id=0; id<N_MOTORES; id++){
    EnterMotorMode(can_idv[id]);
    }
    STATE=MODE;
  }
  else if(MODE==false && STATE==true){
    for(int id=0; id<N_MOTORES; id++){
    ExitMotorMode(can_idv[id]);
    }
    STATE=MODE;
   }
  else if(MODE==true && STATE==true){
    pack_cmd(in);
 
    //Check if there´s a message and unpack it
   while(mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
      unpack_reply();
    }
   }
  }

 
# endif
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 5
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
extern "C" void BLDC_SteadyWin_Outputs_wrapper(const real32_T *In1,
			const real32_T *In2,
			const real32_T *In3,
			const real32_T *In4,
			const real32_T *In5,
			const real32_T *In6,
			const uint8_T *Mode,
			real32_T *Out1,
			real32_T *Out2,
			real32_T *Out3,
			real32_T *Out4,
			real32_T *Out5,
			real32_T *Out6,
			const real_T *xD)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
if (xD[0]==1) {
        /* don't do anything for mex file generation */
    # ifndef MATLAB_MEX_FILE
    
       // copia paramatros de entrada y los envia
    
       for (int j=0; j <5; j++)
            in[0][j]=  In1[j];

       for (int j=0; j <5; j++)
            in[1][j]=  In2[j];
       
       for (int j=0; j <5; j++)
            in[2][j]=  In3[j];

       for (int j=0; j <5; j++)
            in[3][j]=  In4[j];
       
       for (int j=0; j <5; j++)
            in[4][j]=  In5[j];

       for (int j=0; j <5; j++)
            in[5][j]=  In6[j];   
      
    
       MODE=Mode[0];
       switch (MODE){
          case 0:
            Send_Receive(in,false);
         break;
         case 1:
             Send_Receive(in,true);
         break;
         case 2:
             if (MODE != mode_ant){
                 
              // pone a cero la posicion logica de los motores
             for(int id=0; id<N_MOTORES; id++){
                 ZeroPosition(can_idv[id]);
                 }
             delay(10);
             }
            Send_Receive(in,true);
         break;
       }
       mode_ant=MODE;
       for (int j=0; j<4; j++)
            Out1[j]=  out[0][j];
       for (int j=0; j<4; j++)
            Out2[j]=  out[1][j];
       for (int j=0; j<4; j++)
            Out3[j]=  out[2][j];
       for (int j=0; j<4; j++)
            Out4[j]=  out[3][j];
       for (int j=0; j<4; j++)
            Out5[j]=  out[4][j];
       for (int j=0; j<4; j++)
            Out6[j]=  out[5][j];
               
         
    # endif
    
}
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
 * Updates function
 *
 */
extern "C" void BLDC_SteadyWin_Update_wrapper(const real32_T *In1,
			const real32_T *In2,
			const real32_T *In3,
			const real32_T *In4,
			const real32_T *In5,
			const real32_T *In6,
			const uint8_T *Mode,
			real32_T *Out1,
			real32_T *Out2,
			real32_T *Out3,
			real32_T *Out4,
			real32_T *Out5,
			real32_T *Out6,
			real_T *xD)
{
/* %%%-SFUNWIZ_wrapper_Update_Changes_BEGIN --- EDIT HERE TO _END */
if (xD[0]!=1) {
    
    /* don't do anything for MEX-file generation */
    # ifndef MATLAB_MEX_FILE
      
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  
  //Serial.println("CAN BUS Shield init ok!");
  
//apaga los motores
  for(int id=0; id<N_MOTORES; id++){
    ExitMotorMode(can_idv[id]);
    }
              
  
  // pone a cero la posicion logica de los motores
  for(int id=0; id<N_MOTORES; id++){
    ZeroPosition(can_idv[id]);
    }
//   
  
# endif
    
    /* initialization done */ 
    xD[0]=1;
}
/* %%%-SFUNWIZ_wrapper_Update_Changes_END --- EDIT HERE TO _BEGIN */
}

