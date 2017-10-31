#include <EEPROM.h>
#include <analogShield.h>
#include <math.h>
#include<SPI.h>  // SPI.h has to be included **after** analogShield.h

#define UNO_ID "interferometer_lock\r\n"
#define ZEROV 32768 //Zero volts
#define V2P5 49512  // 2.5 V

#define N_STEPS 100
#define N_STEPS_DOWN 100

#define PI 3.14159
#define PI2 6.28318

#define MIN_FRINGE_VISIBILITY 27000.

#define MONITOR_ERR_780 1
#define MONITOR_FIT_780 2
#define MONITOR_SET_PHASE_780 3
#define MONITOR_RESIDUE_780 4

#define MONITOR_ERR_423 10
#define MONITOR_FIT_423 11
#define MONITOR_SET_PHASE_423 12

#define MONITOR_ERR_453 13
#define MONITOR_FIT_453 14
#define MONITOR_SET_PHASE_453 15


struct Params {
  int ramp_amplitude;

  float freq_780, freq_423, freq_453;

  float gain_p_780, gain_i_780;
  float gain_p_423, gain_i_423;
  float gain_p_453, gain_i_453;

  float set_phase_780, set_phase_423, set_phase_453;
  int lock_state_780, lock_state_423, lock_state_453;

  int output_offset_423, output_offset_453;
  int monitor_channel;
};

Params params;

int in0, in1, in2, in3;
int out0, out1, out2, out3;
int ramp_mean;
int ramp_offset;

int cycle_up = 0;
int cycle_down = 0;

int ramp_step;
int ramp_step_down;

bool ramp_direction;

int in0_array[N_STEPS];

unsigned long current_time;

float estimation_matrix[3*2*N_STEPS];

float p_780[2];
float p_780_sum[2];
float p_780_set[2];
float phase_780;
float accumulator_780;

float freq_ratio_423, freq_ratio_453;

float p_423[2];
float p_423_sum[2];
float p_423_set[2];
float phase_423;
float accumulator_423;

float p_453[2];
float p_453_sum[2];
float p_453_set[2];
float phase_453;
float accumulator_453;

float sin_array_780[N_STEPS];
float cos_array_780[N_STEPS];

float sin_array_423[N_STEPS];
float cos_array_423[N_STEPS];

float sin_array_453[N_STEPS];
float cos_array_453[N_STEPS];

float phase_damp_423;
float phase_damp_453;

bool serial_log;

void computeData() {
  for(int i=0; i<N_STEPS; i++) {
    sin_array_780[i] = sin(PI2*((float)i)*params.freq_780/N_STEPS);
    cos_array_780[i] = cos(PI2*((float)i)*params.freq_780/N_STEPS);

    sin_array_423[i] = sin(PI2*((float)i)*params.freq_423/N_STEPS);
    cos_array_423[i] = cos(PI2*((float)i)*params.freq_423/N_STEPS);

    sin_array_453[i] = sin(PI2*((float)i)*params.freq_453/N_STEPS);
    cos_array_453[i] = cos(PI2*((float)i)*params.freq_453/N_STEPS);
  }
}

void setup() {
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  Serial.begin(115200);
  
  // put your setup code here, to run once:
  in0 = ZEROV;
  in1 = ZEROV;
  in2 = ZEROV;
  in3 = ZEROV;
  
  out0 = ZEROV;
  out1 = V2P5;
  out2 = ZEROV;
  out3 = ZEROV;

  ramp_mean = ZEROV;
  params.ramp_amplitude = 2500;

  params.gain_p_780 = 100.;
  params.gain_i_780 = 200.;

  params.gain_p_423 = 100.;
  params.gain_i_423 = 200.;

  params.gain_p_453 = 100.;
  params.gain_i_453 = 200.;
  
  phase_780 = 0.0;
  accumulator_780 = 0.0;

  params.monitor_channel = MONITOR_ERR_780;
  
  params.freq_780 = 2.2;
  params.freq_423 = 2.2/(423./780.);
  params.freq_453 = 2.2/(453./780.);

  params.set_phase_780 = 0.0;
  params.set_phase_423 = 0.0;
  params.set_phase_453 = 0.0;

  params.lock_state_780 = 0;
  params.lock_state_423 = 0;
  params.lock_state_453 = 0;
  
  params.output_offset_423 = V2P5;
  params.output_offset_453 = V2P5;
  
  processParams();

  phase_damp_423 = 0.0;
  phase_damp_453 = 0.0;

  for(int i=0; i<N_STEPS; i++)
  {
    in0_array[i] = ZEROV;
  }

  ramp_direction = true;
  params.lock_state_780 = false;
  params.lock_state_423 = false;
  params.lock_state_453 = false;
  serial_log = false;
  current_time = micros();
}

void processParams() {
  ramp_direction = true;
  ramp_offset = -params.ramp_amplitude;
  ramp_step = 2*params.ramp_amplitude/N_STEPS;
  ramp_step_down = 2*params.ramp_amplitude/N_STEPS_DOWN;
  cycle_up = 0;
  cycle_down = 0;

  freq_ratio_423 = params.freq_423/params.freq_780;
  freq_ratio_453 = params.freq_453/params.freq_780;
  
  evaluate_sin_cos();
  computeData();
}

void evaluate_sin_cos() {
  p_780_set[0] = sin(params.set_phase_780);
  p_780_set[1] = cos(params.set_phase_780);

  p_423_set[0] = sin(params.set_phase_423);
  p_423_set[1] = cos(params.set_phase_423);

  p_453_set[0] = sin(params.set_phase_453);
  p_453_set[1] = cos(params.set_phase_453);
}

double get_phase_difference(float *p_act, float *p_set) {
  float sinp, cosp;
  float l2 = p_act[0]*p_act[0] + p_act[1]*p_act[1]; 
  if(l2 < MIN_FRINGE_VISIBILITY) {
    // Most likely visibility dropped due to laser multi-moding
    return 0.0;
  }
  float invhypot = 1./sqrt(l2);
  
  sinp = invhypot*(p_act[0]*p_set[0] - p_act[1]*p_set[1]);
  cosp = invhypot*(p_act[0]*p_set[1] + p_act[1]*p_set[0]);
  return atan2(sinp, cosp);  
}

void loop() {
  if(Serial.available())
    parseSerial();

  unsigned long previous_time = current_time;
  current_time = micros();
    
  in0 = analog.read(0, false);
  in1 = analog.read(1, false);
  in2 = analog.read(2, false);
  in3 = analog.read(3, false);
  
  if(ramp_direction) {
    in0_array[cycle_up] = in0;

    p_780[0] += estimation_matrix[cycle_up]*((float)in0);
    p_780[1] += estimation_matrix[cycle_up+N_STEPS]*((float)in0);

    p_423[0] += estimation_matrix[cycle_up+2*N_STEPS]*((float)in1);
    p_423[1] += estimation_matrix[cycle_up+3*N_STEPS]*((float)in1);

    p_453[0] += estimation_matrix[cycle_up+2*N_STEPS]*((float)in2);
    p_453[1] += estimation_matrix[cycle_up+3*N_STEPS]*((float)in2);
    
    cycle_up += 1;
    ramp_offset += ramp_step;
    if(cycle_up == N_STEPS)
    {
       cycle_up = 0;
       ramp_direction = false; // switch ramp direction
       // ramp done, reset fit parameters
       
       p_780_sum[0] = p_780[0];
       p_780_sum[1] = p_780[1];
       
       p_423_sum[0] = p_423[0];
       p_423_sum[1] = p_423[1];
       
       p_453_sum[0] = p_453[0];
       p_453_sum[1] = p_453[1];
       
       p_780[0] = 0.0;
       p_780[1] = 0.0;
       
       p_423[0] = 0.0;
       p_423[1] = 0.0;
       
       p_453[0] = 0.0;
       p_453[1] = 0.0;
       
       phase_780 = get_phase_difference(p_780_sum, p_780_set);
       phase_423 = get_phase_difference(p_423_sum, p_423_set);
       phase_453 = get_phase_difference(p_453_sum, p_453_set);
         
       if(params.lock_state_780) {
         accumulator_780 += params.gain_i_780 * phase_780;
         ramp_mean = ZEROV + (int)(accumulator_780 + params.gain_p_780*phase_780);

         // if interferometer is locked, then subtract residual error in phase
         phase_423 -= freq_ratio_423*phase_780;
         phase_453 -= freq_ratio_453*phase_780;
       }
       else {
         accumulator_780 = 0.0;
         ramp_mean = ZEROV; 
       }

       if(params.lock_state_423) {
         float pd = phase_423 - phase_damp_423;
         phase_damp_423 *= 0.99;
         accumulator_423 += params.gain_i_423 * pd;
         out1 = params.output_offset_423 - (int)(accumulator_423 + params.gain_p_423*pd);
       }
       else {
         accumulator_423 = 0.0;
         out1 = params.output_offset_423;
         phase_damp_423 = phase_423;
       }

       if(params.lock_state_453) {
         float pd = phase_453 - phase_damp_453;
         phase_damp_453 *= 0.99;
         accumulator_453 += params.gain_i_453 * pd;
         out2 = params.output_offset_453 - (int)(accumulator_453 + params.gain_p_453*pd);
        
       }
       else {
         accumulator_453 = 0.0;
         out2 = params.output_offset_453;
         phase_damp_453 = phase_453;
       }

       // write to monitor channel
       switch(params.monitor_channel) {
         case MONITOR_ERR_780:
           out3 = ZEROV + (int)(phase_780/PI*ZEROV);
           break;
         case MONITOR_ERR_423:
           out3 = ZEROV + (int)(phase_423/PI*ZEROV);
           break;
         case MONITOR_ERR_453:
           out3 = ZEROV + (int)(phase_453/PI*ZEROV);
           break;
         case MONITOR_SET_PHASE_780:
           out3 = ZEROV + (int)(params.set_phase_780/PI*ZEROV);
           break;
         case MONITOR_SET_PHASE_423:
           out3 = ZEROV + (int)(params.set_phase_423/PI*ZEROV);
           break;       
         case MONITOR_SET_PHASE_453:
           out3 = ZEROV + (int)(params.set_phase_453/PI*ZEROV);
           break;
       }

       if(serial_log) {
        // write out3 to the serial channel
        Serial.write((const uint8_t*)&out3, sizeof(int));
       }
    }

    switch(params.monitor_channel) {
      case MONITOR_FIT_780:
        out3 = ZEROV + p_780_sum[0]*sin_array_780[cycle_up] + p_780_sum[1]*cos_array_780[cycle_up];
        break;
       case MONITOR_RESIDUE_780:
         out3 = ZEROV + in0 - (p_780_sum[0]*sin_array_780[cycle_up] + p_780_sum[1]*cos_array_780[cycle_up]);
         break;
      case MONITOR_FIT_423:
        out3 = ZEROV + p_423_sum[0]*sin_array_423[cycle_up] + p_423_sum[1]*cos_array_423[cycle_up];
        break;
      case MONITOR_FIT_453:
        out3 = ZEROV + p_453_sum[0]*sin_array_453[cycle_up] + p_453_sum[1]*cos_array_453[cycle_up];
        break;
     }
  }
  else {
    // we are cycling down
    cycle_down += 1;
    ramp_offset -= ramp_step_down;
    if(cycle_down == N_STEPS_DOWN) {
      cycle_down = 0;
      ramp_direction = true;
    }
  }
    
  out0 = ramp_mean + ramp_offset;
  analog.write(out0, out1, out2, out3, true);
  
}

/*
 * g - get params
 * w - write to eeprom
 * r - read from eeprom
 * i - return UNO_ID
 * s - set params
 * a - read matrix from serial
 * z - write matrix to serial
 * l - toggle locks on the interferometer, 423 nm laser, or 453 nm laser
 * o - change the output offset on the 423 or 453 nm piezos
 * p - only change the interferometer locking phase
 * m - toggle serial monitoring
 */
void parseSerial() {
  char byte_read = Serial.read();

  switch(byte_read) {
    case 'g':
      // get params, send the entire struct in one go
      Serial.write((const uint8_t*)&params, sizeof(Params));
      break;
    case 'w':
      // write to EEPROM
      EEPROM_writeAnything(0, params);
      break;
    case 'r':
      EEPROM_readAnything(0, params);
      // EEPROM_readAnything(sizeof(params), logger);      
      break;
    case 'i':
      // return ID
      Serial.write(UNO_ID);
      break;
    case 's':
      // set params struct
      Serial.readBytes((char *) &params, sizeof(Params));
      processParams();
      break;
    case 'a':
      // get the estimation matrix
      Serial.readBytes((char *)&estimation_matrix, 3*2*N_STEPS*sizeof(float));
      break;
    case 'z':
      // send out the estimation matrix
      Serial.write((const uint8_t*)&estimation_matrix, 3*2*N_STEPS*sizeof(float));
      break;
    case 'l':
      // change the lock state for all 3 locks
      Serial.readBytes((char *) &params.lock_state_780, sizeof(int)*3);
      break;
    case 'p':
      // change the set phase
      Serial.readBytes((char *) &params.set_phase_780, sizeof(float)*3);
      evaluate_sin_cos();
      break;
      
    case 'o':
      // change the output offset
      Serial.readBytes((char *) &params.output_offset_423, sizeof(int)*2);
      break;

    case 'm':
      serial_log = !serial_log;
      break;
  }
}

template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          EEPROM.write(ee++, *p++);
    return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          *p++ = EEPROM.read(ee++);
    return i;
}
