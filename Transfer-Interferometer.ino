/*
 * Transfer Interferometer
 *
 * Tested with the Chipkit uc32 microcontroller -
 * https://reference.digilentinc.com/reference/microprocessor/uc32/start
 *
 * along with the Analog Shield for the DACs and ADCs -
 * http://store.digilentinc.com/analog-shield-high-performance-add-on-board-for-the-arduino-uno/
 *
 * The transfer interferometer reads in three analog inputs from the analog shield:
 *
 * in0 (A0 on shield): Interferometer signal for the reference laser
 * in1 (A1 on shield): Interferometer signal for locking laser 1
 * in2 (A2 on shield): Interferometer signal for locking laser 2
 *
 * and has 4 outputs:
 * out0 (D0 on shield): Piezo control for scanning and locking the interferometer path length
 * out1 (D1 on shield): Piezo control for laser 1
 * out2 (D2 on shield): Piezo control for laser 2
 * out3 (D3 on shield): Monitor channel
 */

#include <EEPROM.h>
#include <analogShield.h>
#include <math.h>
#include <SPI.h>  // SPI.h has to be included **after** analogShield.h

#define UNO_ID "interferometer_lock\r\n"  // unique identity of the microcontroller

#define ZEROV 32768 //Zero volts
#define V2P5 49512  // 2.5 V

#define N_STEPS 50  // Number of ramp steps when ramping up the piezo voltage
#define N_STEPS_DOWN 20  // Number of steps when ramping down (value less than 20 gives rise to oscillations)

#define PI 3.14159
#define PI2 6.28318

// Minimum fringe visibility required to reliably detect fringes
// If fringe visibility suddenly drops, it's most likely an indication of a laser mode-hop
#define MIN_FRINGE_VISIBILITY 27000.

#define MONITOR_ERR_REF 1
#define MONITOR_FIT_REF 2
#define MONITOR_SET_PHASE_REF 3
#define MONITOR_RESIDUE_REF 4

#define MONITOR_ERR_L1 11
#define MONITOR_FIT_L1 12
#define MONITOR_SET_PHASE_L1 13
#define MONITOR_RESIDUE_L1 14

#define MONITOR_ERR_L2 21
#define MONITOR_FIT_L2 22
#define MONITOR_SET_PHASE_L2 23
#define MONITOR_RESIDUE_L2 24


// This struct has all the parameters that the muC needs to run
struct Params {
  int ramp_amplitude;

  float freq_ref, freq_l1, freq_l2;

  float gain_p_ref, gain_i_ref;
  float gain_p_l1, gain_i_l1;
  float gain_p_l2, gain_i_l2;

  float set_phase_ref, set_phase_l1, set_phase_l2;
  int lock_state_ref, lock_state_l1, lock_state_l2;

  int output_offset_l1, output_offset_l2;
  int monitor_channel;

  int ramp_amplitude_l1, ramp_amplitude_l2;
  int ramp_n_steps_l1, ramp_n_steps_l2;
  int ramp_state_l1, ramp_state_l2;

};

Params params;

int in0, in1, in2, in3;
int out0, out1, out2, out3;

int out1_pid, out2_pid;

int ramp_mean;
int ramp_offset;

int ramp_offset_l1;
int ramp_offset_l2;

int cycle_up = 0;
int cycle_up_l1 = 0;
int cycle_up_l2 = 0;

int cycle_down = 0;
int cycle_down_l1 = 0;
int cycle_down_l2 = 0;

int ramp_step;
int ramp_step_down;

int ramp_step_l1;
int ramp_step_l2;

bool ramp_direction;
bool ramp_direction_l1;
bool ramp_direction_l2;

int in0_array[N_STEPS];
int in0_array2[N_STEPS];

int in0_buffer;

unsigned long current_time;

// 3 matrices, one for each wavelength, each of size 2xN_STEPS
float estimation_matrix[3*2*N_STEPS];

float p_ref[2];
float p_ref_sum[2];
float p_ref_set[2];
float phase_ref;
float accumulator_ref;

float freq_ratio_l1, freq_ratio_l2;

float p_l1[2];
float p_l1_sum[2];
float p_l1_set[2];
float phase_l1;
float accumulator_l1;

float p_l2[2];
float p_l2_sum[2];
float p_l2_set[2];
float phase_l2;
float accumulator_l2;

float sin_array_ref[N_STEPS];
float cos_array_ref[N_STEPS];

float sin_array_l1[N_STEPS];
float cos_array_l1[N_STEPS];

float sin_array_l2[N_STEPS];
float cos_array_l2[N_STEPS];

float phase_damp_l1;
float phase_damp_l2;


bool serial_log;

// Pre-compute sin and cos required for monitoring the fit
void computeData() {
  for(int i=0; i<N_STEPS; i++) {
    sin_array_ref[i] = sin(PI2*((float)i)*params.freq_ref/N_STEPS);
    cos_array_ref[i] = cos(PI2*((float)i)*params.freq_ref/N_STEPS);

    sin_array_l1[i] = sin(PI2*((float)i)*params.freq_l1/N_STEPS);
    cos_array_l1[i] = cos(PI2*((float)i)*params.freq_l1/N_STEPS);

    sin_array_l2[i] = sin(PI2*((float)i)*params.freq_l2/N_STEPS);
    cos_array_l2[i] = cos(PI2*((float)i)*params.freq_l2/N_STEPS);
  }
}

void setup() {
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  Serial.begin(115200);

  in0_buffer = 0;

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

  params.gain_p_ref = 100.;
  params.gain_i_ref = 200.;

  params.gain_p_l1 = 100.;
  params.gain_i_l1 = 200.;

  params.gain_p_l2 = 100.;
  params.gain_i_l2 = 200.;

  phase_ref = 0.0;
  accumulator_ref = 0.0;

  params.monitor_channel = MONITOR_ERR_REF;

  params.freq_ref = 2.2;
  params.freq_l1 = 2.2/(423./780.);
  params.freq_l2 = 2.2/(453./780.);

  params.set_phase_ref = 0.0;
  params.set_phase_l1 = 0.0;
  params.set_phase_l2 = 0.0;

  params.lock_state_ref = 0;
  params.lock_state_l1 = 0;
  params.lock_state_l2 = 0;

  params.output_offset_l1 = V2P5;
  params.output_offset_l2 = V2P5;

  phase_damp_l1 = 0.0;
  phase_damp_l2 = 0.0;

  for(int i=0; i<N_STEPS; i++)
  {
    in0_array[i] = ZEROV;
  }

  ramp_direction = true;
  params.lock_state_ref = false;
  params.lock_state_l1 = false;
  params.lock_state_l2 = false;

  params.ramp_amplitude_l1 = 3000;
  params.ramp_amplitude_l2 = 3000;
  params.ramp_n_steps_l1 = 100;
  params.ramp_n_steps_l2 = 100;
  params.ramp_state_l1 = 0;
  params.ramp_state_l2 = 0;

  processParams();

  out1_pid = 0;
  out2_pid = 0;

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

  freq_ratio_l1 = params.freq_l1/params.freq_ref;
  freq_ratio_l2 = params.freq_l2/params.freq_ref;

  ramp_direction_l1 = true;
  ramp_offset_l1 = -params.ramp_amplitude_l1;
  ramp_step_l1 = 2*params.ramp_amplitude_l1/params.ramp_n_steps_l1;
  cycle_up_l1 = 0;
  cycle_down_l1 = 0;

  ramp_direction_l2 = true;
  ramp_offset_l2 = -params.ramp_amplitude_l2;
  ramp_step_l2 = 2*params.ramp_amplitude_l2/params.ramp_n_steps_l2;
  cycle_up_l2 = 0;
  cycle_down_l2 = 0;

  evaluate_sin_cos();
  computeData();
}

void evaluate_sin_cos() {
  p_ref_set[0] = sin(params.set_phase_ref);
  p_ref_set[1] = cos(params.set_phase_ref);

  p_l1_set[0] = sin(params.set_phase_l1);
  p_l1_set[1] = cos(params.set_phase_l1);

  p_l2_set[0] = sin(params.set_phase_l2);
  p_l2_set[1] = cos(params.set_phase_l2);
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
    if(in0_buffer)
      in0_array2[cycle_up] = in0;
    else
      in0_array[cycle_up] = in0;

    p_ref[0] += estimation_matrix[cycle_up]*((float)in0);
    p_ref[1] += estimation_matrix[cycle_up+N_STEPS]*((float)in0);

    p_l1[0] += estimation_matrix[cycle_up+2*N_STEPS]*((float)in1);
    p_l1[1] += estimation_matrix[cycle_up+3*N_STEPS]*((float)in1);

    p_l2[0] += estimation_matrix[cycle_up+2*N_STEPS]*((float)in2);
    p_l2[1] += estimation_matrix[cycle_up+3*N_STEPS]*((float)in2);

    cycle_up += 1;
    ramp_offset += ramp_step;
    if(cycle_up == N_STEPS)
    {
       cycle_up = 0;
       ramp_direction = false; // switch ramp direction
       // ramp done, reset fit parameters

       p_ref_sum[0] = p_ref[0];
       p_ref_sum[1] = p_ref[1];

       p_l1_sum[0] = p_l1[0];
       p_l1_sum[1] = p_l1[1];

       p_l2_sum[0] = p_l2[0];
       p_l2_sum[1] = p_l2[1];

       p_ref[0] = 0.0;
       p_ref[1] = 0.0;

       p_l1[0] = 0.0;
       p_l1[1] = 0.0;

       p_l2[0] = 0.0;
       p_l2[1] = 0.0;

       phase_ref = get_phase_difference(p_ref_sum, p_ref_set);
       phase_l1 = get_phase_difference(p_l1_sum, p_l1_set);
       phase_l2 = get_phase_difference(p_l2_sum, p_l2_set);

       if(params.lock_state_ref) {
         accumulator_ref += params.gain_i_ref * phase_ref;
         ramp_mean = ZEROV + (int)(accumulator_ref + params.gain_p_ref*phase_ref);

         // if interferometer is locked, then subtract residual error in phase
         phase_l1 -= freq_ratio_l1*phase_ref;
         phase_l2 -= freq_ratio_l2*phase_ref;
       }
       else {
         accumulator_ref = 0.0;
         ramp_mean = ZEROV;
       }

       if(params.lock_state_l1) {
         float pd = phase_l1 - phase_damp_l1;
         phase_damp_l1 *= 0.99;
         accumulator_l1 += params.gain_i_l1 * pd;
         out1_pid = (int)(accumulator_l1 + params.gain_p_l1*pd);

       }
       else {
         accumulator_l1 = 0.0;
         out1_pid = 0;
         phase_damp_l1 = phase_l1;
       }

       if(params.lock_state_l2) {
         float pd = phase_l2 - phase_damp_l2;
         phase_damp_l2 *= 0.99;
         accumulator_l2 += params.gain_i_l2 * pd;
         out2_pid = (int)(accumulator_l2 + params.gain_p_l2*pd);
       }
       else {
         accumulator_l2 = 0.0;
         out2_pid = 0;
         phase_damp_l2 = phase_l2;
       }

       // write to monitor channel
       switch(params.monitor_channel) {
         case MONITOR_ERR_REF:
           out3 = ZEROV + (int)(phase_ref/PI*ZEROV);
           break;
         case MONITOR_ERR_L1:
           out3 = ZEROV + (int)(phase_l1/PI*ZEROV);
           break;
         case MONITOR_ERR_L2:
           out3 = ZEROV + (int)(phase_l2/PI*ZEROV);
           break;
         case MONITOR_SET_PHASE_REF:
           out3 = ZEROV + (int)(params.set_phase_ref/PI*ZEROV);
           break;
         case MONITOR_SET_PHASE_L1:
           out3 = ZEROV + (int)(params.set_phase_l1/PI*ZEROV);
           break;
         case MONITOR_SET_PHASE_L2:
           out3 = ZEROV + (int)(params.set_phase_l2/PI*ZEROV);
           break;
       }

       if(serial_log) {
        // write out3 to the serial channel
        Serial.write((const uint8_t*)&out3, sizeof(int));
       }
    }

    switch(params.monitor_channel) {
      case MONITOR_FIT_REF:
        out3 = ZEROV + p_ref_sum[0]*sin_array_ref[cycle_up] + p_ref_sum[1]*cos_array_ref[cycle_up];
        break;
       case MONITOR_RESIDUE_REF:
         out3 = in0 - (p_ref_sum[0]*sin_array_ref[cycle_up] + p_ref_sum[1]*cos_array_ref[cycle_up]);
         break;
      case MONITOR_FIT_L1:
        out3 = ZEROV + p_l1_sum[0]*sin_array_l1[cycle_up] + p_l1_sum[1]*cos_array_l1[cycle_up];
        break;
      case MONITOR_RESIDUE_L1:
        out3 = in1 - (p_l1_sum[0]*sin_array_l1[cycle_up] + p_l1_sum[1]*cos_array_l1[cycle_up]);
        break;
      case MONITOR_FIT_L2:
        out3 = ZEROV + p_l2_sum[0]*sin_array_l2[cycle_up] + p_l2_sum[1]*cos_array_l2[cycle_up];
        break;
      case MONITOR_RESIDUE_L2:
        out3 = in2 - (p_l2_sum[0]*sin_array_l2[cycle_up] + p_l2_sum[1]*cos_array_l2[cycle_up]);
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
      if(in0_buffer)
        in0_buffer = 0;
      else
        in0_buffer = 1;

    }
  }


  out1 = params.output_offset_l1 - out1_pid;
  out2 = params.output_offset_l2 - out2_pid;

  // ramp the l1 and l2 channels
  if(params.ramp_state_l1) {
    // we want to ramp the l1 channel
    if(ramp_direction_l1) {
      // we are ramping up
      cycle_up_l1 += 1;
      ramp_offset_l1 += ramp_step_l1;

      if(cycle_up_l1 == params.ramp_n_steps_l1) {
        // end of cycle up
        cycle_up_l1 = 0;
        ramp_direction_l1 = false;
      }
    }
    else {
      // ramping down
      cycle_down_l1 += 1;
      ramp_offset_l1 -= ramp_step_l1;
      if(cycle_down_l1 == params.ramp_n_steps_l1) {
        // end of cycle down
        cycle_down_l1 = 0;
        ramp_direction_l1 = true;
      }

    }
    out1 += ramp_offset_l1;
  }
  else {
    cycle_up_l1 = 0;
    cycle_down_l1 = 0;
    ramp_direction_l1 = true;
    ramp_offset_l1 = -params.ramp_amplitude_l1;
  }

  // ramp the l2 laser
  if(params.ramp_state_l2) {
    // we want to ramp the l2 channel
    if(ramp_direction_l2) {
      // we are ramping up
      cycle_up_l2 += 1;
      ramp_offset_l2 += ramp_step_l2;

      if(cycle_up_l2 == params.ramp_n_steps_l2) {
        // end of cycle up
        cycle_up_l2 = 0;
        ramp_direction_l2 = false;
      }
    }
    else {
      // ramping down
      cycle_down_l2 += 1;
      ramp_offset_l2 -= ramp_step_l2;
      if(cycle_down_l2 == params.ramp_n_steps_l2) {
        // end of cycle down
        cycle_down_l2 = 0;
        ramp_direction_l2 = true;
      }

    }
    out2 += ramp_offset_l2;
  }
  else {
    cycle_up_l2 = 0;
    cycle_down_l2 = 0;
    ramp_direction_l2 = true;
    ramp_offset_l2 = -params.ramp_amplitude_l2;
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
 * l - toggle locks on the interferometer, l1 nm laser, or l2 nm laser
 * o - change the output offset on the l1 or l2 nm piezos
 * p - only change the interferometer locking phase
 * m - toggle serial monitoring
 * d - write in0 buffer data
 * c - set current phase as the set phase
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
      Serial.readBytes((char *) &params.lock_state_ref, sizeof(int)*3);
      break;
    case 'p':
      // change the set phase
      Serial.readBytes((char *) &params.set_phase_ref, sizeof(float)*3);
      evaluate_sin_cos();
      break;
    case 'o':
      // change the output offset
      Serial.readBytes((char *) &params.output_offset_l1, sizeof(int)*2);
      break;
    case 'm':
      // toggle serial log
      serial_log = !serial_log;
      break;
    case 'd':
      if(in0_buffer)
        Serial.write((const uint8_t*)&in0_array2, N_STEPS*sizeof(int));
      else
        Serial.write((const uint8_t*)&in0_array, N_STEPS*sizeof(int));
      break;
    case 'c':
      // set current phase as the lock phase
      char which_laser = Serial.read();
      Serial.readBytes((char *) &which_laser, sizeof(char));
      float zero_p[2] = {0.0, 1.0};
      if(which_laser == '1') {
        params.set_phase_l1 = -get_phase_difference(p_l1_sum, zero_p);
        processParams();
      }
      else if(which_laser == '2') {
        params.set_phase_l2 = -get_phase_difference(p_l2_sum, zero_p);
        processParams();
      }
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
