#include <Servo.h> 
#include <Wire.h>
#include <stdint.h>
#include <PID_v1.h>
#include <util/atomic.h>

// ROV Params
#define M 2.0976
#define Iz 0.02296
#define VOLUME 0.002427
#define DRAG_COEFF_2_Z 0 // z nonlinear drag
#define DRAG_COEFF_Z 0 // z linear drag
#define DRAG_COEFF_YAW 0 // linear yaw drag
#define N_RDOT 0 // added mass for yaw
#define N_WDOT 0 // added mass for heave
#define DIST_L 1 // distance from left motor to COM
#define DIST_R 1 // distance from right motor to COM
#define K_c 1 // center motor input gain 
#define K_a 1 // right motor input gain
#define K_b 1 // left motor input gain

#define GRAVITY_OFFSET 0 // GRAVITY - BOUYANCY

#define Wb_YAW 0.1
#define Wb_HEAVE 0.1
#define T_HEAVE 0 // heave time constant
#define T_YAW 0.431 // yaw time constant
#define T_SURGE 0 // surge input gain
#define K_YAW 1.13 // yaw gain for an input of 55 microseconds
#define K_HEAVE 1
#define PID_RATE 5 // Hz
#define YAW_W_HPF_CUTOFF 5 // Hz
#define IMU_SAMPLE_RATE 42 // Hz

#define YAW_CTRL_MAX 80
#define SURGE_CTRL_MAX 100
#define HEAVE_CTRL_MAX 120

#define YAW_RATE_D_MAX 1.0
#define YAW_ACCEL_D_MAX 0.5
#define HEAVE_RATE_D_MAX 0.2
#define HEAVE_ACCEL_D_MAX 0.1
#define SURGE_RATE_D_MAX 1.6
#define SURGE_ACCEL_D_MAX 0.5

// Motor Params
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MID_PULSE_LENGTH 1500 // Middle pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs
#define DEAD_RANGE 90
#define MIN_FWD 1545 // needs to be checked
#define MIN_RWD 1455 // needs to be checked
#define RIGHT_MOTOR_PIN 10
#define LEFT_MOTOR_PIN 11
#define MIDDLE_MOTOR_PIN 9

// Pressure Sensor Params
#define WATER_DENSITY 998.57 // [kg/m^3] @ 18 [deg]
#define GRAVITY 9.81
#define PRESSURE_SENSOR_PIN A1
#define MIN_OFFSET 0.204 // Vdc
#define FULL_SCALE_OUTPUT 4.896 // Vdc
#define FULL_SCALE_SPAN 4.692 // Vdc
#define SENSITIVITY 0.02
#define MIN_PRESSURE 20
#define MAX_PRESSURE 250
#define ATM_PRESSURE 100.7 // kPa

// Arduino ADC
#define ADC_RES 1023
#define ADC_LSB 0.00488

// I2C constants
#define SLAVE_ADDRESS 0x04
#define SEND_SIZE 8 // bytes

// I2C commands
enum I2C_CMD {
	ALL_IMU,
	REF_TRAG,
	USER_CMD,
	SEND_ALL,
	CHANGE_MODE,
	YAW_DYN,
	HEAVE_DYN,
	SURGE_DYN,
	CHANGE_YAW_PID,
	CHANGE_HEAVE_PID
}; 

// ROV Motion State
enum State {
	IDLE,
	SURGING,
	YAWING,
	HEAVING
};

// ROV MODE
enum Mode {
	AUTONOMOUS,
	USER_CONTROL
};

enum ManualDir {
	STOP,
	FWD_SURGE,
	RWD_SURGE,
	UP_HEAVE,
	DWN_HEAVE,
	YAW_LEFT,
	YAW_RIGHT,
	SHUT_DWN = 0xff
};

typedef union float2bytes_t { 
  float f;
  byte b[sizeof(float)]; 
}; //float2bytes_t b2f;

typedef union double2bytes_t { 
  double d;
  byte b[sizeof(double)]; 
}; //float2bytes_t b2f;


typedef struct state_info{
	double yaw_pid_out;
	double heave_pid_out;
	float depth;
} state_info_t;

typedef union i2c_packet {
	state_info_t data;
	byte packet[sizeof(state_info_t)];
}i2c_packet_t;

typedef struct position{
	double x;
	double y;
	double z;
	double roll;
	double pitch;
	double yaw;
} position_t;

typedef struct user_cmd{
	uint8_t mode; // autonomous = 0 or manual = 1
	int16_t direction; // if mode == 1 then set respective motors to power
	int16_t power; // if mode == 1 then set current motors to power, otherwise ignore
} user_cmd_t;

typedef struct pid_params {
	double wb;
	double damp_ratio;
	double T;
	double K;
	double Km;
	double wn;
	double m;
	double Kp;
	double Ki;
	double Kd;
	//unsigned long sample_freq; // might be equvialent to PID_T aka time const
	//float max_r;
	//float max_yaw;
} pid_params_t;

typedef struct pid_control {
	double Kp;
	double Kd;
	double Ki;
	double sample_time;
	double limit;
} pid_contol_t;

typedef struct mot_sig {
	int right;
	int left;
	int center;
	double yaw_thrust;
	double heave_thrust;
	double surge_thrust;
} mot_sig_t;

