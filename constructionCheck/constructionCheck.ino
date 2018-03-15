#include<Servo.h> 
#include <Wire.h>
#include <stdint.h>

#if 0

#define MOTOR_DEBUG 1

#endif

#if 0

#define DEBUG 1

#endif

// Motor Params
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MID_PULSE_LENGTH 1500 // Middle pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs
#define RANGE MAX_PULSE_LENGTH-MID_PULSE_LENGTH
#define RIGHT_MOTOR_PIN 9
#define LEFT_MOTOR_PIN 11
#define MIDDLE_MOTOR_PIN 10

// Pressure Sensor Params
#define WATER_DENSITY 998.57 // [kg/m^3] @ 18 [deg]
#define GRAVITY 9.81
#define PRESSURE_SENSOR_PIN A1
#define MIN_OFFSET 0.204 // Vdc
#define FULL_SCALE_OUTPUT 4.896 // Vdc
#define FULL_SCALE_SPAN 4.692 // Vdc
#define SENSITIVITY 0.02

#define ADC_RES 1023
#define ADC_LSB 0.00488

#define MIN_PRESSURE 20
#define MAX_PRESSURE 250
#define ATM_PRESSURE 100.7 // kPa

#define IDLE 0
#define SURGING 1
#define YAWING 2
#define HEAVING 3

#define PI_PACKET_SIZE 5
#define SLAVE_ADDRESS 0x04
#define REG_REF_SURGE 0x03
#define REG_REF_DEPTH 0x04
#define REG_REF_HEADING 0x05 # Desired heading from LOS path following controller
#define REG_GX 0x06
#define REG_GY 0x07
#define REG_GZ 0x08
#define REG_AX 0x09
#define REG_AY 0x0A
#define REG_AZ 0x0B
#define REG_X 0x0C
#define REG_Y 0x0D
#define REG_Z 0x0E
#define REG_ALL_IMU 0x0F
#define REG_R_ALL 0x10

Servo motA, motB, motC;

const int thrust_delay = 10; // miliseconds
const int thrust_step = 5;
const int pulse_delay = 250;

// YAW
const float Kp = 0;
const float Ki = 0;
const float Kd = 0;
const float Km = 0;
const float m = 0;
const float time_const = 0;
const float K = 0;

const float max_r = 0;
const float max_yaw = 89;

float r_d = 0;
float r = 0;
float r_err = 0; // r - rd
float yaw_d = 0;
float yaw = 0; 
float yaw_err = 0; // yaw - yaw_d

float prev_yaw;
float cur_yaw;
float i_gain;
float d_gain;
float p_gain;
double t_now = 0;
double t_prev = 0;
unsigned long pid_time = 0; 
unsigned long time = 0;
int state = 0;
volatile float cur_depth = 0;

float acX=10,acY=10,acZ=10,tmp=10,gyX=10,gyY=10,gyZ=10;
char cmd = 0;

typedef union float2bytes_t { 
  float f; 
  byte b[sizeof(float)]; 
}; //float2bytes_t b2f;

typedef struct state_info{
	int cur_state;
	int prev_state; // random / should be replaced with other 2 byte padding
	float depth;
} state_info_t;

typedef union i2c_packet {
	state_info_t data;
	byte packet[sizeof(state_info_t)];
}i2c_packet_t;

i2c_packet_t i2c_send;

byte imu_data[24]; // 6axis * (4bytes/float)
float imu_data_f[6];
int yaw_controller(float yaw);
void process_imu_data(int type,int i);

void setup(){

	Serial.begin(9600);
	
	//////// MOTOR SETUP ////////	
	motA.attach(RIGHT_MOTOR_PIN, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // right
    motB.attach(LEFT_MOTOR_PIN, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // left 
    motC.attach(MIDDLE_MOTOR_PIN, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // middle

	// initialize i2c as slave
	Wire.begin(SLAVE_ADDRESS);

	// define callbacks for i2c communication
	Wire.onReceive(receiveData);
	Wire.onRequest(sendData);
   
	// Set to mid point
	motA.writeMicroseconds(MID_PULSE_LENGTH);
	motB.writeMicroseconds(MID_PULSE_LENGTH);
	motC.writeMicroseconds(MID_PULSE_LENGTH);
	
	state = IDLE;
}

void surge(int input_signal) {
	int j = 0;
	for (int i = MID_PULSE_LENGTH; i <= (MID_PULSE_LENGTH+input_signal); i += thrust_step) {
		motA.writeMicroseconds(i);
		j = MID_PULSE_LENGTH - i + MID_PULSE_LENGTH;
		motB.writeMicroseconds(j);
		delay(thrust_delay);
	}
	delay(pulse_delay);
	motA.writeMicroseconds(MID_PULSE_LENGTH);
	motB.writeMicroseconds(MID_PULSE_LENGTH);
}

void yaw_rov(int input_signal) {
    for (int i = MID_PULSE_LENGTH; i <= (MID_PULSE_LENGTH+input_signal); i += thrust_step) {
        motA.writeMicroseconds(i);
        motB.writeMicroseconds(i);
        delay(thrust_delay);
    }
	delay(pulse_delay);
    motA.writeMicroseconds(MID_PULSE_LENGTH);
    motB.writeMicroseconds(MID_PULSE_LENGTH);
}

void heave(int input_signal) {
    for (int i = MID_PULSE_LENGTH; i <= (MID_PULSE_LENGTH+input_signal); i += thrust_step) {
        motC.writeMicroseconds(i);
        delay(thrust_delay);
    }
	delay(pulse_delay);
    motC.writeMicroseconds(MID_PULSE_LENGTH);
}

void loop(void){
	cur_depth = get_depth();	
	switch (state){
		case IDLE:
			if (cmd == SURGING){
				state = SURGING;
			}else if(cmd == YAWING){
				state = YAWING;
			}else if(cmd == HEAVING){
				state = HEAVING;
			}
			break;
		case SURGING:
			//surge(x_d);
			break;	
		case YAWING:
			//cur_yaw = gyZ*(time-millis()) + cur_yaw;
			//time = millis();
			//yaw(yaw_d);
			break;
		case HEAVING:
			//heave(z_d);
			break;
	}
	/*	
	if ( (millis() - pid_time) >= PID_T) {
		motor_cv = pidController(cur_yaw);
		//analogWrite(PWM_LEFT, motor_cv);
		//analogWrite(PWM_RIGHT, motor_cv);
		pid_time = millis();
	}*/

}

//TODO: TEST I2C Communication

// callback for sending data
void sendData(){
	//float2bytes_t p;
	//p.f = get_depth();
	if (cmd == REG_R_ALL){
		i2c_send.data.cur_state = state;
		i2c_send.data.prev_state = 10;
		i2c_send.data.depth = cur_depth;
		//Serial.println("requested");
		//Serial.println(sizeof(state_info_t));
		Wire.write(i2c_send.packet, sizeof(state_info_t));
	}
}

void receiveData(int byteCount){
	int i = 0;
	cmd = Wire.read();
	//Serial.print("Bytes Recv: ");
	//Serial.println(byteCount);
	
	
	switch(cmd){
		case REG_GZ:
			while (Wire.available()){
				gyZ = ((uint16_t)Wire.read() << 8) | (uint16_t)Wire.read();
			}
			break;
		case REG_AX:
			while (Wire.available()){
				acX = ((uint16_t)Wire.read() << 8) | (uint16_t)Wire.read();
			}
			break;
		case REG_AY:
			while (Wire.available()){
				acY = ((uint16_t)Wire.read() << 8) | (uint16_t)Wire.read();
			}
			break;
		case REG_ALL_IMU:
			Wire.read();
			while (Wire.available()) {
				imu_data[i] = Wire.read();
				i++;
			}
			process_imu_data(REG_ALL_IMU,i);
			break;
	}
	
}

float get_depth(){
	float v = analogRead(PRESSURE_SENSOR_PIN) * ADC_LSB;
	if (v < 0.204) return 20.0;
	if (v > 4.896) return 250.0;
	//float P =((v - MIN_OFFSET)/SENSITIVITY) + MIN_PRESSURE;
	float P = (v/SENSITIVITY) + MIN_PRESSURE;
	return ( 1000*(P-ATM_PRESSURE) / (WATER_DENSITY*GRAVITY) );	
}

int yaw_controller(float yaw_angle) {
    // Calculate time since last time PID was called (~10ms)
    t_now = (double)millis();
    double dt = t_now - t_prev;
 
    // Calculate Error
    float e = SP - yaw_angle;
 
    // Calculate our PID terms
    p_gain = Kp * e;
    i_gain += Ki * e * dt;
    d_gain = Kd * (yaw - prev_yaw) / dt; 
 
    prev_yaw = yaw_angle;
    t_prev = t_now;
 
    // Set PWM Value
    float pwm_control_out = p_gain + i_gain - d_gain;
 
    // Limits PID to Yaw angle
    if (pwm_control_out > max_yaw) {
		pwm_control_out = max_yaw;
    } else if (pwm_control_out < -max_yaw) {
		pwm_control_out = -max_yaw; 
 	}
 
    // Return PID Output
    return int(pwm_control_out);
}

void process_imu_data(int type, int sz){
	int i = 0;
	float2bytes_t b2f;
	switch(type){
		case REG_ALL_IMU:
			for (i = 0; i < (sz>>2); i++){
				b2f.b[0] = imu_data[i*4];
				b2f.b[1] = imu_data[i*4+1];
				b2f.b[2] = imu_data[i*4+2];
				b2f.b[3] = imu_data[i*4+3];
				imu_data_f[i] = b2f.f;
				//Serial.println(imu_data_f[i],4);
			}
			Serial.println(imu_data_f[2],4);
			break;
	}
}
