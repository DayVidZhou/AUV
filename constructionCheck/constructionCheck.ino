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
#define WATER_DENSITY 1 // g/cm^3
#define GRAVITY 9.81
#define PRESSURE_SENSOR_PIN A1
#define MIN_OFFSET 0.204
#define FULL_SCALE_OUTPUT 4.896
#define FULL_SCALE_SPAN 4.692
#define SENSITIVITY 0.02
#define ADC_RES 1023
#define ADC_LSB 0.0049

#define MIN_PRESSURE 20
#define MAX_PRESSURE 250

#define IDLE 0
#define SURGING 1
#define YAWING 2
#define HEAVING 3

Servo motA, motB, motC;

const int thrust_delay = 10; // miliseconds
const int thrust_step = 5;
const int pulse_delay = 250;

const float Kp = 0;
const float Ki = 0;
const float Kd = 0;
const float Km = 0;
const float m = 0;
const time_const = 0;
const float K = 0;

const float max_r = 0;
const float max_yaw = 89;

float r_d = 0;
float r = 0;
float r_err = 0; // r - rd
float yaw_d = 0
float yaw = 0; 
float yaw_err = 0; // yaw - yaw_d

float prev_yaw;
float i_gain;
float d_gain;
float p_gain;
double t_now = 0;
double t_prev = 0;
unsigned long pid_time = 0; 
unsigned long time = 0;
int state = 0;

float acX=10,acY=10,acZ=10,tmp=10,gyX=10,gyY=10,gyZ=10;
char cmd = 0;

void setup(){
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

void yaw(int input_signal) {
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
			surge(x_d);
			break;	
		case YAWING:
			yaw(yaw_d);
			break;
		case HEAVING:
			heave(z_d);
			break;
	}
	/*if (readByte(MPU_ADDR, INT_STATUS) & 0x01) {
		get_acc(acc_p);
		acX = (float)acc_p[0]*a_res;
		acY = (float)acc_p[1]*a_res;
		acZ = (float)acc_p[2]*a_res;
		get_gyro(gyr_p);
		gyX = (float)gyr_p[0]*g_res;
    	gyY = (float)gyr_p[1]*g_res;
    	gyZ = (float)gyr_p[2]*g_res;
		printMPUData();
		cur_yaw = gyZ*(time-millis()) + cur_yaw;
		time = millis();
	}
	
	if ( (millis() - pid_time) >= PID_T) {
		motor_cv = pidController(cur_yaw);
		//analogWrite(PWM_LEFT, motor_cv);
		//analogWrite(PWM_RIGHT, motor_cv);
		pid_time = millis();
	}*/

}

// callback for sending data
void sendData(){
	if (state == IDLE) {
		Wire.write(get_depth());
	} else {
		Wire.write(state);
	}
}

void receiveData(int byteCount){

	while(Wire.available()) {
		cmd = Wire.read();
		switch (state){
			case IDLE:
			// get command + reference signal
				break;	
			case YAWING:
			// get imu data		
				break;
			case SURGING:
			// get imu data
				break;
			case HEAVING:
			// get imu data
				break;
		}
	}
}

float get_depth(){
	float v = analogRead(PRESSURE_SENSOR_PIN) * ADC_LSB;
	if (v <= 0.204) return 20;
	if (v >= 4.896) return 250;
	float P =((v - MIN_OFFSET)/SENSITIVITY) + 20;
	return (P/(WATER_DENSITY*GRAVITY))*100;	
}

int pidController(float yaw) {
    // Calculate time since last time PID was called (~10ms)
    t_now = (double)millis();
    double dt = t_now - t_prev;
 
    // Calculate Error
    float e = SP - yaw;
 
    // Calculate our PID terms
    p_gain = Kp * error;
    i_gain += Ki * error * dt;
    d_gain = Kd * (yaw - prev_yaw) / dt; 
 
    prev_yaw = yaw;
    t_prev = t_now;
 
    // Set PWM Value
    float pwm_control_out = p_gain + i_gain - d_gain;
 
    // Limits PID to Yaw angle
    if (pwm_control_out > MAX_YAW) {
		pwm_control_out = MAX_YAW;
    } else if (pwm_control_out < -MAX_YAW) {
		pwm_control_out = -MAX_YAW; 
 	}
 
    // Return PID Output
    return int(pwm_control_out);
}
