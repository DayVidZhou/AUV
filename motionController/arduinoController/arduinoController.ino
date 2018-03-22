#include "arduinoController.h"
#define BYTES_TO_DOUBLE(b, i)					\
						((long)b[i]) 			| \
						((long)b[i+1] << 8)	| \
						((long)b[i+2] << 16)	| \
						((long)b[i+3] << 20)
Servo motA, motB, motC;

const int thrust_delay = 5; // miliseconds
const int thrust_step = 1;
const int pulse_delay = 100;

pid_params_t yaw_pid = {Wb_YAW, 1.0,T_YAW,K_YAW, 0};
							
pid_params_t heave_pid = {Wb_HEAVE,1.0,T_HEAVE,K_HEAVE, 0};
State state = IDLE;

I2C_CMD cmd;

position_t pos;
position_t accel;
position_t vel;
position_t accel_d;
position_t vel_d;
position_t pos_d;
user_cmd_t user;
mot_sig_t cur_mot;


i2c_packet_t i2c_send;
byte i2c_buffer[127];
volatile float mpx4250_depth = 0;
double yaw_pid_output;
double heave_pid_output;
int user_power;
double prev_vel_yaw;
double prev_accel_yaw;
int first_sample = 0;

PID yaw_pid_ctrl(&(yaw_pid_output), &(pos.yaw), &(pos_d.yaw), yaw_pid.Kp, yaw_pid.Ki, yaw_pid.Kd, DIRECT);
PID heave_pid_ctrl(&(heave_pid_output), &(pos.z), &(pos_d.z), heave_pid.Kp, heave_pid.Ki, heave_pid.Kd, DIRECT);


int yaw_controller(float);
void process_imu_data(int, int);
void lr_motor_drive(int, int);
void surge(int);
void yaw(int);
void heave(int);
void handle_manual_input(void);
inline void read_bytes(byte bytes[4], byte *p, int i);
double gyro_accel_hpf();

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
	user.mode = USER_CONTROL;
	
	yaw_pid.m = Iz - N_RDOT;
	yaw_pid.Km = 0.1 * yaw_pid.m;
	yaw_pid.wn = 1.56*Wb_YAW;
	yaw_pid.Kp = (yaw_pid.m + yaw_pid.Km) * yaw_pid.wn*yaw_pid.wn;
	yaw_pid.Kd = 2*yaw_pid.wn*yaw_pid.damp_ratio*(yaw_pid.m+yaw_pid.Km) - (yaw_pid.m/yaw_pid.T);
	yaw_pid.Ki = (yaw_pid.wn/10.0)*yaw_pid.Kp;
	
	heave_pid.m = M - N_WDOT;
	heave_pid.Km = 0.1 * heave_pid.m;
	heave_pid.wn = 1.56*Wb_HEAVE;
	heave_pid.Kp = (heave_pid.m + heave_pid.Km) * heave_pid.wn*heave_pid.wn;
	heave_pid.Kd = 2*heave_pid.wn*heave_pid.damp_ratio*(heave_pid.m+heave_pid.Km) - (heave_pid.m/heave_pid.T);
	heave_pid.Ki = (heave_pid.wn/10.0)*heave_pid.Kp;

	cur_mot.center = MID_PULSE_LENGTH;
	cur_mot.left = MID_PULSE_LENGTH;
	cur_mot.right = MID_PULSE_LENGTH;

}

void loop(void){
	mpx4250_depth = get_depth(); // Read Pressure Sensor
	pos.z = mpx4250_depth;

	

	if (first_sample) {
		accel.yaw = vel.yaw;
	
	} else {
		accel.yaw = gyro_accel_hpf(); 	
	}

	cur_mot.yaw_thrust = ((yaw_pid.m + yaw_pid.Km) * (accel_d.yaw + (vel_d.yaw/yaw_pid.T))) - yaw_pid_output - (yaw_pid.Km * accel.yaw);
	cur_mot.heave_thrust = ((heave_pid.m + heave_pid.Km) * (accel_d.z + (vel_d.z/heave_pid.T))) - heave_pid_output - (heave_pid.Km * accel.z) - GRAVITY_OFFSET;

	yaw_pid_ctrl.Compute();
	heave_pid_ctrl.Compute();

	if (user.mode == USER_CONTROL) {
		handle_manual_input();
	}

/*	Serial.print("left: ");
	Serial.print(cur_mot.left);
	Serial.print(" right: ");
	Serial.print(cur_mot.right);
	Serial.print(" center: ");
	Serial.println(cur_mot.center);*/
	delay(200);
}


void sendData(){ // i2c request callback
	
	if (cmd == SEND_ALL){
		i2c_send.data.cur_state = (byte)state;
		i2c_send.data.prev_state = 0; // tmp
		i2c_send.data.depth = mpx4250_depth;
		Wire.write(i2c_send.packet, sizeof(state_info_t));
	}
}

void receiveData(int byteCount){ // i2c recieve callback
	int i = 0;
	cmd = Wire.read();
	double2bytes_t b2d;
	//Serial.print("numB ");
	//Serial.println(byteCount);

	while (Wire.available()){
		i2c_buffer[i] = Wire.read();
		i++;
	}


	switch(cmd) {

		case ALL_IMU:
			i--;
			if ( (signed char)i2c_buffer[i] != -16) {
				return;
			}
			read_bytes(b2d.b, i2c_buffer, 1); accel.x = b2d.d;	
			read_bytes(b2d.b, i2c_buffer, 5); accel.y = b2d.d;
			read_bytes(b2d.b, i2c_buffer, 9); accel.z = b2d.d;
			read_bytes(b2d.b, i2c_buffer, 13); pos.yaw = b2d.d;
			break;

		case USER_CMD:
			user.direction =  i2c_buffer[1] |(i2c_buffer[2] << 8);
			user.power = i2c_buffer[3] |(i2c_buffer[4] << 8) ;
			break;

		case REF_TRAG:
			read_bytes(b2d.b, i2c_buffer, 1); pos_d.x = b2d.d;	
			read_bytes(b2d.b, i2c_buffer, 5); pos_d.y = b2d.d;
			read_bytes(b2d.b, i2c_buffer, 9); pos_d.z = b2d.d;
			read_bytes(b2d.b, i2c_buffer, 13); vel_d.roll = b2d.d;
			read_bytes(b2d.b, i2c_buffer, 17); vel_d.pitch = b2d.d;
			read_bytes(b2d.b, i2c_buffer, 21); vel_d.yaw = b2d.d;
			break;
		
		case YAW_DYN:
			read_bytes(b2d.b, i2c_buffer, 1); pos.yaw = b2d.d;	
			read_bytes(b2d.b, i2c_buffer, 5); vel.yaw = b2d.d;
			break;

		case HEAVE_DYN:
			read_bytes(b2d.b, i2c_buffer, 1); pos.z = b2d.d;	
			read_bytes(b2d.b, i2c_buffer, 5); vel.z = b2d.d;
			read_bytes(b2d.b, i2c_buffer, 9); accel.z = b2d.d;
			break;

		case SURGE_DYN:
			read_bytes(b2d.b, i2c_buffer, 1); pos.x = b2d.d;	
			read_bytes(b2d.b, i2c_buffer, 5); vel.x = b2d.d;
			read_bytes(b2d.b, i2c_buffer, 9); accel.x = b2d.d;
			break;

		case CHANGE_MODE:
			user.mode = i2c_buffer[0];
			break;
	}

}

double get_depth(){
	double v = analogRead(PRESSURE_SENSOR_PIN) * ADC_LSB;
	if (v < 0.204) return 20.0;
	if (v > 4.896) return 250.0;
	//float P =((v - MIN_OFFSET)/SENSITIVITY) + MIN_PRESSURE;
	double P = (v/SENSITIVITY) + MIN_PRESSURE;
	return ( 1000*(P-ATM_PRESSURE) / (WATER_DENSITY*GRAVITY) );	
}


/**
 * *** Actuation Model ***
 * 
 * ** Output forces and moments are found from input signals
 * ** K_a, K_b and K_c are thrust coefficents
 * ** NOTE: K_a = -1 * K_b
 *
 * X = (K_a * cur_mot.right) + (Kb * cur_mot.left)
 * Z = K_c * cur_mot.middle
 * N = (K_a * cur_mot.right * dist_right_arm) + (Kb * cur_mot.left * dist_left_arm)
 */

/*
 * Positive input causes ROV to surge forward
 * Negative input causes ROV to surge backward
 */
void surge(int thrust) {
	lr_motor_drive(MID_PULSE_LENGTH - (thrust / K_b), MID_PULSE_LENGTH+(thrust / K_a));
}
/*
 * Positive input causes ROV to YAW left of its heading 
 * Negative input causes ROV to YAW right of its heading
 */
void yaw(int thrust) {
	lr_motor_drive(MID_PULSE_LENGTH+(thrust / (K_b * DIST_L)), MID_PULSE_LENGTH + (thrust / (K_a * DIST_R) ));
}

void lr_motor_drive(int left, int right) {
	
	int dl = thrust_step;
	int dr = thrust_step;

	if ( ((left - 1500)*( cur_mot.left - 1500)) < 0 ) {
		motB.writeMicroseconds(MID_PULSE_LENGTH);
		cur_mot.left = MID_PULSE_LENGTH;
	}

	if ( ((cur_mot.right - 1500)*(right-1500)) < 0){
		motA.writeMicroseconds(MID_PULSE_LENGTH);
		cur_mot.right = MID_PULSE_LENGTH;
	}

	if (left < cur_mot.left) {
		dl *= -1;
	}
	
	if (right < cur_mot.right) {
		dr *= -1;
	}

	while ( cur_mot.left != left) {
		cur_mot.left += dl;
		motB.writeMicroseconds(cur_mot.left);
		delay(thrust_delay);
	}

	while (cur_mot.right != right) {
		cur_mot.right += dr;
		motA.writeMicroseconds(cur_mot.right);
		delay(thrust_delay);
	}
}

/*
 * Actuates center motor so ROV heaves. Input corresponds to the width
 * of the square pulse signal given to ESC.
 *
 * Positive input causes ROV to heave up
 * Negative input causes ROV to heave down
 */

void heave(int thrust) {

	int dc = thrust_step;
	int input = MID_PULSE_LENGTH + (thrust / K_HEAVE);

	if ( ((input - 1500)*(cur_mot.center - 1500)) < 0) {
		motC.writeMicroseconds(MID_PULSE_LENGTH);
		cur_mot.center = MID_PULSE_LENGTH;
	}

	if (input < cur_mot.center) {
		dc *= -1;
	}

	while (cur_mot.center != input) {
		cur_mot.center += dc;
		motC.writeMicroseconds(cur_mot.center);
		delay(thrust_delay);
	}
}

// type == 0 -> r_d
// type == 1 -> rdot_d
// type == 2 -> w_d
// type == 3 -> wdot_d
// type == 4 -> u_d
// type == 5 -> udot_d
double saturate(int type, double input) {
	switch (type) {
		case 0:
			return constrain(input, -1*YAW_RATE_D_MAX, YAW_RATE_D_MAX);
		case 1:
			return constrain(input, -1*YAW_ACCEL_D_MAX, YAW_ACCEL_D_MAX);
		case 2:
			return constrain(input, -1*HEAVE_RATE_D_MAX, HEAVE_RATE_D_MAX);
		case 3:
			return constrain(input, -1*HEAVE_RATE_D_MAX, HEAVE_ACCEL_D_MAX);
		case 4:
			return constrain(input, -1*SURGE_RATE_D_MAX, SURGE_RATE_D_MAX);
		case 5:
			return constrain(input, -1*SURGE_ACCEL_D_MAX, SURGE_ACCEL_D_MAX);
	}
}

inline void read_bytes(byte bytes[4], byte *p, int i) {
							bytes[0] = p[i];
							bytes[1] = p[i+1];
							bytes[2] = p[i+2];
							bytes[3] = p[i+3];
							}
void handle_manual_input(void) {
	switch(user.direction) {	
		case STOP: // left
			state = IDLE;
			cur_mot.left = MID_PULSE_LENGTH;
			cur_mot.right = MID_PULSE_LENGTH;
			cur_mot.center = MID_PULSE_LENGTH;
			motA.writeMicroseconds(cur_mot.right);
			motB.writeMicroseconds(cur_mot.left);
			motC.writeMicroseconds(cur_mot.center);
			break;
		case FWD_SURGE:
			constrain(user.power, -SURGE_CTRL_MAX, SURGE_CTRL_MAX);
			state = SURGING;
			//cur_mot.center = MID_PULSE_LENGTH;
			//motC.writeMicroseconds(MID_PULSE_LENGTH);		
			surge(user.power);
			break;
		case RWD_SURGE:
			state = SURGING;
			constrain(user.power, -SURGE_CTRL_MAX, SURGE_CTRL_MAX);
			//cur_mot.center = MID_PULSE_LENGTH;
			//motC.writeMicroseconds(MID_PULSE_LENGTH);
			surge(-user.power);
			break;
		case UP_HEAVE:
			constrain(user.power, -HEAVE_CTRL_MAX, HEAVE_CTRL_MAX);
			//cur_mot.left = MID_PULSE_LENGTH;
			//cur_mot.right = MID_PULSE_LENGTH;
			//motA.writeMicroseconds(MID_PULSE_LENGTH);
			//motB.writeMicroseconds(MID_PULSE_LENGTH);
			heave(user.power);
			break;
		case DWN_HEAVE:
			constrain(user.power, -HEAVE_CTRL_MAX, HEAVE_CTRL_MAX);
			//cur_mot.left = MID_PULSE_LENGTH;
			//cur_mot.right = MID_PULSE_LENGTH;
			//motA.writeMicroseconds(MID_PULSE_LENGTH);
			//motB.writeMicroseconds(MID_PULSE_LENGTH);
			state = HEAVING;
			heave( -user.power);
			break;
		case YAW_LEFT:
			state = YAWING;
			constrain(user.power, -YAW_CTRL_MAX, YAW_CTRL_MAX);
			///cur_mot.center = MID_PULSE_LENGTH;
			//motC.writeMicroseconds(MID_PULSE_LENGTH);		
			yaw( user.power);
			break;
		case YAW_RIGHT:
			state = YAWING;
			constrain(user.power, -YAW_CTRL_MAX, YAW_CTRL_MAX);
			//cur_mot.center = MID_PULSE_LENGTH;
			//motC.writeMicroseconds(MID_PULSE_LENGTH);		
			yaw(-user.power);
			break;
		case SHUT_DWN:
			state = IDLE;
			cur_mot.left = MID_PULSE_LENGTH;
			cur_mot.right = MID_PULSE_LENGTH;
			cur_mot.center = MID_PULSE_LENGTH;
			motA.writeMicroseconds(MID_PULSE_LENGTH);
			motB.writeMicroseconds(MID_PULSE_LENGTH);
			motC.writeMicroseconds(MID_PULSE_LENGTH);
			break;
		default:
			state = IDLE;
			cur_mot.left = MID_PULSE_LENGTH;
			cur_mot.right = MID_PULSE_LENGTH;
			cur_mot.center = MID_PULSE_LENGTH;
			motA.writeMicroseconds(MID_PULSE_LENGTH);
			motB.writeMicroseconds(MID_PULSE_LENGTH);
			motC.writeMicroseconds(MID_PULSE_LENGTH);
			break;
	}
}

double gyro_accel_hpf(){
    double alpha = (1.0/(YAW_W_HPF_CUTOFF*2*3.14)) / ((1.0/(YAW_W_HPF_CUTOFF*2*3.14)) + ( 1.0/IMU_SAMPLE_RATE ));
	return alpha *(prev_accel_yaw + vel.yaw - prev_vel_yaw);	
}
