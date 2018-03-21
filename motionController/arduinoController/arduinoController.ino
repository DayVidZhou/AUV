#include "arduinoController.h"

Servo motA, motB, motC;

const int thrust_delay = 5; // miliseconds
const int thrust_step = 1;
const int pulse_delay = 100;

pid_params_t yaw_pid = {Wb_YAW, 1.0,T_YAW,K_YAW, 0};
							
pid_params_t heave_pid = {Wb_HEAVE,1.0,T_HEAVE,K_HEAVE, 0};
State state = IDLE;

//char cmd = 0;
I2C_CMD cmd;

position_t pos;
position_t accel;
position_t vel;
position_t accel_d;
position_t vel_d;
position_t pos_d;
user_cmd_t user;
//pid_contol_t yaw_ctrl;
//pid_contol_t heave_ctrl;
mot_sig_t cur_mot;

i2c_packet_t i2c_send;
byte imu_data_b[24]; // 6axis * (4bytes/float)
float imu_data_f[6]; // TODO: get rid of this
byte i2c_buffer[32];
float mpx4250_depth = 0;
double yaw_pid_output;
double heave_pid_output;
int user_power;

PID yaw_pid_ctrl(&(yaw_pid_output), &(pos.yaw), &(pos_d.yaw), yaw_pid.Kp, yaw_pid.Ki, yaw_pid.Kd, DIRECT);
PID heave_pid_ctrl(&(heave_pid_output), &(pos.z), &(pos_d.z), heave_pid.Kp, heave_pid.Ki, heave_pid.Kd, DIRECT);


int yaw_controller(float);
void process_imu_data(int, int);
void lr_motor_drive(int, int);
void surge(int);
void yaw(int);
void heave(int);
inline void read_bytes(byte bytes[4]);

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
	user.mode = 1;
	yaw_pid.Kp = (yaw_pid.m + yaw_pid.Km) * yaw_pid.wn*yaw_pid.wn;
	yaw_pid.Kd = 2*yaw_pid.wn*yaw_pid.damp_ratio*(yaw_pid.m+yaw_pid.Km) - (yaw_pid.m/yaw_pid.T);
	yaw_pid.Ki = (yaw_pid.wn/10.0)*yaw_pid.Kp;
	yaw_pid.m = Iz - N_RDOT;
	yaw_pid.wn = 1.56*Wb_YAW;
	heave_pid.Kp = (heave_pid.m + heave_pid.Km) * heave_pid.wn*heave_pid.wn;
	heave_pid.Kd = 2*heave_pid.wn*heave_pid.damp_ratio*(heave_pid.m+heave_pid.Km) - (heave_pid.m/heave_pid.T);
	heave_pid.Ki = (heave_pid.wn/10.0)*heave_pid.Kp;
	heave_pid.m = M - N_WDOT;
	heave_pid.wn = 1.56*Wb_HEAVE;
	cur_mot.center = MID_PULSE_LENGTH;
	cur_mot.left = MID_PULSE_LENGTH;
	cur_mot.right = MID_PULSE_LENGTH;
}

void loop(void){
	mpx4250_depth = get_depth(); // Read Pressure Sensor
	pos.z = mpx4250_depth;

	yaw_pid_ctrl.Compute();
	heave_pid_ctrl.Compute();

	cur_mot.yaw_thrust = ((yaw_pid.m + yaw_pid.Km) * (accel_d.yaw + (vel_d.yaw/yaw_pid.T))) - yaw_pid_output - (yaw_pid.Km * accel.yaw);
	cur_mot.heave_thrust = ((heave_pid.m + heave_pid.Km) * (accel_d.z + (vel_d.z/heave_pid.T))) - heave_pid_output - (heave_pid.Km * accel.z) - GRAVITY_OFFSET;

	if (user.mode) {
		switch(user.direction) {	
			case STOP: // left
				state = IDLE;
				motA.writeMicroseconds(MID_PULSE_LENGTH);
				motB.writeMicroseconds(MID_PULSE_LENGTH);
				motC.writeMicroseconds(MID_PULSE_LENGTH);
				break;
			case FWD_SURGE:
				state = SURGING;
				motC.writeMicroseconds(MID_PULSE_LENGTH);		
				surge(user.power);
				break;
			case RWD_SURGE:
				state = SURGING;
				motC.writeMicroseconds(MID_PULSE_LENGTH);
				surge(-user.power);
				break;
			case UP_HEAVE:
				motA.writeMicroseconds(MID_PULSE_LENGTH);
				motB.writeMicroseconds(MID_PULSE_LENGTH);
				heave(user.power);
				break;
			case DWN_HEAVE:
				state = HEAVING;
				motA.writeMicroseconds(MID_PULSE_LENGTH);
				motB.writeMicroseconds(MID_PULSE_LENGTH);
				heave( -user.power);
				break;
			case YAW_LEFT:
				state = YAWING;
				motC.writeMicroseconds(MID_PULSE_LENGTH);		
				yaw( user.power);
				break;
			case YAW_RIGHT:
				state = YAWING;
				motC.writeMicroseconds(MID_PULSE_LENGTH);		
				yaw(-user.power);
				break;
			case SHUT_DWN:
				state = IDLE;
				motA.writeMicroseconds(MID_PULSE_LENGTH);
				motB.writeMicroseconds(MID_PULSE_LENGTH);
				motC.writeMicroseconds(MID_PULSE_LENGTH);
				break;
			default:
				state = IDLE;
				motA.writeMicroseconds(MID_PULSE_LENGTH);
				motB.writeMicroseconds(MID_PULSE_LENGTH);
				motC.writeMicroseconds(MID_PULSE_LENGTH);
				break;
		}
		
	}
/*
	Serial.print("left: ");
	Serial.print(cur_mot.left);
	Serial.print(" right: ");
	Serial.print(cur_mot.right);
	Serial.print(" center: ");
	Serial.println(cur_mot.center);
*/
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
	signed char chksum;
	cmd = Wire.read();
	float2bytes_t b2f;
	//Serial.println(cmd);
	//Serial.println(byteCount);
	
	switch(cmd){
		case ALL_IMU:
			Wire.read();
			while (Wire.available()) {
				imu_data_b[i] = Wire.read();
				i++;
			}
			process_imu_data(ALL_IMU,i);
			break;

		case USER_CMD:
			Wire.read();
			user.direction =  Wire.read()|(Wire.read() << 8)  ;
			user.power = Wire.read() |(Wire.read() << 8) ;
			//user.power = Wire.read();
			//user.direction = Wire.read();
			chksum = Wire.read();
			/*Serial.print("pow: ");
			Serial.print(user.power);
			Serial.print(" dir: ");
			Serial.print(user.direction);
			Serial.print(" i: ");
			Serial.println(chksum);*/
			break;

		case REF_TRAG:
			Wire.read();
			read_bytes(b2f.b); pos_d.x = (double)b2f.f;
			read_bytes(b2f.b); pos_d.z = (double)b2f.f;
			read_bytes(b2f.b); pos_d.yaw = (double)b2f.f;
			read_bytes(b2f.b); vel_d.x = (double)b2f.f;
			read_bytes(b2f.b); vel_d.z = (double)b2f.f;
			read_bytes(b2f.b); vel_d.yaw = (double)b2f.f;
			read_bytes(b2f.b); accel_d.x = (double)b2f.f;
			read_bytes(b2f.b); accel_d.z = (double)b2f.f;
			read_bytes(b2f.b); accel_d.yaw = (double)b2f.f;
			break;
		case CHANGE_MODE:
			user.mode = Wire.read();
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


void process_imu_data(int type, int sz){
	int i = 0;
	float2bytes_t b2f;
	double * p = &(pos.x);
	switch(type){
		case ALL_IMU:
			for (i = 0; i < (sz>>2); i++){
				b2f.b[0] = imu_data_b[i*4];
				b2f.b[1] = imu_data_b[i*4+1];
				b2f.b[2] = imu_data_b[i*4+2];
				b2f.b[3] = imu_data_b[i*4+3];
				imu_data_f[i] = b2f.f;
				*p = (double)b2f.f;
				p++;
			}
			//Serial.println(imu_data_f[2],4);
			break;
	}
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

inline void read_bytes(byte bytes[4]) {
							bytes[0] = Wire.read();
							bytes[1] = Wire.read();
							bytes[2] = Wire.read();
							bytes[3] = Wire.read();
							}
