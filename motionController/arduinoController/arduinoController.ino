#include "arduinoController.h"
Servo motA, motB, motC;

const int thrust_delay = 5; // miliseconds
const int thrust_step = 1;
const int pulse_delay = 100;

pid_params_t yaw_pid = {	
							.wb = Wb_YAW, 
							.damp_ratio = 1.0,
							.wn = 1.56*Wb_YAW,
							.T = T_yaw,
							.K = K_yaw,  
							.m = Iz - N_WDOT,
							.Km = 0,
							.Kp = (m + Km) * wn*wn,
							.Kd = 2*wn*damp_ratio*(m+Km) - (m/T),
							.Ki = (wn/10.0)*Kp};
pid_params_t heave_pid = {
							.wb = Wb_HEAVE,
							.damp_ratio = 1.0,
							.wn = 1.56*Wb_HEAVE,
							.T = T_HEAVE,
							.K = K_HEAVE,  
							.m = m - N_WDOT,
							.Km = 0,
							.Kp = (m + Km) * wn*wn,
							.Kd = 2*wn*damp_ratio*(m+Km) - (m/T),
							.Ki = (wn/10.0)*Kp};
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
float mpx4250_depth = 0;
double yaw_pid_output;
double heave_pid_output;

PID yaw_pid_ctrl(&(yaw_pid_output), &(pos.yaw), &(pos_d.yaw), yaw_pid.Kp, yaw_pid.Ki, yaw_pid.Kd, DIRECT);
PID heave_pid_ctrl(&(heave_pid_output), &(pos.z), &(pos_d.z), heave_pid.Kp, heave_pid.Ki, heave_pid.Kd, DIRECT);


int yaw_controller(float);
void process_imu_data(int, int);
void lr_motor_drive(int, int);
void surge(int);
void yaw(int);
void heave(int);

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

void loop(void){
	mpx4250_depth = get_depth(); // Read Pressure Sensor
	pos.z = mpx4250_depth;

	yaw_pid_ctrl.Compute();
	heave_pid_ctrl.Compute();
	
	cur_mot.yaw_thrust = ((yaw_pid.m + yaw_pid.Km) * (accel_d.yaw + (vel_d.yaw/yaw_pid.T))) - yaw_pid_output - (yaw_pid.Km * accel.yaw);
	cur_mot.heave_thrust = ((heave_pid.m + heave_pid.Km) * (accel_d.yaw + (vel_d.heave/heave_pid.T))) - heave_pid_output - (heave_pid.Km * accel.heave) - GRAVITY_OFFSET;
	
	if (user.mode == MANUAL) {
		switch(user.direction) {
			case 0: // left
				state = IDLE;
				break;
			case 1:
				state = SURGING;
			case 2:
				state = SURGING;
				break;
			case 3:
			case 4:
				state = HEAVING;
				break;
			case 5:
			case 6:
				state = YAWING
		}

		switch (state){
			case IDLE:			
				motA.writeMicroseconds(MID_PULSE_LENGTH);
				motB.writeMicroseconds(MID_PULSE_LENGTH);
				motC.writeMicroseconds(MID_PULSE_LENGTH);		
				break;
			case SURGING:
				motC.writeMicroseconds(MID_PULSE_LENGTH);		
				surge(user.power);
				break;
			case YAWING:
				motC.writeMicroseconds(MID_PULSE_LENGTH);		
				yaw(user.power);
				break;
			case HEAVING:
				motA.writeMicroseconds(MID_PULSE_LENGTH);
				motB.writeMicroseconds(MID_PULSE_LENGTH);
				heave(user.power);
				break;
		}
	}
	
}


void sendData(){ // i2c request callback
	
	if (cmd == REG_R_ALL){
		i2c_send.data.cur_state = (byte)state;
		i2c_send.data.prev_state = 0; // tmp
		i2c_send.data.depth = mpx4250_depth;
		Wire.write(i2c_send.packet, sizeof(state_info_t));
	}
}

void receiveData(int byteCount){ // i2c recieve callback
	int i = 0;
	cmd = Wire.read();
	float2bytes_t b2f;

	switch(cmd){
		case REG_ALL_IMU:
			Wire.read();
			while (Wire.available()) {
				imu_data_b[i] = Wire.read();
				i++;
			}
			process_imu_data(REG_ALL_IMU,i);
			break;

		case REG_USER_CMD:
			Wire.read();
			user.direction = ((uint16_t)Wire.read() << 8) | Wire.read();
			user.power = ((uint16_t)Wire.read() << 8) | Wire.read();
			break;

		case REG_REF_TRAG:
			Wire.read();
			READ_BYTES(b2f.b); pos_d.x = b2f.f;
			READ_BYES(b2f.b); pos_d.z = b2f.f;
			READ_BYTES(b2f.b); pos_d.yaw = b2f.f;
			READ_BYTES(b2f.b); vel_d.x = b2f.f;
			READ_BYTES(b2f.b); vel_d.z = b2f.f;
			READ_BYTES(b2f.b); vel_d.yaw = b2f.f;
			READ_BYTES(b2f.b); accel_d.x = b2f.f;
			READ_BYTES(b2f.b); accel_d.z = b2f.f;
			READ_BYTES(b2f.b); accel_d.yaw = b2f.f;
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
	float * p = &pos;
	switch(type){
		case REG_ALL_IMU:
			for (i = 0; i < (sz>>2); i++){
				b2f.b[0] = imu_data_b[i*4];
				b2f.b[1] = imu_data_b[i*4+1];
				b2f.b[2] = imu_data_b[i*4+2];
				b2f.b[3] = imu_data_b[i*4+3];
				imu_data_f[i] = b2f.f;
				*p = b2f.f;
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
	if (thrust > 0) { // forward
		lr_motor_drive(-1 * thrust / K_b, thrust / K_a);
	} else { //backward
		lr_motor_drive(thrust / K_b, -1 * thrust / K_a);
	}
}

/*
 * Positive input causes ROV to YAW left of its heading 
 * Negative input causes ROV to YAW right of its heading
 */
void yaw(int thrust) {
	if (thrust > 0) { // yaw left
		lr_motor_drive(thrust / (K_b * DIST_L), thrust / (K_a * DIST_R));
	} else { // yaw right
		lr_motor_drive(-1 * thrust / (K_b * DIST_L), -1 * thrust/(K_a * DIST_R));
	}
}

void lr_motor_drive(int left, int right) {
	
	int dl = thrust_step;
	int dr = thrust_step;

	if ( (left * cur_mot.left) < 0 ) {
		motB.writeMicroseconds(MID_PULSE_LENGTH);
		cur_mot.left = MID_PULSE_LENGTH;
	}

	if ( (cur_mot.right * right) < 0){
		motA.writeMicroseconds(MID_PULSE_LENGTH);
		cur_mot.right = MID_PULSE_LENGTH;
	}

	if (left > cur_mot.left) {
		dl *= -1;
	}
	
	if (right > cur_mot.right) {
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
	int input = thrust / K_heave;

	if ( (input * cur_mot.center) < 0) 
		motC.writeMicroseconds(MID_PULSE_LENGTH);
		cur_mot.center = MID_PULSE_LENGTH;
	}

	if (input > cur_mot.center) {
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
