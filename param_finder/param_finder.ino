#include <Servo.h>
#include <Wire.h>
#include <stdint.h>

#define SLAVE_ADDRESS 0x04

#define YAW_TEST 1
#define HEAVE_TEST 2
#define SURGE_TEST 3

#define TEST_TYPE_MASK (uint16_t)0xf
#define AMP_MASK (uint16_t)0xf0
#define PERIOD_MASK (uint16_t)0x0f00
#define DELAY_SHIFT (uint16_t)0xf000

#define STATE_NULL 0
#define STATE_TESTTYPE_GOOD 1
#define STATE_AMP_GOOD 2
#define STATE_T_GOOD 3
#define STATE_DELAY_GOOD 4
#define STATE_TESTING 5

#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MID_PULSE_LENGTH 1500 // Middle pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs
#define RANGE MAX_PULSE_LENGTH-MID_PULSE_LENGTH
//SURGE FORWARD THEN BACK
//HEAVE UP THEN DOWN
//YAW 
Servo motA, motB, motC;
int ncycles = 2;

volatile char cmd = 0;
volatile int state = 0;

volatile int amplitude = 0;
volatile int period = 0;
volatile int pulse_delay = 0;
volatile int test_type = 0;

void yaw_test();
void heave_test();
void yaw();
void heave();
void start_test(void);

void setup() {
    Serial.begin(9600);
 	
	// initialize i2c as slave
	Wire.begin(SLAVE_ADDRESS);

	// define callbacks for i2c communication
	Wire.onReceive(receiveData);
	Wire.onRequest(sendData);
   
    motA.attach(9, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // right
    motB.attach(11, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // left 
    motC.attach(10, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // middle
	delay(1000);
  
  motA.writeMicroseconds(MID_PULSE_LENGTH);
  motB.writeMicroseconds(MID_PULSE_LENGTH);
  motC.writeMicroseconds(MID_PULSE_LENGTH);
//  delay(5000);
//  amplitude = 200;
//  period = 500;
//  pulse_delay = 2000;
//  yaw();
}

void loop(void) {

  if (state == STATE_TESTING){
      start_test();
      state = STATE_NULL;
   }
   delay(200);
}

// callback for received data
void receiveData(int byteCount){

	while(Wire.available()) {
		cmd = Wire.read();
		//Serial.print("data received: ");
		//Serial.println(number);

		switch(state){
			case STATE_NULL:
				test_type = cmd;
				state = STATE_TESTTYPE_GOOD;
        Serial.print(cmd);
				//Serial.println(" STATE_TESTTYPE_GOOD");
				break;
			case STATE_TESTTYPE_GOOD:
				amplitude = 100*cmd;
				state = STATE_AMP_GOOD;
        Serial.println(cmd);
				//Serial.println(" STATE_AMP_GOOD");
				break;
			case STATE_AMP_GOOD:
				period = cmd*100;
				state = STATE_T_GOOD;
				Serial.print(cmd);
				//Serial.println(" STATE_T_GOOD");
				break;
			case STATE_T_GOOD:
				pulse_delay = cmd*1000;
				state = STATE_DELAY_GOOD;
       Serial.print(cmd);
				//Serial.println(" STATE_DELAY_GOOD");
				break;
			case STATE_DELAY_GOOD:
				Serial.println(test_type);
        Serial.println(amplitude);
				Serial.println(period);
				Serial.println(pulse_delay);
				if (cmd == 1){
					state = STATE_TESTING;
					Serial.println(" STATE_TESTING");
					//start_test();
         //state = STATE_NULL;
				}
				break;
			case STATE_TESTING:
				break;
			default:
				Serial.println("Invalid cmd");


		}
	}
}

void start_test(void){
	if (test_type == YAW_TEST){
		//yaw_test();
   yaw();
	} 
  else if (test_type == HEAVE_TEST) {
    heave();
   }
  else if (test_type == SURGE_TEST) {
    surge();
   }
}

// callback for sending data
void sendData(){
	Wire.write(state);
}
void yaw() {
  delay(3000);
    for (int i = MID_PULSE_LENGTH; i <= (MID_PULSE_LENGTH+amplitude); i += 5) {
        motA.writeMicroseconds(i);
        motB.writeMicroseconds(i);
        delay(10);
    }
	delay(period/2);
    motA.writeMicroseconds(MID_PULSE_LENGTH);
    motB.writeMicroseconds(MID_PULSE_LENGTH);
	delay(pulse_delay);
	Serial.println("first half");
	for (int i = MID_PULSE_LENGTH; i >= (MID_PULSE_LENGTH-amplitude); i -= 5) {
        motA.writeMicroseconds(i);
        motB.writeMicroseconds(i);
        delay(10);
    }

	delay(period/2);
    motA.writeMicroseconds(MID_PULSE_LENGTH);
    motB.writeMicroseconds(MID_PULSE_LENGTH);
	delay(pulse_delay);
 Serial.println("done!");

}

void heave() {
  delay(3000);
    for (int i = MID_PULSE_LENGTH; i <= (MID_PULSE_LENGTH+amplitude); i += 1) {
        motC.writeMicroseconds(i);
        delay(1);
    }
	delay(period/2);
    motC.writeMicroseconds(MID_PULSE_LENGTH);
	delay(pulse_delay);
	
	for (int i = MID_PULSE_LENGTH; i >= (MID_PULSE_LENGTH-amplitude); i -= 1) {
        motC.writeMicroseconds(i);
        delay(1);
    }

	delay(period/2);
    motC.writeMicroseconds(MID_PULSE_LENGTH);
	delay(pulse_delay);
}

void surge() {
  delay(3000);
    for (int i = MID_PULSE_LENGTH; i <= (MID_PULSE_LENGTH+amplitude); i += 5) {
        motA.writeMicroseconds(i);
        int j = MID_PULSE_LENGTH - i + MID_PULSE_LENGTH;
        motB.writeMicroseconds(j);
        delay(10);
    }
  delay(period/2);
    motA.writeMicroseconds(MID_PULSE_LENGTH);
    motB.writeMicroseconds(MID_PULSE_LENGTH);
  delay(pulse_delay);
  Serial.println("first half");
  for (int i = MID_PULSE_LENGTH; i >= (MID_PULSE_LENGTH-amplitude); i -= 5) {
        motA.writeMicroseconds(i);
        int j = MID_PULSE_LENGTH - i + MID_PULSE_LENGTH;
        motB.writeMicroseconds(j);
        delay(10);
    }

  delay(period/2);
    motA.writeMicroseconds(MID_PULSE_LENGTH);
    motB.writeMicroseconds(MID_PULSE_LENGTH);
  delay(pulse_delay);
 Serial.println("done!");
}
void yaw_test(){//(int T, int A, int wait_time) {
	for (int i = 0; i < ncycles; i++){
		yaw();
	}
}

void heave_test(int T, int A, int wait_time) {
	for (int i = 0; i < ncycles; i++){
		heave();
	}
}
