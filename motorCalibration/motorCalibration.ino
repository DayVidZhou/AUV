#include <Servo.h>

// ---------------------------------------------------------------------------
// Instructions
// 1. Set motor throttles to max
// 2. Connect Battery
// 3. Wait 2s then on 3rd tone set throttles to the middle
// 4. When motor makes a single tone set throttles to min
// 5. After 2s motor will make single tone followed by continoues tones indicating motor is ready
// ---------------------------------------------------------------------------
//A mid WHITE
//B right BLUE
//C left ORANGE
// Customize here pulse lengths as needed
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MID_PULSE_LENGTH 1500 // Middle pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs
// ---------------------------------------------------------------------------
Servo motA, motB, motC;
char data;
// ---------------------------------------------------------------------------

/**
 * Initialisation routine
 */
 
void setup() {
    Serial.begin(9600);
    
    motA.attach(9, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motB.attach(10, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motC.attach(11, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    
    displayInstructions();
}

/**
 * Main function
 */
void loop() {
    if (Serial.available()) {
        data = Serial.read();

        switch (data) {
            // 0
            case 48 : Serial.println("Sending minimum throttle");
                      motA.writeMicroseconds(MIN_PULSE_LENGTH);
                      motB.writeMicroseconds(MIN_PULSE_LENGTH);
                      motC.writeMicroseconds(MIN_PULSE_LENGTH);
            break;
			
			// 1
            case 49 : Serial.println("Sending middle throttle");
                      motA.writeMicroseconds(MID_PULSE_LENGTH);
                      motB.writeMicroseconds(MID_PULSE_LENGTH);
                      motC.writeMicroseconds(MID_PULSE_LENGTH);
            break;
  
			// 2
            case 50 : Serial.println("Sending maximum throttle");
                      motA.writeMicroseconds(MAX_PULSE_LENGTH);
                      motB.writeMicroseconds(MAX_PULSE_LENGTH);
                      motC.writeMicroseconds(MAX_PULSE_LENGTH);
            break;

            // 3
            case 51 : Serial.print("Running test in 3");
                      delay(1000);
                      Serial.print(" 2");
                      delay(1000);
                      Serial.println(" 1...");
                      delay(1000);
                      test();
            break;
        }
    }
}

/**
 * Test function: send min throttle to max throttle to each ESC.
 */
void test()
{
    for (int i = MID_PULSE_LENGTH; i <= MID_PULSE_LENGTH+150; i += 5) {
        motA.writeMicroseconds(i);
        delay(10);
    }
    delay(1000);
    motA.writeMicroseconds(MID_PULSE_LENGTH);
  	for (int i = MID_PULSE_LENGTH; i <= MID_PULSE_LENGTH+150; i += 5) {
          motB.writeMicroseconds(i);
          delay(10);
      }
     delay(1000);
     motB.writeMicroseconds(MID_PULSE_LENGTH);
  	for (int i = MID_PULSE_LENGTH; i <= MID_PULSE_LENGTH+150; i += 5) {
          motC.writeMicroseconds(i);
          delay(10);
     }
     delay(1000);
     motC.writeMicroseconds(MID_PULSE_LENGTH);
}

/**
 * Displays instructions to user
 */
void displayInstructions()
{  
    Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
    Serial.println("\t0 : Send min throttle");
    Serial.println("\t1 : Send mid throttle");
  	Serial.println("\t2 : Send max throttle");
    Serial.println("\t3 : Run test function\n");
}
