#include <Adafruit_MotorShield.h>

// Pin Setup
const byte COLL_PIN_RIGHT = 3;
const byte COLL_PIN_LEFT = 5;
// const byte PB_PIN = 2;
const byte TRIG_PIN_RIGHT = 6;
const byte ECHO_PIN_RIGHT = 7;
const byte TRIG_PIN_LEFT = 8;
const byte ECHO_PIN_LEFT = 9;
const byte LED_PIN = 13; 

// Set up Motor Shield
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); //create motor shield obj w/ default i2c address
Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(1); // connect to port m1
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(2); // m2

// Variable Setup
// Motors
const int maxMotorSpeed = 220;
volatile bool buttonPressed = false;

// Hebbian input indices
const int IN_UL = 0; // left ultrasonic
const int IN_UR = 1; // right ultrasonic
const int IN_CL = 2; // left collision
const int IN_CR = 3; // right collision

// Hebbian action indices
const int ACT_FORWARD = 0;
const int ACT_AVOID_LEFT = 1;
const int ACT_AVOID_RIGHT = 2;
const int ACT_STOP = 3;

// Learning parameters
float weights[4][4];
const float eta = 0.005; //eta = learning rate (greek symbol)

// Timing Vars + Backup Check
unsigned long actionStartTime = 0;
const unsigned long AVOID_DURATION = 400;
bool actionLocked = false;
const float forwardBias = 0.02;

// PB Interrupt
volatile unsigned long last_pb_time = 0; // for debounce delay
const unsigned long DEBOUNCE_MS = 200; //fixed debounce from datasheet

// Prototypes
void forwardStart();
void avoidRightStart();
void avoidLeftStart();
void stop();

void initWeights();

// States + Actions
typedef enum {FORWARD_STATE,
              AVOID_RIGHT_STATE,
              AVOID_LEFT_STATE,
              STOP_STATE} StateType;

typedef enum {FORWARD_START,
              AVOID_RIGHT_START,
              AVOID_LEFT_START,
              STOP} Action;


static void (*action_table[])(void) = { forwardStart, avoidRightStart, avoidLeftStart, stop};
volatile StateType curr_state;
volatile Action curr_action;

void setup() {
  Serial.begin(9600);

  AFMS.begin();

  // Sensors
  pinMode(COLL_PIN_RIGHT, INPUT_PULLUP);
  pinMode(COLL_PIN_LEFT, INPUT_PULLUP);

  pinMode(TRIG_PIN_RIGHT, OUTPUT);  
	pinMode(ECHO_PIN_RIGHT, INPUT); 
  pinMode(TRIG_PIN_LEFT, OUTPUT);  
	pinMode(ECHO_PIN_LEFT, INPUT); 

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  // pinMode(PB_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(PB_PIN), PB_ISR, FALLING);

  // Set Speed at 0
  L_MOTOR->setSpeed(0);
  R_MOTOR->setSpeed(0);

  // Initialize FSM + Weightings
  initWeights();
  curr_state = FORWARD_STATE;
  curr_action = FORWARD_START;
  action_table[curr_action]();
}

// Interrupts
// void PB_ISR() {
//   unsigned long now = millis();

//   if (now - last_pb_time < DEBOUNCE_MS) return;

//   last_pb_time = now;
//   buttonPressed = true;
// }

void loop() {
  // Interrupt Check
  // if (buttonPressed) {
  //   buttonPressed = false;

  //   // toggle LED
  //   if (digitalRead(LED_PIN) == LOW) {
  //     digitalWrite(LED_PIN, HIGH);
  //     curr_state = FORWARD_STATE;
  //     curr_action = FORWARD_START;
  //   } else {
  //     digitalWrite(LED_PIN, LOW);
  //     curr_state = STOP_STATE;
  //     curr_action = STOP;
  //   }
  // }

  // if (digitalRead(LED_PIN) == LOW) {
  //   curr_state = STOP_STATE;
  //   curr_action = STOP;
  //   action_table[curr_action]();
  //   return;
  // }

  // Action Lock Check
  if (actionLocked) {
    if (millis() - actionStartTime >= AVOID_DURATION) {
      actionLocked = false;
    } else {
      return;
    }
  }

  // Read Inputs
  long distRight = readDistance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
  long distLeft  = readDistance(TRIG_PIN_LEFT,  ECHO_PIN_LEFT);
  bool collRight = digitalRead(COLL_PIN_RIGHT);
  bool collLeft  = digitalRead(COLL_PIN_LEFT);

  // Input Array Update
  float inputs[4];
  inputs[IN_UL] = (distLeft < 8) ? 1.0 : 0.0;
  inputs[IN_UR] = (distRight < 8) ? 1.0 : 0.0;
  inputs[IN_CL] = (collLeft == LOW) ? 1.0 : 0.0;
  inputs[IN_CR] = (collRight == LOW) ? 1.0 : 0.0;

  // Output Array Update
  float outputs[4];
  outputs[ACT_FORWARD] = 0.0;
  outputs[ACT_AVOID_LEFT] = 0.0;
  outputs[ACT_AVOID_RIGHT] = 0.0;
  outputs[ACT_STOP] = 0.0;

  // Compute Action Scores
  for (int action = 0; action < 4; action++) {
    for (int input = 0; input < 4; input++) {
      outputs[action] += inputs[input] * weights[input][action];
    }
  }

  if (inputs[IN_UL] == 1.0 || inputs[IN_UR] == 1.0) {
    outputs[ACT_FORWARD] += forwardBias; // NOT STORED IN WEIGHTS AND DOES NOT ACCUMULATE
  }

  // Choose Winning Action
  bool allZero = true; 
  for (int i = 0; i < 4; i++) { // default action if all inputs 0
    if (inputs[i] != 0.0) {
      allZero = false;
      break;
    }
  }

  int chosenAction;
  if (allZero) {
    chosenAction = ACT_FORWARD;
  } else {
    chosenAction = 0;
    for (int action = 1; action < 4; action++) { //choose max action
      if (outputs[action] > outputs[chosenAction]) {
        chosenAction = action;
      }
    }
  }

  // Execute Action
  switch (chosenAction) {
    case ACT_FORWARD:
      curr_state = FORWARD_STATE;
      curr_action = FORWARD_START;
      break;
    case ACT_AVOID_LEFT:
      curr_state = AVOID_LEFT_STATE;
      curr_action = AVOID_LEFT_START;
      break;
    case ACT_AVOID_RIGHT:
      curr_state = AVOID_RIGHT_STATE;
      curr_action = AVOID_RIGHT_START;
      break;
    case ACT_STOP:
      curr_state = STOP_STATE;
      curr_action = STOP;
      break;
  }

  if (chosenAction == ACT_AVOID_LEFT || chosenAction == ACT_AVOID_RIGHT) { // lock check update
    actionLocked = true;
    actionStartTime = millis();
  }

  // Update Weightings
  if (!allZero && (chosenAction == ACT_AVOID_LEFT || chosenAction == ACT_AVOID_RIGHT)) {
    for (int input = 0; input < 4; input++) {
      if (inputs[input] == 1.0) {
        weights[input][chosenAction] += eta;

        if (weights[input][chosenAction] > 1.0) {
          weights[input][chosenAction] = 1.0;
        }
      }
    }
  }

  // Execute FSM
  action_table[curr_action]();

  // Testing Stuff
  Serial.print("Right Dist: ");
  Serial.print(distRight);
  Serial.print(" cm   ");

  Serial.print("Left Dist: ");
  Serial.print(distLeft);
  Serial.print(" cm   ");

  Serial.print("Right Switch: ");
  Serial.print(collRight);

  Serial.print("   Left Switch: ");
  Serial.println(collLeft);

  // Print all weights
  Serial.println("Weights:");

  Serial.print("UL -> F: ");
  Serial.print(weights[IN_UL][ACT_FORWARD]);
  Serial.print("  AL: ");
  Serial.print(weights[IN_UL][ACT_AVOID_LEFT]);
  Serial.print("  AR: ");
  Serial.print(weights[IN_UL][ACT_AVOID_RIGHT]);
  Serial.print("  ST: ");
  Serial.println(weights[IN_UL][ACT_STOP]);

  Serial.print("UR -> F: ");
  Serial.print(weights[IN_UR][ACT_FORWARD]);
  Serial.print("  AL: ");
  Serial.print(weights[IN_UR][ACT_AVOID_LEFT]);
  Serial.print("  AR: ");
  Serial.print(weights[IN_UR][ACT_AVOID_RIGHT]);
  Serial.print("  ST: ");
  Serial.println(weights[IN_UR][ACT_STOP]);

  Serial.print("CL -> F: ");
  Serial.print(weights[IN_CL][ACT_FORWARD]);
  Serial.print("  AL: ");
  Serial.print(weights[IN_CL][ACT_AVOID_LEFT]);
  Serial.print("  AR: ");
  Serial.print(weights[IN_CL][ACT_AVOID_RIGHT]);
  Serial.print("  ST: ");
  Serial.println(weights[IN_CL][ACT_STOP]);

  Serial.print("CR -> F: ");
  Serial.print(weights[IN_CR][ACT_FORWARD]);
  Serial.print("  AL: ");
  Serial.print(weights[IN_CR][ACT_AVOID_LEFT]);
  Serial.print("  AR: ");
  Serial.print(weights[IN_CR][ACT_AVOID_RIGHT]);
  Serial.print("  ST: ");
  Serial.println(weights[IN_CR][ACT_STOP]);

  Serial.println("------------------------");
}


// Actions
void forwardStart(){
  L_MOTOR->setSpeed(120);
  R_MOTOR->setSpeed(120);
  L_MOTOR->run(FORWARD);
  R_MOTOR->run(FORWARD);
}

void avoidRightStart() {
  // backup first
  L_MOTOR->setSpeed(150); R_MOTOR->setSpeed(150);
  L_MOTOR->run(BACKWARD); R_MOTOR->run(BACKWARD);
  delay(250);
  // then turn
  L_MOTOR->setSpeed(150); R_MOTOR->setSpeed(150);
  L_MOTOR->run(FORWARD);  R_MOTOR->run(BACKWARD);
}

void avoidLeftStart() {
  L_MOTOR->setSpeed(150); R_MOTOR->setSpeed(150);
  L_MOTOR->run(BACKWARD); R_MOTOR->run(BACKWARD);
  delay(250);
  L_MOTOR->setSpeed(150); R_MOTOR->setSpeed(150);
  L_MOTOR->run(BACKWARD); R_MOTOR->run(FORWARD);
}

void stop(){
  L_MOTOR->setSpeed(0);
  R_MOTOR->setSpeed(0);
  L_MOTOR->run(RELEASE);
  R_MOTOR->run(RELEASE);
}

// Helper
long readDistance(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2;
  return distance;
}

void initWeights() {
  weights[IN_UL][ACT_FORWARD] = 0.0;
  weights[IN_UL][ACT_AVOID_LEFT] = 0.0;
  weights[IN_UL][ACT_AVOID_RIGHT] = 0.0;
  weights[IN_UL][ACT_STOP] = 0.0;

  weights[IN_UR][ACT_FORWARD] = 0.0;
  weights[IN_UR][ACT_AVOID_LEFT] = 0.0;
  weights[IN_UR][ACT_AVOID_RIGHT] = 0.0;
  weights[IN_UR][ACT_STOP] = 0.0;

  weights[IN_CL][ACT_FORWARD] = 0.0;
  weights[IN_CL][ACT_AVOID_LEFT] = 0.1;
  weights[IN_CL][ACT_AVOID_RIGHT] = 0.7;
  weights[IN_CL][ACT_STOP] = 0.0;

  weights[IN_CR][ACT_FORWARD] = 0.0;
  weights[IN_CR][ACT_AVOID_LEFT] = 0.7;
  weights[IN_CR][ACT_AVOID_RIGHT] = 0.1;
  weights[IN_CR][ACT_STOP] = 0.0;
}
