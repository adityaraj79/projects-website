// DEFINE THE PINS

#define Enable 8

// 1 = left stepper, 2 = right stepper

#define dirPin1 5
#define stepPin1 2

#define dirPin2 6
#define stepPin2 3


// DEFINE THE CONSTANTS

const float W = 1620; // building width, in mm
const float w = 210;  // distance between the two cable guides, in mm
const float r = 9;    // radius of the drum, in mm


// DEFINE THE VARIABLES

// (Initial) angles, in rad
float theta1 = 30 * (2*PI)/360;
float theta2 = 25 * (2*PI)/360;

// Cable lengths, from the robot to the top of the building, in mm
float L1;
float L2;

// (Initial) position on the building, in mm
int x = 0;
int y = 0; // starting from the ground

// Displacements allowed
int dx = 10;
int dy = 10;


void setup() {
  
  pinMode(Enable, OUTPUT);
  
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin1, OUTPUT);
  
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  
  pinMode(4,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(11,OUTPUT);
  
  digitalWrite(Enable, LOW); // to enable the CNC shield
  
}


void loop() {
  
  // Make the servos continuously turn
  analogWrite(7,255);
  digitalWrite(4, LOW);
  digitalWrite(11, LOW);
  
  // Move on the building

  if ( x % (2*200) == 0 && x <= 1400 && y < 1800 ) {
    moveUp(dy);
    y = y + dy;
  }
  else if ( x % (2*200) == 200 && x <= 1400 && y > 800) {
    moveUp(-dy);
    y = y - dy;
  }
  else if ( (y == 800 || y == 1800) && x < 1400 ) {
    moveRight(dx);
    x = x + dx;
  }
  
}


// FUNCTION TO MOVE VERTICALLY: move up (dy > 0), move down (dy < 0)

void moveUp(float dy) {

  // Cable lengths according to the angles, in mm
  L2 = ( W - w ) / ( sin(theta1) * cos(theta2)/cos(theta1) + sin(theta2) );
  L1 = L2 * cos(theta2)/cos(theta1);

  // Variations in length in order to move vertically, in mm
  float dL1 = sqrt( pow(dy,2) + pow(L1,2) - 2*L1*dy*cos(theta1) ) - L1;
  float dL2 = sqrt( pow(dy,2) + pow(L2,2) - 2*L2*dy*cos(theta2) ) - L2;

  // Angles of rotation of the drums, in rad
  float dphi1 = dL1/r;
  float dphi2 = dL2/r;

  // For sixteenth stepping: 1 pulse = 1/16 step, and 3.6° = 1 step
  int npulse1 = round( 16/3.6 * 360/(2*PI) * dphi1 );
  int npulse2 = round( 16/3.6 * 360/(2*PI) * dphi2 );

  // Define the direction of rotation

  if (npulse1 > 0) { // i.e. dL1>0 -> go down
    digitalWrite(dirPin1, LOW);
  }
  else if (npulse1 < 0) { // i.e. dL1<0 -> go up
    digitalWrite(dirPin1, HIGH);
    npulse1 = -npulse1;
  }

  if (npulse2 > 0) { // i.e. dL2>0 -> go down
    digitalWrite(dirPin2, LOW);
  }
  else if (npulse2 < 0) { // i.e. dL2<0 -> go up
    digitalWrite(dirPin2, HIGH);
    npulse2 = -npulse2;
  }

  // Move

  for (int i = 1; i <= max(npulse1, npulse2); i++) {
    if (i <= npulse1) {
      digitalWrite(stepPin1, HIGH);
    }
    if (i <= npulse2) {
      digitalWrite(stepPin2, HIGH);
    }
    delay(3); // ms
    digitalWrite(stepPin1, LOW);
    digitalWrite(stepPin2, LOW);
    delay(3); // ms
  }
  
}


// FUNCTION TO MOVE HORIZONTALLY: move right (dx > 0), move left (dx < 0)

void moveRight(float dx) {

  // Cable lengths according to the angles
  L2 = ( W - w ) / ( sin(theta1) * cos(theta2)/cos(theta1) + sin(theta2) );
  L1 = L2 * cos(theta2)/cos(theta1);

  // Variations in length in order to move horizontally
  float dL1 = sqrt( pow(dx,2) + pow(L1,2) + 2*L1*dx*sin(theta1) ) - L1;
  float dL2 = sqrt( pow(dx,2) + pow(L2,2) - 2*L2*dx*sin(theta2) ) - L2;

  // Angles of rotation of the drums
  float dphi1 = dL1/r;
  float dphi2 = dL2/r;

  // For sixteenth stepping: 1 pulse = 1/16 step, and 3.6° = 1 step
  int npulse1 = round( 16/3.6 * 360/(2*PI) * dphi1 );
  int npulse2 = round( 16/3.6 * 360/(2*PI) * dphi2 );

  // Define the direction of rotation

  if (npulse1 > 0) { // i.e. dL1>0 -> go right
    digitalWrite(dirPin1, LOW);
  }
  else if (npulse1 < 0) { // i.e. dL1<0 -> go left
    digitalWrite(dirPin1, HIGH);
    npulse1 = -npulse1;
  }

  if (npulse2 > 0) { // i.e. dL2>0 -> go left
    digitalWrite(dirPin2, LOW);
  }
  else if (npulse2 < 0) { // i.e. dL2<0 -> go right
    digitalWrite(dirPin2, HIGH);
    npulse2 = -npulse2;
  }

  // Move

  for (int i = 1; i <= max(npulse1, npulse2); i++) {
    if (i <= npulse1) {
      digitalWrite(stepPin1, HIGH);
    }
    if (i <= npulse2) {
      digitalWrite(stepPin2, HIGH);
    }
    delay(3); // ms
    digitalWrite(stepPin1, LOW);
    digitalWrite(stepPin2, LOW);
    delay(3); // ms
  }
  
}
