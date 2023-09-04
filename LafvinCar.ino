#include <NewPing.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>



// =========================== PIN DEFINITIONS =================================
// DEFINING LEFT SIDE WHEEL PINS
#define WHEEL_LB 2
#define WHEEL_LF 4
// DEFINING RIGHT SIDE WHEEL PINS
#define WHEEL_RB 7
#define WHEEL_RF 8
// DEFINING PWM PINS FOR SPEED CONTROL
#define LSPEED 5
#define RSPEED 6

// FOR TURNING OFF MOVEMENT WITHOUT TURNING OFF THE ARDUINO
#define KILLSWITCH 13

#define SERVO A2

#define IRSENSOR 10

// FOR ULTRASOUND DISTANCE SENSOR
#define TRIGGER_PIN A1
#define ECHO_PIN A0
#define MAX_DISTANCE 300

// ============================================================================



// =================== FUNCTION DECLARATIONS ==================================

// Changes car speed (Recommended range: 100-200)
void setSpeed(unsigned char speed);

// Functions for car movement
void forward();
void right();
void left();
void backward();
void stop();

// Functions for servo control
void lookLeft();
void lookForward();
void lookRight();

// Mood types
void happy();
void sad();

// Some utility functions
bool secondsLoop(int seconds);
void avoidObstacle();

// ============================================================================



// ====================== VARIABLE/OBJECT INITIALIZATIONS ====================

// CAR SPEED
// I don't know why it's a char, just keeping it the same as lafvin's
//(Recommended range: 130-200)
unsigned char speed = 130;



// Mood variables
int numberOfMoods = 2;
int mood;



// LCD OBJECT INITIALIZATION FOR LCD DISPLAY
LiquidCrystal_I2C lcd(0x27,20,4);
Servo servo;

// Ultrasound Distance Sensor object initialization
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
//===========================================================================





// ===================== MAIN ARDUINO CODE ====================================
void setup() {
  
  // SERVO OBJECT SETUP
  servo.attach(SERVO);

  // LCD OBJECT SETUP
  lcd.init();
  lcd.backlight();

  // SETTING MOTOR PIN MODES TO OUTPUT
  pinMode(WHEEL_LB, OUTPUT);
  pinMode(WHEEL_LF, OUTPUT);
  pinMode(WHEEL_RB, OUTPUT);
  pinMode(WHEEL_RF, OUTPUT);
  pinMode(LSPEED, OUTPUT);
  pinMode(RSPEED, OUTPUT);

  pinMode(KILLSWITCH, INPUT);
  pinMode(IRSENSOR, INPUT);
  
  setSpeed(speed);

  lcd.print("Starting...");
  delay(5000);

}

void loop() {
  if (digitalRead(KILLSWITCH)){

    if (!digitalRead(IRSENSOR)){
      
      lookForward();
      setSpeed(speed);
      
      if (secondsLoop(10)){
        mood = (random(1,numberOfMoods + 1));

        lcd.clear();
        lcd.setCursor(0, 0);
      }

      switch(mood){
        case 1:
          happy();
          break;
        
        case 2:
          happy();
          break;
        
        default:
          happy();
      }

    } else{
      lcd.setCursor(0,0);
      lcd.print("Oh no! Too high!");
      lcd.setCursor(0,1);
      lcd.print("D:");

      setSpeed("0");
      stop();
      lookLeft();
      delay(500);
      lookRight();
      delay(500);

    }
  } else{
    setSpeed("0");
    stop();
    lookForward();
    
    lcd.setCursor(0,0);
    lcd.print("Killswitched :(");
  }
}
// ===========================================================================





// ==================== MOOD FUNCTIONS =======================================

void happy(){
  int sonarInt;
  setSpeed(130);

  sonarInt = sonar.ping_cm();
  delay(35);

  if (sonarInt < 20){
    if (random (1,4) == 1){

      lcd.clear();
      lcd.print("Backing up!");

      backward();
      delay(1000);
      stop();
    }

    lcd.clear();
    lcd.print("Where do I go?");
    avoidObstacle();
    lcd.clear();
  }

  lcd.setCursor(0, 0);
  lcd.print("Onwards!");
  forward();
}

void sad(){
  stop();
  servo.write(90);
}

//============================================================================





//==================== FUNCTION CODES ========================================

// Changes car speed (Recommended range: 130-200)
void setSpeed(unsigned char speed){
  analogWrite(LSPEED, speed);
  analogWrite(RSPEED, speed);
}



// Functions for car movement
void forward(){ // Go Forward
  digitalWrite(WHEEL_RB, LOW);
  digitalWrite(WHEEL_RF, HIGH);
  digitalWrite(WHEEL_LB, LOW);
  digitalWrite(WHEEL_LF, HIGH);
}

void right(){ // Turn Right
  digitalWrite(WHEEL_RB, HIGH);
  digitalWrite(WHEEL_RF, LOW);
  digitalWrite(WHEEL_LB, LOW);
  digitalWrite(WHEEL_LF, HIGH);
}

void left(){ // Turn Left
  digitalWrite(WHEEL_RB, LOW);
  digitalWrite(WHEEL_RF, HIGH);
  digitalWrite(WHEEL_LB, HIGH);
  digitalWrite(WHEEL_LF, LOW);
}

void backward(){ // Go Backwards
  digitalWrite(WHEEL_RB, HIGH);
  digitalWrite(WHEEL_RF, LOW);
  digitalWrite(WHEEL_LB, HIGH);
  digitalWrite(WHEEL_LF, LOW);
}

void stop(){ // Stops movement
  digitalWrite(WHEEL_RB, HIGH);
  digitalWrite(WHEEL_RF, HIGH);
  digitalWrite(WHEEL_LB, HIGH);
  digitalWrite(WHEEL_LF, HIGH);  
}



// Servo Control
void lookLeft(){
  servo.write(20);
}

void lookForward(){
  servo.write(90);
}

void lookRight(){
  servo.write(160);
}



// Utility functions

bool secondsLoop(int seconds){
  bool flag;

  if (millis()%(seconds*1000)==0){
    flag = true;
  }
  else {
    flag = false;
  }

  return flag;
}

void avoidObstacle(){
  long L,R;

  stop();
  servo.write(20);
  delay(300);
  R = sonar.ping_cm();


  servo.write(160);
  delay(300);
  L = sonar.ping_cm();
 

  if (L >= R){
    right();
    servo.write(160);
    delay(700);
    stop();
  } else{
    left();
    servo.write(20);
    delay(700);
    stop();
  }

  servo.write(90);
  delay(300);
}
//===========================================================================
