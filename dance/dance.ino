// ______________________________________________
//                 Serial Structure
// ______________________________________________
//
// TYPE :  uint8 | float | float | 
// STRUC : CMD | PosX | PosY | Theta | SpeedX | SpeedY | SpeedT
// CMD = 0 : 0

// ______________________________________________
//                 PQ12 Variable
// ______________________________________________
// Potentiometer is connected to GPIO 34 input of PQ12 pot
const int PQ12_potPin = 34;

// PQ12 pin command output
const int PQ12_speedPin = 32;
const int PQ12_directionPin = 23;

// variable for storing the potentiometer value
int PQ12_potValue = 0;
const int PQ12_speedValue = 250;
const int PQ12_high = 4090;
const int PQ12_low = 10;

// setting PWM properties
const int freq = 96;
const int PQ12_PWMChannel = 4;
const int resolution = 8;

//Initialize State Machine for extraction motor
enum PQ12_enum {STOP, RETRACT, EXTRACT};
int PQ12_state = STOP;


//Def of PQ12 functions
void PQ12_init();
int PQ12_getPot(); // 10bits (0-4096)
void PQ12_extract(int Speed);
void PQ12_retract(int Speed);
void PQ12_stop();
void PQ12_state_machine();

char incomingChar;

void setup() {
  Serial.begin(115200);
  delay(1000);
  PQ12_init();
}

void loop() {
  Serial.println(PQ12_getPot());
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingChar = Serial.read();
    Serial.print("I receive: "); Serial.println(incomingChar);
    if (incomingChar == '0'){ 
      PQ12_state = RETRACT;
    }
    if (incomingChar == '1'){
      PQ12_state = EXTRACT;
    }
    PQ12_state_machine();
  }
}

void PQ12_init(){
  // PQ12_directionPin forward/Backward pin
  pinMode(PQ12_directionPin, OUTPUT);
  // configure Motor PWM functionalitites
  ledcSetup(PQ12_PWMChannel, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(PQ12_speedPin, PQ12_PWMChannel);
  }

int PQ12_getPot(){
  return analogRead(PQ12_potPin);
  }

void PQ12_extract(int Speed){
  digitalWrite(PQ12_directionPin, LOW);
  ledcWrite(PQ12_PWMChannel, Speed);
  }

void PQ12_retract(int Speed){
  digitalWrite(PQ12_directionPin, HIGH);
  ledcWrite(PQ12_PWMChannel, Speed);
  }

void PQ12_stop(){
  digitalWrite(PQ12_directionPin, LOW);
  ledcWrite(PQ12_PWMChannel, 0);
  }

void PQ12_state_machine(){
  switch(PQ12_state)
    {
    case STOP:
      PQ12_stop();
      break;
       
    case RETRACT:
      while(PQ12_getPot() > PQ12_low){
        PQ12_retract(PQ12_speedValue);
      }
      delay(200);
      PQ12_state = STOP;
      PQ12_state_machine();
      break;
 
    case EXTRACT:
      while(PQ12_getPot() < PQ12_high){
        PQ12_extract(PQ12_speedValue);
      }
      delay(200);
      PQ12_state = STOP;
      PQ12_state_machine();
      break;
    }
  }
