TicI2C StepperX(12);
TicI2C StepperY(11);
TicI2C StepperT(10);

void setup() {
  Wire.begin();
  delay(20);

  StepperX.exitSafeStart();
  StepperY.exitSafeStart();
  StepperT.exitSafeStart();
  
}

void loop() {
  // put your main code here, to run repeatedly:

}

void steppers_init()  {
  
}
