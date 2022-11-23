bool sens_1_state = 0;
bool sens_2_state = 0;
bool sens_3_state = 0;
bool sens_4_state = 0;

long delta1 = 0;
long delta2 = 0;
int incomingByte= 0;
int tester = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
}

unsigned long last_time = 0;
void loop() {
  unsigned long Timer= millis();
  // put your main code here, to run repeatedly:
  // checks if the reflection is white or black
  bool sens_1_read = bool_read(analogRead(A2), 100);
  bool sens_2_read = bool_read(analogRead(A3), 100);
  bool sens_3_read = bool_read(analogRead(A9), 150);
  bool sens_4_read = bool_read(analogRead(A10), 150);

// test to see if the sensors are reading
if (tester == 1){ 
  Serial.print("A2:");
  Serial.print(sens_1_read);
  Serial.print(" ");
  Serial.print("A3:");
  Serial.println(sens_2_read);
}

// Checks the state of the sensor to show how many ticks the wheel has gone around.
  if ((sens_1_read != sens_1_state && sens_1_read == sens_2_state) || (sens_2_read != sens_2_state && sens_2_read != sens_1_state)){
    delta1 += 1;
  }
  if ((sens_2_read != sens_2_state && sens_2_read == sens_1_state) || (sens_1_read != sens_1_state && sens_1_read != sens_2_state)){
    delta1 -= 1;
  }
  // syncing to the python code
   if (Serial.available() > 0){
  incomingByte = Serial.read();
    if (incomingByte > 1){
      send_delta(Timer);
    incomingByte=0;
    }
  }
  // updates the states of the sensors
  sens_1_state = sens_1_read;
  sens_2_state = sens_2_read;
}

bool bool_read (int reading, int thresh){
  if (reading >= thresh) {return true;}
  else {return false;}
}
// information sent to python
void send_delta(unsigned long timer){
  if (Serial.availableForWrite()){
    Serial.print(delta1/2);
    Serial.print("$");
    Serial.println(timer);
    delta1=0;
    delta2=0;
  }
}