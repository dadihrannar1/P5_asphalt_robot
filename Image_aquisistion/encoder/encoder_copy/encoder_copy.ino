bool sens_1_state = 0;
bool sens_2_state = 0;
bool sens_3_state = 0;
bool sens_4_state = 0;

int delta1 = 0;
int delta2 = 0;
int incomingByte= 0;
//int tester = 0;
int onlyforward=0;
int arraySize=0;

bool LED_STATE = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  pinMode(LED_BUILTIN, OUTPUT);
}

unsigned long last_time = 0;
void loop() {
  unsigned long Timer= millis();
  // put your main code here, to run repeatedly:
  // checks if the reflection is white or black
  float sens1 = analogRead(A2);
  float sens2 = analogRead(A3);
  float sens3 = analogRead(A0);
  float sens4 = analogRead(A1);

  bool sens_1_read = bool_read(sens1, 100);
  bool sens_2_read = bool_read(sens2, 90);
  bool sens_3_read = bool_read(sens3, 50);
  bool sens_4_read = bool_read(sens4, 100);

// Checks the state of the sensor to show how many ticks the wheel has gone around.
if (sens_1_read != sens_1_state){
  if(sens_2_read != sens_1_read){
    delta1 -= 1;
  }
  else{
    delta1 +=1;
  }
}
if (sens_4_read != sens_4_state){
    delta2 +=1;
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
  sens_3_state = sens_3_read;
  sens_4_state = sens_4_read;
}

bool bool_read (int reading, int thresh){
  if (reading >= thresh) {return true;}
  else {return false;}
}
// information sent to python
void send_delta(unsigned long timer){
  if (Serial.availableForWrite()){
    if(LED_STATE == 0){
      digitalWrite(LED_BUILTIN, HIGH);
      LED_STATE = 1;
    }else{
      digitalWrite(LED_BUILTIN, LOW);
      LED_STATE = 0;
    }
      
      Serial.print(delta1);
      Serial.print("$");
      Serial.print(delta2);
      Serial.print("$");
      Serial.println(timer);
      delta1=0;
      delta2=0;
  }
}
