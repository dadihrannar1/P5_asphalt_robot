bool sens_1_state = 0;
bool sens_2_state = 0;
bool sens_3_state = 0;
bool sens_4_state = 0;

float peak1 = 0;
float peak2 = 0;
float peak3 = 0;
float peak4 = 0;

long delta1 = 0;
long delta2 = 0;
int incomingByte= 0;
int tester = 0;
int onlyforward=0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
}

unsigned long last_time = 0;
void loop() {
  unsigned long Timer= millis();
  // put your main code here, to run repeatedly:
  // checks if the reflection is white or black
  int sens1 = analogRead(A2);
  int sens2 = analogRead(A3);
  int sens3 = analogRead(A0);
  int sens4 = analogRead(A1);
  //peakValue(sens1,sens2,sens3,sens4);
  bool sens_1_read = bool_read(sens1, 100);
  bool sens_2_read = bool_read(sens2, 90);
  bool sens_3_read = bool_read(sens3, 50);
  bool sens_4_read = bool_read(sens4, 80);

// test to see if the sensors are reading
/*
if (tester == 1){ 
  //Serial.print("sens1:");
  //Serial.println(peak1);
  //Serial.print(" ");
  //Serial.print("sens2:");
  //Serial.println(peak2);
  //Serial.print(" ");
  //Serial.print("sens3:");
  //Serial.println(peak3);
  //Serial.print(" ");
  //Serial.print("sens4:");
  //Serial.println(peak4);

  Serial.print("Bool1:");
  Serial.println(sens_1_read);
  Serial.print(" ");
  Serial.print("Bool2:");
  Serial.println(sens_2_read);
  Serial.print(" ");
  Serial.print("Bool3:");
  Serial.println(sens_3_read);
  //Serial.print("anal3:");
  //Serial.println(sens3);
  Serial.print(" ");
  Serial.print("Bool4:");
  Serial.println(sens_4_read);
  //Serial.print("sens1: ");
  //Serial.println(sens_1_read);
  //Serial.print("sens2: ");
  //Serial.println(sens_2_read);
  //Serial.print("sens3: ");
  //Serial.println(sens_3_read);
  //Serial.print("sens4: ");
 // Serial.println(sens_4_read);
   Serial.print("delta1:");
  Serial.print(delta1);
  Serial.print(" ");
  Serial.print("delta2:");
  Serial.println(delta2);
}
if(onlyforward==1){
if (sens_1_read != sens_1_state){
  delta1 += 1;
}
 if (sens_3_read != sens_3_state){
   delta2 += 1;
 }
 }
else{

*/
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
    Serial.print(delta1);
    Serial.print("$");
    Serial.print(delta2);
    Serial.print("$");
    Serial.println(timer);
    delta1=0;
    delta2=0;
  }
}