// USING 3S V7 AND BOARDS 1.0.4

uint8_t _serial_data[100];
uint8_t _data_length = 0;
uint8_t _send_data[100];
unsigned long myTime;
float accel[3];
bool FLAG = true;

uint8_t manda2[6] = {0xF,0x7,0x4,0xB,0x5,0xE};

void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);
  for(uint8_t i=0;i<20;i++){
    delay(50);
    digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
  }
  beginPower();
  myTime = 0;
  myTime = millis();
}

void loop() {
//  if(Serial2.available()){
//    ReadFloats(accel);
//    for(uint8_t i=0;i<3;i++){
//      Serial.print(String(accel[i]));
//      Serial.print("\t");
//    }
//    Serial.println("");
//  }
  if(Serial2.available()){
    ReadFloats(accel);
    Serial.print("Serial2:");
    for(uint8_t i = 0;i<3;i++){
      Serial.print(accel[i]);
      Serial.print("\t");
    }
    Serial.println(""); 
  }
  if(Serial3.available()){
    _data_length = RX_Serial3(_serial_data);
    Serial.print("Serial3:");
    Serial.write(_serial_data,_data_length);
    Serial.println(""); 
  }
  if(millis() - myTime > 500){
    if (FLAG){
      SendChar2(manda2,6);
      FLAG = false;
    } else {
      FLAG = true;
    }
    myTime = millis();
  }
}

void beginPower(void){
  Serial.begin(115200);
  while(!Serial);
  Serial2.begin(9600);
  Serial3.begin(9600);
  pinMode(POWER_5V,OUTPUT);
  pinMode(CLICK_TWO,OUTPUT);
  pinMode(CLICK_THREE,OUTPUT);
  pinMode(A4,OUTPUT);
  pinMode(44,OUTPUT);
  pinMode(A9,OUTPUT);
  pinMode(47,OUTPUT);
  digitalWrite(POWER_5V,HIGH);
  digitalWrite(CLICK_TWO,HIGH);
  digitalWrite(CLICK_THREE,HIGH);
  digitalWrite(A4,LOW);
  digitalWrite(44,LOW);
  digitalWrite(A9,LOW);
  digitalWrite(47,LOW);
  
}

uint8_t RX_Serial(uint8_t _buf[]) {
  uint8_t _RX_size = 0;
  while (Serial.available()) {
    _buf[_RX_size] = Serial.read();
    _RX_size++;
    delay(5);
  }
  return _RX_size;
}

uint8_t RX_Serial2(uint8_t _buf[]) {
  uint8_t _RX_size = 0;
  while (Serial2.available()) {
    _buf[_RX_size] = Serial2.read();
    _RX_size++;
    delay(5);
  }
  return _RX_size;
}

uint8_t RX_Serial3(uint8_t _buf[]) {
  uint8_t _RX_size = 0;
  while (Serial3.available()) {
    _buf[_RX_size] = Serial3.read();
    _RX_size++;
    delay(5);
  }
  return _RX_size;
}

void SendChar2(uint8_t char_buffer[], uint8_t _data_length){
  digitalWrite(A4, HIGH); // PUT MB CHIP TO TRANSMISSION MODE
  digitalWrite(44, HIGH); // PUT MB CHIP TO TRANSMISSION MODE
  delay(20); // WAITS A LITTLE BIT
  // SEND CHARACTERS ONE BY ONE
  for (uint32_t i = 0; i < _data_length; i++) {
    Serial2.write(char_buffer[i]);
  }
  delay(20); // WAITS A LITTLE BIT
  digitalWrite(A4, LOW); // RETURNS MB CHIP TO RECEPTION MODE
  digitalWrite(44, LOW); // RETURNS MB CHIP TO RECEPTION MODE
  delay(20); // WAITS A LITTLE BIT
  return;
}

void SendChar3(uint8_t char_buffer[], uint8_t _data_length){
  digitalWrite(A9, HIGH); // PUT MB CHIP TO TRANSMISSION MODE
  digitalWrite(47, HIGH); // PUT MB CHIP TO TRANSMISSION MODE
  delay(20); // WAITS A LITTLE BIT
  // SEND CHARACTERS ONE BY ONE
  for (uint32_t i = 0; i < _data_length; i++) {
    Serial3.write(char_buffer[i]);
  }
  delay(20); // WAITS A LITTLE BIT
  digitalWrite(A9, LOW); // RETURNS MB CHIP TO RECEPTION MODE
  digitalWrite(47, LOW); // RETURNS MB CHIP TO RECEPTION MODE
  delay(20); // WAITS A LITTLE BIT
  return;
}

void SendChar(uint8_t char_buffer[], uint8_t _data_length){
  // SEND CHARACTERS ONE BY ONE
  for (uint32_t i = 0; i < _data_length; i++) {
    Serial.write(char_buffer[i]);
  }
  return;
}

void ReadFloats(float _outputs[]){
  _data_length = RX_Serial2(_serial_data);
  union float_conv {
    uint8_t charac[12];
    float floater[3];
  };
  float_conv _temp;
  for (uint8_t i=0;i<12;i++){
    _temp.charac[i] = _serial_data[i];
  }
  for (uint8_t i=0;i<3;i++){
    _outputs[i] = _temp.floater[i];
  }
}
