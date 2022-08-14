/*
 * Code to read Modbus vibration sensor based on accelerometer
 * and Campbell 034B Wind Set
 * All data is sent to FTTECH Gateway by Xbee
 * 
 * Autor: Jhonatan da Silva Cruz
 * From FTTECH Software Team
 * 
 * created: 2022-08-12
*/
#include "FTTech_SAMD51Clicks.h"

#define PIN_CH1_4a20 A1 // Wind direction
#define PIN_CH2_4a20 A0 // Wind Speed

int BAUDRATE = 115200;
bool DEBUG = false;

/*********************************
*        VIBRATION SENSOR        *
*********************************/
uint8_t _serial_data[100];
uint8_t _data_length = 0;
uint8_t _send_data[100];
unsigned long myTime;
float accel[3];
bool FLAG = true;
uint8_t manda2[6] = {0xF,0x7,0x4,0xB,0x5,0xE};
char accel_x[50];
char accel_y[50];
char accel_z[50];

/**********************************
*            INTERVALO            *
**********************************/
unsigned long previousMillis = 0; // Stores the last time reading was made
const long interval = 10000;       // Interval at which to read (milliseconds)

/************************************
*            WIND SENSOR            *
************************************/
float dir_temp[4];
float spd_temp[4];
char sensor_dir[50];
char sensor_spd[50];

/**************************************
*            CONFIGURATION            *
**************************************/

void setup() {
    //4-20 Click uses external feeding, so
    //you doesn't need to power it with the click output.
    digitalWrite(52, HIGH); //Put Click 4 ON
    FTClicks.turnON_5V(); // Enable 5V output
    FTClicks.turnON(2);   // Enable the 3.3V output for Click 2
    digitalWrite(A9, HIGH); //Xbee Reset
    delay(100);
    Serial2.begin(9600);
    pinMode(A4,OUTPUT);
    pinMode(44,OUTPUT);
    digitalWrite(A4,LOW);
    digitalWrite(44,LOW);

    if(DEBUG) Serial.begin(9600);
    else Serial4.begin(BAUDRATE);

    if(DEBUG) Serial.println(F("*****************INICIOU*****************"));
    delay(1000);
}

void loop() {

  // ===========================================================================================
  //                 ===================== Efetua a Leitura =======================
  // ===========================================================================================
  // the interval at which you want to read and send data.
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {

    // ==================================== Measuring data ====================================
    uint8_t aux = 0;
    float CH1_4a20; // Wind direction
    float CH2_4a20; // Wind speed

    while(aux < 3){
      dir_temp[aux] = analogRead(PIN_CH1_4a20) * (3.3/1023);
      spd_temp[aux] = analogRead(PIN_CH2_4a20) * (3.3/1023);
      aux++;
    }

    //Trata as leituras de Direção do Vento
    if(dir_temp[0] != 0 && dir_temp[1] != 0 && dir_temp[2] != 0) CH1_4a20 = dir_temp[2];
    else CH1_4a20 = 0;

    //Trata as leituras de Velocidade do Vento
    if(spd_temp[0] != 0 && spd_temp[1] != 0 && spd_temp[2] != 0) CH2_4a20 = spd_temp[2];
    else CH2_4a20 = 0;

  // ==========================================================================================

    // Reading data from modbus on Serial2
    if(Serial2.available()){
        ReadFloats(accel);
    }

    if(millis() - myTime > 500){
        if (FLAG){
            SendChar2(manda2,6);
            FLAG = false;
        }
        else {
            FLAG = true;
        }
        myTime = millis();
    }

    // save the last time reading
    previousMillis = currentMillis;

    if(DEBUG){
      Serial.println("Canal 1: ");
      Serial.print("\tTensão: "); Serial.println(CH1_4a20);
      Serial.println("DIREÇÃO: " + String(getWindDirection(CH1_4a20),3));
      Serial.println("Canal 2 : ");
      Serial.print("\tTensão: "); Serial.println(CH2_4a20);
      Serial.println("SPEED: " + String(getWindSpeed(CH2_4a20),3));
      Serial.println("Vibração: ");
      Serial.print("\tX: "); Serial.println(accel[0]);
      Serial.print("\tY: "); Serial.println(accel[1]);
      Serial.print("\tZ: "); Serial.println(accel[2]);
    }

    else{
        // ===== Send Wind Direction to Xbee =====
        sprintf(sensor_dir, "FTMS,DIR,%f", getWindDirection(CH1_4a20));
        Serial4.write(sensor_dir);
        delay(1000);

        // ===== Send Wind Speed to Xbee =====
        sprintf(sensor_spd, "FTMS,SPD,%f", getWindSpeed(CH2_4a20));
        Serial4.write(sensor_spd);
        delay(1000);

        // ===== Send Accel X to Xbee =====
        sprintf(accel_x, "FTMS,ACX,%f", accel[0]);
        Serial4.write(accel_x);
        delay(1000);

        // ===== Send Accel Y to Xbee =====
        sprintf(accel_y, "FTMS,ACY,%f", accel[1]);
        Serial4.write(accel_y);
        delay(1000);

        // ===== Send Accel Z to Xbee =====
        sprintf(accel_z, "FTMS,ACZ,%f", accel[2]);
        Serial4.write(accel_z);
        delay(1000);
    }
  }
}

float getWindDirection(float CH1_4a20){

  float maxVoltage = 2.93;              // VOLTS
  float minVoltage = 0.19;              // VOLTS
  float tensao = CH1_4a20;

  if(tensao > maxVoltage) maxVoltage = tensao;
  if(tensao < minVoltage) minVoltage = tensao;

  float grads = (tensao - minVoltage) * 360 / (maxVoltage - minVoltage); // GRAUS

  float interval = 45;

  String direction[8] = {"Norte","Nordeste","Leste","Sudeste","Sul","Sudoeste","Oeste","Noroeste"};

  String message = "";

  if( grads >= (360 - (interval/2)) || grads < (interval/2)){
    message = "NORTE " + (String)grads + "º";
  }
  else if(grads >= (interval/2) && grads < 3*(interval/2)){
    message = "NORDESTE " + (String)grads + "º";
  }
  else if(grads >= 3*(interval/2) && grads < 5*(interval/2)){
    message = "LESTE " + (String)grads + "º";
  }
  else if(grads >= 5*(interval/2) && grads < 7*(interval/2)){
    message = "SUDESTE " + (String)grads + "º";
  }
  else if(grads >= 7*(interval/2) && grads < 9*(interval/2)){
    message = "SUL " + (String)grads + "º";
  }
  else if(grads >= 9*(interval/2) && grads < 11*(interval/2)){
    message = "SUDOESTE " + (String)grads + "º";
  }
  else if(grads >= 11*(interval/2) && grads < 13*(interval/2)){
    message = "OESTE " + (String)grads + "º";
  }
  else if(grads >= 13*(interval/2) && grads < 15*(interval/2)){
    message = "NOROESTE " + (String)grads + "º";
  }

  return grads;
}

float getWindSpeed(float CH2_4a20){

  float maxVoltage = 2.93;  // VOLTS
  float minVoltage = 0.19;  // VOLTS
  float tensao = CH2_4a20;

  if(tensao > maxVoltage) maxVoltage = tensao;
  if(tensao < minVoltage) minVoltage = tensao;

  float spd = (tensao - minVoltage) * 50 / (maxVoltage - minVoltage); // VELOCIDADE

  if(spd < 0.5) spd = 0;

  return spd;
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
