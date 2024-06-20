#include <Arduino.h>

struct DataFromPC {
  float move_spd; //[m/s]
  float move_dir; //[deg]
  float rot; //[deg]
  uint8_t field; //0~1
  uint8_t action; //0~5
};

struct DataSTM {
  float x; //[mm]
  float y; //[mm]
  float theta; //[deg]
  uint8_t field; //0~1
  uint8_t action; //0~5
  uint8_t finished; //0~5
};

constexpr uint8_t sw = PC13;
//constexpr uint8_t DD[6] = {PC7, PC6, PB9, PB8, PB7, PB6};
constexpr uint8_t valve[3] = {PB6, PB7, PB8};

HardwareSerial SerialPC(USART2);
//HardwareSerial SerialMDD1(PA10,PA9);
DataFromPC received;
DataSTM now;

void PC_receive(DataFromPC *data) {
  uint8_t raw[14];
  SerialPC.readBytes(raw, 14);
  union {float f; uint8_t u8[4];} conv;
  
  for (int i=0; i<4; ++i) conv.u8[i] = raw[0+i];
  data->move_spd = conv.f;
  for (int i=0; i<4; ++i) conv.u8[i] = raw[4+i];
  data->move_dir = conv.f;
  for (int i=0; i<4; ++i) conv.u8[i] = raw[8+i];
  data->rot = conv.f;
  data->field = raw[12];
  data->action = raw[13];
}

void PC_send(DataSTM data) {
  uint8_t send_data[14];
  union {float f; uint8_t u8[4];} conv;

  conv.f = data.x;
  for (int i=0; i<4; ++i) send_data[0+i] = conv.u8[i];
  conv.f = data.y;
  for (int i=0; i<4; ++i) send_data[4+i] = conv.u8[i];
  conv.f = data.theta;
  for (int i=0; i<4; ++i) send_data[8+i] = conv.u8[i];
  send_data[12] = data.field;
  send_data[13] = data.action;

  SerialPC.write(send_data, 14);
}

enum moveType{go=0, stop};
uint32_t before = 0;
uint32_t pass;
void MDD1_send(moveType a) {
  if (a == go) {
    now.x += received.move_spd*cos(received.move_dir)*(pass/1'000'000);
    now.y += received.move_spd*sin(received.move_dir)*(pass/1'000'000);
    now.theta += (now.theta-received.rot)*(pass/1'000'000);
  } else if (a == stop) {

  }
}

uint32_t timer;
void simulate() {
  if (received.action == now.finished) {
    now.action = 0;
    MDD1_send(go);
  } else {

  switch (received.action) {
    case 1:
      if (now.action != 1) {
        timer = millis();
        now.action = 1;
        digitalWrite(valve[2], HIGH);
      } else {
        if (millis()-timer > 100) {
          now.finished = 1;
          now.action = 0;
        }
      }
      break;
    case 2:
      if (now.action != 2) {
        timer = millis();
        now.action = 2;
        digitalWrite(valve[0], HIGH);
      } else {
        static bool flag = false;
        if (millis()-timer > 100) {
          if (millis()-timer > 200) {
            now.finished = 2;
            now.action = 0;
          } else {
            if (!flag) {
              digitalWrite(valve[1], HIGH);
              flag = true;
            }
          }
        }
      }
      break;
    case 3:
      if (now.action != 3) {
        timer = millis();
        now.action = 3;
        digitalWrite(valve[0], HIGH);
      } else {
        static bool flag = false;
        if (millis()-timer > 100) {
          if (millis()-timer > 200) {
            now.finished = 3;
            now.action = 0;
          } else {
            if (!flag) {
              digitalWrite(valve[1], HIGH);
              flag = true;
            }
          }
        }
      }
      break;
    case 4:
      if (now.action != 4) {
        timer = millis();
        now.action = 4;
        digitalWrite(valve[0], HIGH);
      } else {
        static bool flag = false;
        if (millis()-timer > 100) {
          if (millis()-timer > 200) {
            now.finished = 4;
            now.action = 0;
          } else {
            if (!flag) {
              digitalWrite(valve[1], HIGH);
              flag = true;
            }
          }
        }
      }
      break;
    case 5:
      if (now.action != 5) {
        timer = millis();
        now.action = 5;
      } else {
        MDD1_send(go);
      }
      break;
    case 0:
      now.action = 0;
      MDD1_send(go);
      break;
    case 255:
      now.action = 255;
      MDD1_send(stop);
      break;
  }

  }
}

void setup() {
  // put your setup code here, to run once:
  for (uint8_t i : valve) pinMode(i, OUTPUT);
  pinMode(sw, INPUT_PULLUP);

  SerialPC.begin(9600);
  SerialPC.println(0);

  now.action = 0;
  now.finished = 0;
}

void loop() {
  pass = micros()-before;
  before = micros();
  
  if (SerialPC.available() >= 14) {
    PC_receive(&received);
    now.field = received.field;
  }

  simulate();

  PC_send(now);
}
