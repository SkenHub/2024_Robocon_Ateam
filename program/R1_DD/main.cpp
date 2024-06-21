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
};

constexpr uint8_t sw = PC13;
//constexpr uint8_t DD[6] = {PC7, PC6, PB9, PB8, PB7, PB6};
constexpr uint8_t valve[3] = {PB6, PB7, PB8};

HardwareSerial SerialPC(USART2);
HardwareSerial SerialMDD1(PA10,PA9);
DataFromPC received;
DataSTM now;

uint8_t ring[16] = {};
bool is_read = false;
void PC_receive(DataFromPC *data) {
  is_read = false;
  while (SerialPC.available() > 0) {
    for (int i = 0; i < 15; ++i) ring[i] = ring[i+1];
    ring[15] = SerialPC.read();
    if (ring[0]==0xA5 && ring[1]==0xA5) {
      is_read = true;
      break;
    }
  }

  if (is_read) {
    union {float f; uint8_t u8[4];} conv;

    for (int i=0; i<4; ++i) conv.u8[i] = ring[2+i];
    data->move_spd = conv.f;
    for (int i=0; i<4; ++i) conv.u8[i] = ring[6+i];
    data->move_dir = conv.f;
    for (int i=0; i<4; ++i) conv.u8[i] = ring[10+i];
    data->rot = conv.f;
    data->field = ring[14];
    data->action = ring[15];

    now.field = received.field;
    now.action = received.action;
  }
}

void PC_send(DataSTM data) {
  uint8_t send_data[16];
  union {float f; uint8_t u8[4];} conv;

  send_data[0] = 0xA5;
  send_data[1] = 0xA5;
  conv.f = data.x;
  for (int i=0; i<4; ++i) send_data[2+i] = conv.u8[i];
  conv.f = data.y;
  for (int i=0; i<4; ++i) send_data[6+i] = conv.u8[i];
  conv.f = data.theta;
  for (int i=0; i<4; ++i) send_data[10+i] = conv.u8[i];
  send_data[14] = data.field;
  send_data[15] = data.action;

  SerialPC.write(send_data, 16);
}

enum moveType{go=0, stop};
uint32_t before = 0;
uint32_t pass;
void MDD1_simulate(moveType a) {
  if (a == go) {
    now.x += received.move_spd*cos(received.move_dir)*(pass/1'000'000);
    now.y += received.move_spd*sin(received.move_dir)*(pass/1'000'000);
    now.theta += (now.theta-received.rot)*(pass/1'000'000);
  } else if (a == stop) {

  }
}

void setup() {
  // put your setup code here, to run once:
  for (uint8_t i : valve) pinMode(i, OUTPUT);
  pinMode(sw, INPUT_PULLUP);

  SerialPC.begin(9600);
  SerialMDD1.begin(9600);

  now.field = 0;
  now.action = 0;
  now.theta = 0;
}

void loop() {
  pass = micros()-before;
  before = micros();
  
  if (SerialPC.available() > 0) {
    PC_receive(&received);
  }

  switch (now.action) {
    case 0:
      MDD1_simulate(go);
      break;
    case 255:
      MDD1_simulate(stop);
      break;
    case 1:
      //mode R100
      [[fallthrough]];
    case 2:
      //mode RB
      [[fallthrough]];
    case 3:
      //mode RO
      [[fallthrough]];
    case 4:
      //mode RH
      [[fallthrough]];
    case 5:
      //mode Ball
      [[fallthrough]];
    default:
      //mode R100,RB,RO,RH,Ball
      pass = pass;
  }

  SerialMDD1.write(digitalRead(sw)==1?1:0);

  PC_send(now);
}
