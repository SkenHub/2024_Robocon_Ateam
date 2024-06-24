#include <Arduino.h>
#include "uartconv.hpp"

constexpr uint8_t sw = PC13;
constexpr uint8_t led = PA5;
constexpr uint8_t valve[3] = {PB6, PB7, PB8};
constexpr uint8_t FROM_MDD1_DATASIZE = 12;
constexpr uint8_t TO_MDD1_DATASIZE = 12;

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
union DataFromMDD1 {
  float f[FROM_MDD1_DATASIZE/4];
  uint8_t u8[FROM_MDD1_DATASIZE];
};
union DataToMDD1 {
  float f[TO_MDD1_DATASIZE/4];
  uint8_t u8[TO_MDD1_DATASIZE];
};

HardwareSerial SerialPC(USART2);
DataFromPC received;

HardwareSerial SerialMDD1(PA10,PA9);
uint8_t MDD1_raw[(FROM_MDD1_DATASIZE+4)*2];
UartConv<FROM_MDD1_DATASIZE> MDD1_conv;
DataFromMDD1 MDD1_data;

HardwareSerial SerialMDD2(PC11,PC10);

DataSTM now;

uint8_t debug[16];

uint8_t ring[17] = {}; // データサイズを17に増加
bool is_read = false;
uint8_t calculate_checksum(const uint8_t* data, size_t length) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < length; ++i) {
    checksum += data[i];
  }
  return checksum % 256;
}

void PC_receive(DataFromPC *data, uint8_t *raw) {
  is_read = false;
  while (SerialPC.available() > 0) {
    for (int i = 0; i < 16; ++i) ring[i] = ring[i+1]; // データサイズを17に増加
    ring[16] = SerialPC.read(); // チェックサムも含める
    if (ring[0] == 0xA5 && ring[1] == 0xA5) {
      for (int i = 0; i < 16; ++i) raw[i] = ring[i];
      uint8_t received_checksum = ring[16];
      uint8_t calculated_checksum = calculate_checksum(ring, 16); // 16バイトのデータに対してチェックサムを計算

      if (received_checksum == calculated_checksum) {
        is_read = true;
        break;
      }
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
  uint8_t send_data[17]; // データサイズを17に増加
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

  uint8_t checksum = calculate_checksum(send_data + 2, 14); // チェックサムを計算
  send_data[16] = checksum; // チェックサムをデータに追加

  SerialPC.write(send_data, 17);
}

void MDD1_send(float s, float d, float r) {
  DataToMDD1 raw;
  raw.f[0] = s;
  raw.f[1] = d;
  raw.f[2] = r;
  uint8_t tmp[TO_MDD1_DATASIZE+4];
  MDD1_conv.write(raw.u8, TO_MDD1_DATASIZE, tmp);
  SerialMDD1.write(tmp, TO_MDD1_DATASIZE+4);
}

void setup() {
  for (uint8_t i : valve) pinMode(i, OUTPUT);
  pinMode(sw, INPUT_PULLUP);
  pinMode(led, OUTPUT);

  SerialPC.begin(9600);
  SerialMDD1.begin(115200);
  SerialMDD2.begin(9600);

  now.x = 0;
  now.y = 0;
  now.theta = 0;
  now.field = 0;
  now.action = 0;
}

void loop() {  
  if (SerialPC.available() > 0) {
    PC_receive(&received,debug);
  }
  if (SerialMDD1.available() >= (FROM_MDD1_DATASIZE+4)*2) {
    SerialMDD1.readBytes(MDD1_raw, (FROM_MDD1_DATASIZE+4)*2);
  }
  //ここから
  //情報をfloatに変換するとこ
  MDD1_conv.read(MDD1_raw, MDD1_data.u8);
  //digitalWrite(led, (MDD1_data.f[0]==1.4013e-45f)? HIGH : LOW);
  now.x = MDD1_data.f[0];
  now.y = MDD1_data.f[1];
  now.theta = MDD1_data.f[2];
  
  SerialMDD1.write(debug, 16);

  /*
  if (now.action == 0) {
    MDD1_send(received.move_spd, received.move_dir, received.rot);
  } else if (now.action == 1) {
    MDD1_send(0, 0, now.theta);
    digitalWrite(valve[0], HIGH);
  } else if (now.action == 255) {
    MDD1_send(0, 0, now.theta);
  } else if (now.action == 5) {
    MDD1_send(received.move_spd, received.move_dir, received.rot);
  } else {
      MDD1_send(0, 0, now.theta);
      digitalWrite(valve[1], HIGH);
      digitalWrite(valve[2], HIGH);
  }
  */

  //MDD2_send
  PC_send(now);
}

/*
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
*/