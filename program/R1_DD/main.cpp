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
constexpr uint8_t led = PA5;
//constexpr uint8_t DD[6] = {PC7, PC6, PB9, PB8, PB7, PB6};
constexpr uint8_t valve[3] = {PB6, PB7, PB8};

HardwareSerial SerialPC(USART2);
HardwareSerial SerialMDD1(PC11, PC10);
uint8_t MDD1_data[16];
HardwareSerial SerialMDD2(PA10, PA9);
DataFromPC received;
DataSTM now;

//myuart.hppの改造版（マイコン間通信用）
//DATA_LENGTH_に全データのバイト数を入れる
template<uint8_t DATA_LENGTH_> bool read(HardwareSerial& uart, uint8_t* container) {
  if (DATA_LENGTH_ > uart.available()) return false;

  static uint8_t ring[DATA_LENGTH_*2] = {};
  static uint8_t received_seq = 0;
  for (int i = 0; i < DATA_LENGTH_; ++i) {
    ring[i] = ring[i+DATA_LENGTH_];
  }
  uart.readBytes(ring+DATA_LENGTH_, DATA_LENGTH_);

  //リングバッファから先頭を検出して読み込み
  uint8_t tmp[2][DATA_LENGTH_];
	for (int i = 0; i < DATA_LENGTH_; ++i) {
		if (ring[i]==0xA5 && ring[i+1]==0xA5) {
			for (int j = 0; j < DATA_LENGTH_; ++j) {
				tmp[0][j] = ring[i+j];
				tmp[1][j] = ring[(i+j) % (DATA_LENGTH_*2)];
			}
			break;
		}
		if (i == DATA_LENGTH_-1) return false;
	}
	//チェックサム計算
	uint32_t check_sum[2] = {};
	for (int i = 2; i < DATA_LENGTH_-2; ++i) {
		check_sum[0] += tmp[0][i];
		check_sum[1] += tmp[1][i];
	}
	//新しいデータの場合、受信データとして格納
	if (tmp[0][DATA_LENGTH_-1]==check_sum[0]%256 && (tmp[0][DATA_LENGTH_-2]>received_seq || tmp[0][DATA_LENGTH_-2]-received_seq<-100)) {
		received_seq = tmp[0][2];
		for (int i = 0; i < DATA_LENGTH_; ++i) container[i] = tmp[0][i];
		return true;
	}
	if (tmp[1][DATA_LENGTH_-1]==check_sum[1]%256 && (tmp[1][DATA_LENGTH_-2]>received_seq || tmp[1][DATA_LENGTH_-2]-received_seq<-100)) {
		received_seq = tmp[1][2];
		for (int i = 0; i < DATA_LENGTH_; ++i) container[i] = tmp[1][i];
		return true;
	}
	return false;
}

uint8_t ring[17] = {}; // データサイズを17に増加
bool is_read = false;
uint8_t calculate_checksum(const uint8_t* data, size_t length) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < length; ++i) {
    checksum += data[i];
  }
  return checksum % 256;
}

void PC_receive(DataFromPC *data) {
  is_read = false;
  while (SerialPC.available() > 0) {
    for (int i = 0; i < 16; ++i) ring[i] = ring[i+1]; // データサイズを17に増加
    ring[16] = SerialPC.read(); // チェックサムも含める
    if (ring[0] == 0xA5 && ring[1] == 0xA5) {
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

enum moveType{go=0, stop};
uint32_t before = 0;
uint32_t pass;
void MDD1_simulate(moveType a) {
  if (a == go) {
    now.x += received.move_spd * cos(received.move_dir) * (pass / 1'000'000);
    now.y += received.move_spd * sin(received.move_dir) * (pass / 1'000'000);
    now.theta += (now.theta - received.rot) * (pass / 1'000'000);
  } else if (a == stop) {
    // 停止処理
  }
}

void setup() {
  for (uint8_t i : valve) pinMode(i, OUTPUT);
  pinMode(sw, INPUT_PULLUP);
  pinMode(led, OUTPUT);

  SerialPC.begin(9600);
  SerialMDD1.begin(9600);
  SerialMDD2.begin(9600);

  now.field = 0;
  now.action = 0;
  now.theta = 0;
}

void loop() {
  pass = micros() - before;
  before = micros();
  
  if (SerialPC.available() > 0) {
    PC_receive(&received);
  }
  read<16>(SerialMDD1, MDD1_data);
  //ここから
  //情報をfloatに変換するとこ

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

  SerialMDD1.write(digitalRead(sw) ? 0x00 : 0x01);
  SerialMDD2.write(digitalRead(sw) ? 0x00 : 0x01);
  digitalWrite(led, (digitalRead(sw) ? 0x00 : 0x01));

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