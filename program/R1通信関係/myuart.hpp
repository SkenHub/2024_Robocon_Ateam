/*
 * myuart.hpp
 *
 *  Created on: Jun 23, 2024
 *      Author: tikuwa404
 */

#ifndef MYUART_HPP_
#define MYUART_HPP_

enum USBname {
	USB_A,
	USB_B
};

//DATA_LENGTH_にデータ部のバイト数を入れる
//チェックサムの確認と先頭検出と新しいデータかの確認をしてくれる
template<uint8_t DATA_LENGTH_> class MyUart {
public:
	MyUart(void) {};
	void init(USBname usb, uint32_t baudrate) {
		switch (usb) {
		case USB_A:
			uart_.init(A9, A10, SERIAL1, baudrate);
			break;
		case USB_B:
			uart_.init(C10, C11, SERIAL3, baudrate);
			break;
		}
		uart_.startDmaRead(raw_, clen_*2);
	}
	bool read(uint8_t* container) {
		//リングバッファから先頭を検出して読み込み
		uint8_t tmp[2][clen_];
		for (int i = 0; i < clen_; ++i) {
			if (raw_[i]==0xA5 && raw_[i+1]==0xA5) {
				for (int j = 0; j < clen_; ++j) {
					tmp[0][j] = raw_[i+j];
					tmp[1][j] = raw_[(i+j) % (clen_*2)];
				}
				break;
			}
			if (i == clen_-1) return false;
		}
		//チェックサム計算
		uint32_t check_sum[2] = {};
		for (int i = 2; i < clen_-2; ++i) {
			check_sum[0] += tmp[0][i];
			check_sum[1] += tmp[1][i];
		}
		//新しいデータの場合、受信データとして格納
		if (tmp[0][clen_-1]==check_sum[0]%256 && (tmp[0][clen_-2]>received_seq_ || tmp[0][clen_-2]-received_seq_<-100)) {
			received_seq_ = tmp[0][2];
			for (int i = 0; i < DATA_LENGTH_; ++i) container[i] = tmp[0][i+2];
			return true;
		}
		if (tmp[1][clen_-1]==check_sum[1]%256 && (tmp[1][clen_-2]>received_seq_ || tmp[1][clen_-2]-received_seq_<-100)) {
			received_seq_ = tmp[1][2];
			for (int i = 0; i < DATA_LENGTH_; ++i) container[i] = tmp[1][i+2];
			return true;
		}
		return false;
	}
	void write(uint8_t* data, uint8_t size) {
		uint8_t tmp[clen_];
		tmp[0] = 0xA5;
		tmp[1] = 0xA5;
		uint32_t checksum = 0;
		for (int i = 0; i < size; ++i) {
			tmp[i+2] = data[i];
			checksum += data[i];
		}
		tmp[size+2] = ++send_seq_;
		tmp[size+3] = checksum % 256;
		uart_.write(tmp, clen_);
	}
private:
	Uart uart_;
	const uint8_t clen_ = DATA_LENGTH_+4; //0xA5,0xA5,seq,checksum
	uint8_t raw_[(DATA_LENGTH_+4)*2];
	uint8_t received_seq_ = 0;
	uint8_t send_seq_ = 0;
};

#endif /* MYUART_HPP_ */
