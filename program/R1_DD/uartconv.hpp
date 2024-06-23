/*
 * uartconv.hpp
 *
 *  Created on: Jun 23, 2024
 *      Author: tikuwa404
 */

#ifndef MYUART_HPP_
#define MYUART_HPP_

#include <Arduino.h>

//myuart.hppの改造版（platformIOのマイコン間通信用）
//DATA_LENGTH_にデータ部のバイト数を入れる
//チェックサムの確認と先頭検出と新しいデータかの確認をしてくれる
template<uint8_t DATA_LENGTH_> class UartConv {
public:
	//rawの長さは(DATA_LENGTH_+4)*2
	//containerの長さはDATA_LENGTH_
	bool read(uint8_t* raw, uint8_t* container) {
		//リングバッファから先頭を検出して読み込み
		uint8_t tmp[2][clen_];
		for (int i = 0; i < clen_; ++i) {
			if (raw[i]==0xA5 && raw[i+1]==0xA5) {
				for (size_t j = 0; j < clen_; ++j) {
					tmp[0][j] = raw[i+j];
					tmp[1][j] = raw[(i+j) % (clen_*2)];
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
	//dataはrawより4長い(dataの長さ=clen_)
	//rawの長さ:size、dataの長さ:size+4
	void write(uint8_t* raw, uint8_t size, uint8_t* data) {
		data[0] = 0xA5;
		data[1] = 0xA5;
		uint32_t checksum = 0;
		for (int i = 0; i < size; ++i) {
			data[i+2] = raw[i];
			checksum += raw[i];
		}
		data[size+2] = ++send_seq_;
		data[size+3] = checksum % 256;
	}
private:
	const uint8_t clen_ = DATA_LENGTH_+4; //0xA5,0xA5,seq,checksum
	uint8_t pointer_ = 0;
	uint8_t received_seq_ = 0;
	uint8_t send_seq_ = 0;
};

#endif /* MYUART_HPP_ */
