//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "sdk_protocol.h"
#include "CRC/Crc.h"
#include "CRC/Crc_Cfg.h"

#include <stdio.h>
#include <string.h>
namespace livox {
namespace lidar {

const uint8_t kSdkProtocolSof = 0xAA;
const uint8_t kSdkVer = 0;
const uint32_t kSdkPacketCrcSize = 4;          // crc32
const uint32_t kSdkPacketPreambleCrcSize = 2;  // crc16

SdkProtocol::SdkProtocol(uint16_t seed16, uint32_t seed32) : crc16_(seed16), crc32_(seed32) {}

SdkProtocol::~SdkProtocol() {
}

int32_t SdkProtocol::Pack(uint8_t *o_buf, uint32_t o_buf_size, uint32_t *o_len, const CommPacket &i_packet) {
  if (kLidarSdk != i_packet.protocol) {
    return -1;
  }

  SdkPacket *sdk_packet = (SdkPacket *)o_buf;

  sdk_packet->sof = kSdkProtocolSof;
  sdk_packet->version = kSdkVer;
  sdk_packet->length = i_packet.data_len + GetPacketWrapperLen();
  if (sdk_packet->length > o_buf_size) {
    return -1;
  }

  sdk_packet->seq_num = i_packet.seq_num & 0xFFFF;
  sdk_packet->cmd_id = i_packet.cmd_id;
  sdk_packet->cmd_type = i_packet.cmd_type;
  sdk_packet->sender_type = i_packet.sender_type;

  sdk_packet->crc16_h = Crc_CalculateCRC16(o_buf, 18, 0, 1);

  if (i_packet.data_len == 0) {
    sdk_packet->crc32_d = 0;
  }  else {
    sdk_packet->crc32_d = Crc_CalculateCRC32(i_packet.data, i_packet.data_len, 0 , 1);
  }

  // if (sdk_packet->length - GetPacketWrapperLen() == 0) {

  //   sdk_packet->crc32_d = 0xffffffff;
  // } else {
  //   sdk_packet->crc32_d = Crc_CalculateCRC32(i_packet.data, i_packet.data_len, 0 , 1);
  // }

  // sdk_packet->crc16_h = crc16_.mcrf4xx_calc(o_buf, 18);
  // sdk_packet->crc32_d = crc32_.crc32_calc(i_packet.data, i_packet.data_len);

  memcpy(sdk_packet->data, i_packet.data, i_packet.data_len);

  *o_len = sdk_packet->length;

  return 0;
}

bool SdkProtocol::ParsePacket(uint8_t *i_buf, uint32_t buf_size, CommPacket *o_packet) {
  SdkPacket *sdk_packet = (SdkPacket *)i_buf;

  if (buf_size < GetPacketWrapperLen()) {
    return false;
  }

  memset((void *)o_packet, 0, sizeof(CommPacket));

  o_packet->protocol = kLidarSdk;
  o_packet->version = sdk_packet->version;
  o_packet->seq_num = sdk_packet->seq_num;
  o_packet->cmd_id = sdk_packet->cmd_id;
  o_packet->cmd_type = sdk_packet->cmd_type;
  o_packet->sender_type = sdk_packet->sender_type;
  o_packet->data = sdk_packet->data;
  o_packet->data_len = sdk_packet->length - GetPacketWrapperLen();
  return true;
}

uint32_t SdkProtocol::GetPreambleLen() {
  return sizeof(SdkPreamble);
}

uint32_t SdkProtocol::GetPacketWrapperLen() {
  return sizeof(SdkPacket) - 1;
}

uint32_t SdkProtocol::GetPacketLen(uint8_t *buf) {
  SdkPreamble *preamble = (SdkPreamble *)buf;
  return preamble->length;
}

bool SdkProtocol::CheckPreamble(uint8_t *buf, uint32_t buf_size) {
  if (buf_size < GetPreambleLen()) {
    return false;
  }

  SdkPacket *packet = (SdkPacket *)buf;
  if (packet->sof != kSdkProtocolSof) {
    return false;
  }

  if (packet->version != kSdkVer) {
    return false;
  }

  if (packet->length < GetPreambleLen()) {
    return false;
  }

  uint16_t crc16_h = Crc_CalculateCRC16(buf, 18, 0, 1);  
  if (packet->crc16_h != crc16_h) {
    return false;
  }


  uint32_t crc32_d = 0;
   if (packet->length - GetPacketWrapperLen() == 0) {
    crc32_d = 0;
  } else {
    crc32_d = Crc_CalculateCRC32(packet->data, packet->length - GetPacketWrapperLen(), 0 , 1);
  }
  
  if (packet->crc32_d != crc32_d) {
    return false;
  }
  return true;
}


} // namespace lidar
}  // namespace livox
