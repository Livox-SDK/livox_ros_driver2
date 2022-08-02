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

#include "comm/comm_port.h"
#include <stdio.h>
#include <string.h>
#include <iostream>

#include "sdk_protocol.h"

namespace livox {
namespace direct {

CommPort::CommPort() {
  //protocol_ = new SdkProtocol(0x4c49, 0x564f580a);
  protocol_ = new SdkProtocol(0x4c49, 0x00);
}

CommPort::~CommPort() {
  if (protocol_) {
    delete protocol_;
  }
}

int32_t CommPort::Pack(uint8_t *o_buf, uint32_t o_buf_size, uint32_t *o_len, const CommPacket &i_packet) {
  return protocol_->Pack(o_buf, o_buf_size, o_len, i_packet);
}

bool CommPort::ParseCommStream(uint8_t *buf, uint32_t buf_size, CommPacket *o_pack) {
  if (!(protocol_->CheckPreamble(buf, buf_size))) {
    printf("CheckPreamble error.\n");
    return false;
  }
  return protocol_->ParsePacket(buf, buf_size, o_pack);
}

} // namespace direct
}  // namespace livox
