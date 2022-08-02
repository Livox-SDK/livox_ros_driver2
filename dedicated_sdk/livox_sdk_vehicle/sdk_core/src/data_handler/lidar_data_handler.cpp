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

#include "lidar_data_handler.h"
#include <mutex>
#include "base/network/network_util.h"
#include "base/logging.h"
#ifdef WIN32
#include <winsock2.h>
#include <ws2def.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <unistd.h>
#endif // WIN32

namespace livox {
namespace vehicle {

void LidarDataHandlerImpl::OnData(socket_t sock, void *client_data) {
  struct sockaddr addr;
  int addrlen = sizeof(addr);
  std::unique_ptr<char[]> &buf = data_buffers_;
  if (buf.get() == nullptr) {
    buf.reset(new char[kMaxBufferSize]);
  }

  int size = kMaxBufferSize;
  size = util::RecvFrom(sock, reinterpret_cast<char *>(buf.get()), kMaxBufferSize, 0, &addr, &addrlen);
  if (size <= 0) {
    return;
  }
  if (handler_) {
    handler_->OnDataCallback(sock, buf.get(), size);
  }
}

}
}  // namespace livox
