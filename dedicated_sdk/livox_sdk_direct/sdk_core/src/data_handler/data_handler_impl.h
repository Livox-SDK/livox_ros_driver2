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

#ifndef DATA_HANDLER_IMPL_H_
#define DATA_HANDLER_IMPL_H_

#include <memory>
#include <mutex>
#include <string>
#include <set>

#include "data_handler.h"

#include "base/io_thread.h"
#include "comm/define.h"

namespace livox {

namespace direct {

class DataHandler;
class DataHandlerImpl : public IOLoop::IOLoopDelegate {
 public:
  DataHandlerImpl(DataHandler* handler);
  virtual ~DataHandlerImpl();
  bool Init(std::shared_ptr<DirectHostIpInfo> direct_host_ipinfo_ptr,
    std::shared_ptr<std::vector<DirectLidarInfo>> direct_lidars_info_ptr);
  bool Start();
  void OnData(socket_t sock, void *client_data);
  void Destory();
 private:
  bool CreateIOThread();
  bool CreateDataChannel();
  bool CreateSocketAndAddDelegate(const std::string& host_ip, const uint16_t port);
 private:
  static const size_t kMaxBufferSize = 8192;
  DataHandler *handler_;
  std::shared_ptr<IOThread> thread_;
  std::shared_ptr<DirectHostIpInfo> direct_host_ipinfo_ptr_;
  std::shared_ptr<std::vector<DirectLidarInfo>> direct_lidars_info_ptr_;
  std::map<std::string, socket_t> data_channel_;

  std::unique_ptr<char[]> data_buffers_;
};

} // namespace direct
}  // namespace livox

#endif  // LIVOX_LIDAR_DATA_HANDLER_H_

