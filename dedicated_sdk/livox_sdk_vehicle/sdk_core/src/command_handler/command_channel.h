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

#ifndef LIVOX_COMMAND_CHANNEL_H_
#define LIVOX_COMMAND_CHANNEL_H_
#include <memory>
#include <map>
#include <list>
#include <string>
#include <algorithm>
#include "base/io_loop.h"
#include "comm/comm_port.h"
#include "base/io_loop.h"

namespace livox {
namespace vehicle {

typedef struct TagCommand {
  uint8_t slot;
  CommPacket packet;
  std::shared_ptr<CommandCallback> cb;
  uint32_t time_out;
  std::string remote_ip;
  TagCommand() : packet(), time_out(0) {}
  TagCommand(uint8_t _slot,
             uint8_t _cmd_type,
             uint8_t _cmd_id,
             uint16_t _seq_num,
             uint8_t *data,
             uint16_t length,
             uint32_t _time_out,
             std::string _remote_ip,
             const std::shared_ptr<CommandCallback> &_cb)
      : slot(_slot), packet(), cb(_cb) {
    packet.cmd_type = _cmd_type;
    packet.cmd_id = _cmd_id;
    packet.seq_num = _seq_num;
    packet.data = data;
    packet.data_len = length;
    time_out = _time_out;
    remote_ip = _remote_ip;
  }
} Command;

class CommandChannelDelegate {
 public:
  virtual void OnCommand(uint8_t slot, const Command &command) = 0;
};

class Protector {};

/**
 * CommandChannel implements the sending/receiving commands with a specific device.
 */
class CommandChannel : public IOLoop::IOLoopDelegate {
 public:
  typedef std::chrono::steady_clock::time_point TimePoint;
 
  CommandChannel(socket_t sock,
                 CommandChannelDelegate *cb);
  virtual ~CommandChannel() { Uninit(); }

  /** Uninitialize CommandChannel. */
  void Uninit();

  /**
   * Bind a CommandChannel with a IOLoop.
   * @param loop the IOLoop to bind.
   * @return true on successfully.
   */
  bool Bind(std::weak_ptr<IOLoop> loop);

  /**
   * Send a command asynchronously.
   * @param command the command to send.
   */
  void SendAsync(const Command &command);

  void OnData(socket_t, void *);
  void OnTimer(TimePoint now);

  static uint16_t GenerateSeq();

 private:
  void Send(const Command &cmd);
  void SendInternal(const Command &command);
  Command DeepCopy(const Command &cmd);
  void LidarSearchBroadcastRequest();

 private:
  socket_t sock_ = -1;
  socket_t log_sock_ = -1;
  std::weak_ptr<IOLoop> loop_;
  CommandChannelDelegate *callback_;
  std::map<uint16_t, std::pair<Command, TimePoint> > commands_;
  std::unique_ptr<CommPort> comm_port_;
  using SharedProtecotr = std::shared_ptr<Protector>;
  using WeakProtector = std::weak_ptr<Protector>;
  SharedProtecotr protector_ = std::make_shared<Protector>();

  TimePoint last_heartbeat_ = TimePoint();
};
}
}  // namespace livox
#endif  // LIVOX_COMMAND_CHANNEL_H_
