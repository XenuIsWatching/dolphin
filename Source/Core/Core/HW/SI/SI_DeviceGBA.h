// Copyright 2008 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include <array>
#include <memory>

#include <SFML/Network.hpp>

#include "Common/CommonTypes.h"
#include "Core/HW/SI/SI_Device.h"

// GameBoy Advance "Link Cable"

// Declared in Core/Host.h, at global scope with the other Host_ hooks: it is a
// thing a FRONTEND provides, not a part of the serial interface.
class GBALinkTransport;

namespace SerialInterface
{
void GBAConnectionWaiter_Shutdown();

class GBASockServer
{
public:
  explicit GBASockServer(int device_number = 0);
  ~GBASockServer();

  bool Connect();
  bool IsConnected();
  void ClockSync(Core::System& system);
  // Let a Game Boy Advance on a frontend-hosted bus run up to `ticks` of this
  // GameCube's clock, and wait until it has. A no-op on sockets.
  void RunUpTo(u64 ticks);
  void Send(const u8* si_buffer);
  int Receive(u8* si_buffer, u8 bytes);
  void Flush();

private:
  void Disconnect();
  GBALinkTransport* Transport();

  std::unique_ptr<sf::TcpSocket> m_client;
  std::unique_ptr<sf::TcpSocket> m_clock_sync;

  // A bus the frontend hosts, when there is one. Where this exists the sockets
  // are never opened at all: it is the same conversation over something that
  // already knows what time it is on both ends.
  std::unique_ptr<GBALinkTransport> m_transport;
  bool m_transport_tried = false;
  int m_device_number = 0;

  u64 m_last_time_slice = 0;
  bool m_booted = false;
};

class CSIDevice_GBA final : public ISIDevice
{
public:
  CSIDevice_GBA(Core::System& system, SIDevices device, int device_number);

  int RunBuffer(u8* buffer, int request_length) override;
  int TransferInterval() override;
  DataResponse GetData(u32& hi, u32& low) override;
  void SendCommand(u32 command, u8 poll) override;

private:
  enum class NextAction
  {
    SendCommand,
    WaitTransferTime,
    ReceiveResponse
  };

  GBASockServer m_sock_server;
  NextAction m_next_action = NextAction::SendCommand;
  EBufferCommands m_last_cmd = EBufferCommands::CMD_STATUS;
  u64 m_timestamp_sent = 0;
};
}  // namespace SerialInterface
