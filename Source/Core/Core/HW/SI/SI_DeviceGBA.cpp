// Copyright 2009 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "Core/Host.h"
#include "Core/HW/SI/SI_DeviceGBA.h"

#include <cstddef>
#include <cstring>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

#include "Common/CommonTypes.h"
#include "Common/Flag.h"
#include "Common/Logging/Log.h"
#include "Common/Thread.h"
#include "Core/CoreTiming.h"
#include "Core/HW/SI/SI_Device.h"
#include "Core/HW/SystemTimers.h"
#include "Core/System.h"

namespace SerialInterface
{
namespace
{
std::thread s_connection_thread;
std::queue<std::unique_ptr<sf::TcpSocket>> s_waiting_socks;
std::queue<std::unique_ptr<sf::TcpSocket>> s_waiting_clocks;
std::mutex s_cs_gba;
std::mutex s_cs_gba_clk;
int s_num_connected;
Common::Flag s_server_running;
}  // namespace

constexpr auto SEND_MAX_SIZE = 5, RECV_MAX_SIZE = 5;

// --- GameBoy Advance "Link Cable" ---

static void GBAConnectionWaiter()
{
  s_server_running.Set();

  Common::SetCurrentThreadName("GBA Connection Waiter");

  sf::TcpListener server;
  sf::TcpListener clock_server;

  // "dolphin gba"
  if (server.listen(0xd6ba) != sf::Socket::Status::Done)
    return;

  // "clock"
  if (clock_server.listen(0xc10c) != sf::Socket::Status::Done)
    return;

  server.setBlocking(false);
  clock_server.setBlocking(false);

  auto new_client = std::make_unique<sf::TcpSocket>();
  while (s_server_running.IsSet())
  {
    if (server.accept(*new_client) == sf::Socket::Status::Done)
    {
      std::lock_guard lk(s_cs_gba);
      s_waiting_socks.push(std::move(new_client));

      new_client = std::make_unique<sf::TcpSocket>();
    }
    if (clock_server.accept(*new_client) == sf::Socket::Status::Done)
    {
      std::lock_guard lk(s_cs_gba_clk);
      s_waiting_clocks.push(std::move(new_client));

      new_client = std::make_unique<sf::TcpSocket>();
    }

    Common::SleepCurrentThread(1);
  }
}

void GBAConnectionWaiter_Shutdown()
{
  s_server_running.Clear();
  if (s_connection_thread.joinable())
    s_connection_thread.join();
}

template <typename T>
static std::unique_ptr<T> MoveFromFront(std::queue<std::unique_ptr<T>>& ptrs)
{
  if (ptrs.empty())
    return nullptr;
  std::unique_ptr<T> ptr = std::move(ptrs.front());
  ptrs.pop();
  return ptr;
}

static std::unique_ptr<sf::TcpSocket> GetNextSock()
{
  std::lock_guard lk(s_cs_gba);
  return MoveFromFront(s_waiting_socks);
}

static std::unique_ptr<sf::TcpSocket> GetNextClock()
{
  std::lock_guard lk(s_cs_gba_clk);
  return MoveFromFront(s_waiting_clocks);
}

GBASockServer::GBASockServer(int device_number) : m_device_number(device_number)
{
  // The bus is asked for on first use, not here.
  //
  // A serial device is constructed while the machine is still being built, and
  // the frontend needs this GameCube's tick rate to join a bus with -- which the
  // system timers do not have yet at that moment. Asking too early got a rate of
  // zero at best and took the process down at worst.
  //
  // The connection waiter is still started here, because it has to be listening
  // before an mGBA dials in, and it costs one idle thread on a machine that
  // turns out to have a bus after all.
  if (!s_connection_thread.joinable())
    s_connection_thread = std::thread(GBAConnectionWaiter);

  s_num_connected = 0;
}

GBASockServer::~GBASockServer()
{
  Disconnect();
}

void GBASockServer::Disconnect()
{
  if (m_client)
  {
    s_num_connected--;
    m_client->disconnect();
    m_client = nullptr;
  }
  if (m_clock_sync)
  {
    m_clock_sync->disconnect();
    m_clock_sync = nullptr;
  }
  m_last_time_slice = 0;
  m_booted = false;
}

void GBASockServer::ClockSync(Core::System& system)
{
  // On a bus there is no time slice to send. The socket version sends one
  // because TCP has no shared clock and the Game Boy Advance would otherwise
  // have no idea how far to run; a frontend running both emulators has a clock
  // they are both on, and converts between a GameCube's tick rate and a GBA's
  // itself. Letting the other machine catch up is all that is left, and that is
  // what RunUpTo does, in the SI device where the wait belongs.
  if (Transport())
  {
    m_last_time_slice = system.GetCoreTiming().GetTicks();
    return;
  }

  if (!m_clock_sync)
    if (!(m_clock_sync = GetNextClock()))
      return;

  const auto& core_timing = system.GetCoreTiming();

  u32 time_slice = 0;

  if (m_last_time_slice == 0)
  {
    s_num_connected++;
    m_last_time_slice = core_timing.GetTicks();
    time_slice = (u32)(system.GetSystemTimers().GetTicksPerSecond() / 60);
  }
  else
  {
    time_slice = (u32)(core_timing.GetTicks() - m_last_time_slice);
  }

  time_slice = (u32)((u64)time_slice * 16777216 / system.GetSystemTimers().GetTicksPerSecond());
  m_last_time_slice = core_timing.GetTicks();
  char bytes[4] = {0, 0, 0, 0};
  bytes[0] = (time_slice >> 24) & 0xff;
  bytes[1] = (time_slice >> 16) & 0xff;
  bytes[2] = (time_slice >> 8) & 0xff;
  bytes[3] = time_slice & 0xff;

  const sf::Socket::Status status = m_clock_sync->send(bytes, 4);
  if (status == sf::Socket::Status::Disconnected)
  {
    m_clock_sync->disconnect();
    m_clock_sync = nullptr;
  }
}

bool GBASockServer::Connect()
{
  if (Transport())
    return m_transport->Connected();

  if (!IsConnected())
  {
    m_client = GetNextSock();
    if (m_client)
      m_client->setBlocking(false);
  }
  return IsConnected();
}

bool GBASockServer::IsConnected()
{
  if (Transport())
    return m_transport->Connected();
  return static_cast<bool>(m_client);
}

// The frontend's bus, asked for once and then remembered either way. Null means
// there is none and the sockets stand.
GBALinkTransport* GBASockServer::Transport()
{
  if (!m_transport_tried)
  {
    m_transport_tried = true;
    m_transport = Host_CreateGBALinkTransport(m_device_number);
  }
  return m_transport.get();
}

void GBASockServer::Send(const u8* si_buffer)
{
  if (!Connect())
    return;

  std::array<u8, SEND_MAX_SIZE> send_data;
  for (size_t i = 0; i < send_data.size(); i++)
    send_data[i] = si_buffer[i];

  const auto cmd = static_cast<EBufferCommands>(send_data[0]);
  // Only a write carries its four bytes; everything else is the command alone.
  const size_t len = (cmd == EBufferCommands::CMD_WRITE_GBA) ? send_data.size() : 1;

  if (Transport())
  {
    m_transport->Send(send_data.data(), len, m_last_time_slice);
    return;
  }

  sf::Socket::Status status = m_client->send(send_data.data(), len);

  if (status == sf::Socket::Status::Disconnected)
    Disconnect();
}

int GBASockServer::Receive(u8* si_buffer, u8 bytes)
{
  if (Transport())
    return m_transport->Receive(si_buffer, bytes);

  if (!m_client)
    return 0;

  if (m_booted)
  {
    sf::SocketSelector selector;
    selector.add(*m_client);
    (void)selector.wait(sf::milliseconds(1000));
  }

  size_t num_received = 0;
  std::array<u8, RECV_MAX_SIZE> recv_data;
  const sf::Socket::Status recv_stat = m_client->receive(recv_data.data(), bytes, num_received);
  if (recv_stat == sf::Socket::Status::Disconnected)
  {
    Disconnect();
    return 0;
  }

  if (recv_stat == sf::Socket::Status::NotReady || num_received == 0)
  {
    m_booted = false;
    return 0;
  }
  m_booted = true;

  for (size_t i = 0; i < recv_data.size(); i++)
    si_buffer[i] = recv_data[i];
  return static_cast<int>(std::min(num_received, recv_data.size()));
}

void GBASockServer::RunUpTo(u64 ticks)
{
  if (Transport())
    m_transport->RunUpTo(ticks);
}

void GBASockServer::Flush()
{
  // Nothing to flush on a bus: it delivers what was sent, in order, and there
  // are no replies left over from a poll that timed out.
  if (Transport())
    return;

  if (!m_client)
    return;

  size_t num_received = 1;
  u8 byte;
  while (num_received)
  {
    const sf::Socket::Status recv_stat = m_client->receive(&byte, 1, num_received);
    if (recv_stat != sf::Socket::Status::Done)
      break;
  }
}

CSIDevice_GBA::CSIDevice_GBA(Core::System& system, SIDevices device, int device_number)
    : ISIDevice(system, device, device_number), m_sock_server(device_number)
{
}

int CSIDevice_GBA::RunBuffer(u8* buffer, int request_length)
{
  switch (m_next_action)
  {
  case NextAction::SendCommand:
  {
    m_sock_server.ClockSync(m_system);
    if (m_sock_server.Connect())
    {
#ifdef _DEBUG
      NOTICE_LOG_FMT(SERIALINTERFACE, "{} cmd {:02x} [> {:02x}{:02x}{:02x}{:02x}]", m_device_number,
                     buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
#endif
      m_sock_server.Flush();  // Clear out any replies we might have timed out waiting for
      m_sock_server.Send(buffer);
    }
    else
    {
      return -1;
    }

    m_last_cmd = static_cast<EBufferCommands>(buffer[0]);
    m_timestamp_sent = m_system.GetCoreTiming().GetTicks();
    m_next_action = NextAction::WaitTransferTime;
    return 0;
  }

  case NextAction::WaitTransferTime:
  {
    const int transfer_time = SIDevice_GetGBATransferTime(m_system.GetSystemTimers(), m_last_cmd);
    const int elapsed_time =
        static_cast<int>(m_system.GetCoreTiming().GetTicks() - m_timestamp_sent);
    // Tell SI to ask again after TransferInterval() cycles
    if (transfer_time > elapsed_time)
      return 0;

    // On a bus the answer is not sitting in a socket waiting to be read: the
    // Game Boy Advance has to have RUN to the point where it gave one. Waiting
    // for it here is the whole of the clock the socket version has to fake with
    // a time slice, and it is why this can be exact where that one is a guess.
    m_sock_server.RunUpTo(m_timestamp_sent + static_cast<u64>(transfer_time));

    m_next_action = NextAction::ReceiveResponse;
    [[fallthrough]];
  }

  case NextAction::ReceiveResponse:
  {
    u8 bytes = 1;
    switch (m_last_cmd)
    {
    case EBufferCommands::CMD_RESET:
    case EBufferCommands::CMD_STATUS:
      bytes = 3;
      break;
    case EBufferCommands::CMD_READ_GBA:
      bytes = 5;
      break;
    default:
      break;
    }
    const int num_data_received = m_sock_server.Receive(buffer, bytes);

    m_next_action = NextAction::SendCommand;
    if (num_data_received == 0)
      return -1;
#ifdef _DEBUG
    const Common::Log::LogLevel log_level =
        (m_last_cmd == EBufferCommands::CMD_STATUS || m_last_cmd == EBufferCommands::CMD_RESET) ?
            Common::Log::LogLevel::LERROR :
            Common::Log::LogLevel::LWARNING;
    GENERIC_LOG_FMT(Common::Log::LogType::SERIALINTERFACE, log_level,
                    "{}                              [< {:02x}{:02x}{:02x}{:02x}{:02x}] ({})",
                    m_device_number, buffer[0], buffer[1], buffer[2], buffer[3], buffer[4],
                    num_data_received);
#endif
    return num_data_received;
  }
  }

  // This should never happen, but appease MSVC which thinks it might.
  ERROR_LOG_FMT(SERIALINTERFACE, "Unknown state {}\n", static_cast<int>(m_next_action));
  return 0;
}

int CSIDevice_GBA::TransferInterval()
{
  return SIDevice_GetGBATransferTime(m_system.GetSystemTimers(), m_last_cmd);
}

DataResponse CSIDevice_GBA::GetData(u32& hi, u32& low)
{
  return DataResponse::NoData;
}

void CSIDevice_GBA::SendCommand(u32 command, u8 poll)
{
}
}  // namespace SerialInterface
