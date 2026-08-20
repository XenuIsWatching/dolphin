// Copyright 2026 RetroXR
// SPDX-License-Identifier: GPL-2.0-or-later

// A GameCube's serial port reaching a Game Boy Advance the frontend is also
// running, over a bus the frontend hosts.
//
// Dolphin has always been able to talk to an external mGBA, over a pair of TCP
// sockets: one carrying JOY bus commands and replies, the other carrying a time
// slice, because TCP has no shared clock and the Game Boy Advance would
// otherwise have no idea how far to run. A frontend running both emulators has
// a clock they are both on, so the second socket has nothing to carry and the
// first becomes a queue rather than a round trip.
//
// The protocol on the wire is unchanged. What the GameCube sends is exactly what
// it sent down the socket -- a command byte, and four more for a write -- and
// what comes back is exactly what mGBA's JOY bus code produces. Only the
// carriage is different.

#include <cstring>
#include <memory>

#include "Common/CommonTypes.h"
#include "Common/Logging/Log.h"
#include "Core/Host.h"
#include "Core/System.h"
#include "Core/HW/SystemTimers.h"

#include "DolphinLibretro/Common/Globals.h"
#include "DolphinLibretro/LinkInterface.h"
#include "DolphinLibretro/Log.h"

namespace Libretro
{
namespace
{
// Named the same at both ends, which is what lets the frontend refuse to join a
// GameCube lead to a Game Boy link lead. They are different cables and different
// conversations, and a bus that let one stand in for the other would produce a
// machine that answers in a language nobody asked a question in.
constexpr const char JOYLINK_PROTOCOL[] = "gba-joy-1";

// How far ahead a GameCube promises not to originate anything.
//
// This is the commit horizon, and on this bus it is a tuning knob rather than a
// correctness one: it only has to be small against the interval between serial
// polls, which is about a millisecond of GameCube time. A thousandth of that is
// far finer than anything the JOY bus does and still coarse enough that the two
// machines are not rendezvousing constantly.
constexpr u64 HORIZON_DIVISOR = 1000000;

const retro_link_interface* g_link = nullptr;
bool g_link_probed = false;

const retro_link_interface* GetLinkInterface()
{
  if (g_link_probed)
    return g_link;
  g_link_probed = true;

  if (!environ_cb)
    return nullptr;

  // The frontend FILLS THE STRUCT, it does not hand back a pointer to one. The
  // address of a pointer variable is the wrong thing to pass and does not fail
  // politely: six function pointers land on eight bytes of stack and the process
  // dies somewhere else entirely, which is exactly how this was found.
  //
  // Experimental number first, then the plain one, so a core built today keeps
  // working against a frontend that later adopts the unflagged value.
  static retro_link_interface stored{};
  retro_link_interface probe{};
  if (environ_cb(RETRO_ENVIRONMENT_GET_LINK_INTERFACE, &probe) ||
      environ_cb(RETRO_ENVIRONMENT_GET_LINK_INTERFACE_FINAL, &probe))
  {
    if (probe.attach && probe.detach && probe.peers && probe.send && probe.recv && probe.advance)
    {
      stored = probe;
      g_link = &stored;
    }
  }

  if (g_link)
    INFO_LOG_FMT(SERIALINTERFACE, "GBA link: frontend hosts a link bus");
  return g_link;
}

class Transport final : public GBALinkTransport
{
public:
  Transport(const retro_link_interface* link, retro_link_handle_t handle, u64 ticks_per_second)
      : m_link(link), m_handle(handle), m_horizon(ticks_per_second / HORIZON_DIVISOR + 1)
  {
  }

  ~Transport() override
  {
    if (m_link && m_handle)
      m_link->detach(m_handle);
  }

  bool Connected() override
  {
    unsigned count = 0;
    return m_link->peers(m_handle, &count) >= 0 && count >= 2;
  }

  void RunUpTo(u64 ticks) override
  {
    // Publishes where this GameCube is and how far it promises not to originate
    // anything, then blocks until the Game Boy Advance has run to the tick the
    // reply is due at. The frontend converts between the two machines' clock
    // rates; neither end ever sees the other's units.
    m_position = ticks;
    m_link->advance(m_handle, ticks, ticks + m_horizon, ticks);
  }

  void Send(const u8* data, size_t len, u64 ticks) override
  {
    u8 msg[MSG_SIZE];
    std::memset(msg, 0, sizeof(msg));
    msg[0] = NL_JOY_CMD;
    msg[4] = data[0];
    if (len > 1)
      std::memcpy(&msg[5], &data[1], std::min<size_t>(len - 1, 4));

    m_position = ticks;
    // Stamped a horizon out, which is the promise made in RunUpTo: nothing this
    // machine originates lands before that tick, so the other end can run up to
    // it without asking.
    m_link->send(m_handle, ticks + m_horizon, RETRO_LINK_BROADCAST, msg, sizeof(msg));
  }

  int Receive(u8* data, u8 bytes) override
  {
    u8 msg[MSG_SIZE];
    u64 tick = 0;
    unsigned from = 0;
    size_t len = sizeof(msg);

    while (m_link->recv(m_handle, &tick, &from, msg, &len))
    {
      if (len == MSG_SIZE && msg[0] == NL_JOY_REPLY)
      {
        const u8 got = std::min<u8>(msg[4], 5);
        const u8 want = std::min<u8>(bytes, got);
        std::memcpy(data, &msg[5], want);
        return want;
      }
      len = sizeof(msg);
    }
    return 0;
  }

private:
  // Message kinds, matching the driver at the other end byte for byte. Packed by
  // hand rather than shipped as a struct: the two ends are separate programs and
  // a shared layout would be an assumption nobody remembers having made.
  static constexpr size_t MSG_SIZE = 12;
  static constexpr u8 NL_JOY_CMD = 5;
  static constexpr u8 NL_JOY_REPLY = 6;

  const retro_link_interface* m_link;
  retro_link_handle_t m_handle;
  u64 m_horizon;
  u64 m_position = 0;
};
}  // namespace
}  // namespace Libretro

std::unique_ptr<GBALinkTransport> Host_CreateGBALinkTransport(int device_number)
{
  const retro_link_interface* link = Libretro::GetLinkInterface();
  if (!link)
    return nullptr;

  // Attached with this GameCube's OWN tick rate, which is the whole point of the
  // rate being declared at attach: a GameCube counts at about 729 million ticks
  // a second and a Game Boy Advance at 16.7 million, and the frontend converts
  // between them exactly rather than either end guessing.
  const u64 ticks_per_second =
      Core::System::GetInstance().GetSystemTimers().GetTicksPerSecond();
  if (!ticks_per_second)
    return nullptr;

  retro_link_handle_t handle = link->attach(static_cast<unsigned>(device_number),
                                            Libretro::JOYLINK_PROTOCOL, ticks_per_second);
  if (!handle)
    return nullptr;

  INFO_LOG_FMT(SERIALINTERFACE, "GBA link: port {} on the frontend's bus at {} Hz",
               device_number, ticks_per_second);
  return std::make_unique<Libretro::Transport>(link, handle, ticks_per_second);
}
