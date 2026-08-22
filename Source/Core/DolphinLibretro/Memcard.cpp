// Copyright 2026 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "DolphinLibretro/Memcard.h"

#include <array>
#include <string>

#include <fmt/format.h>

#include "Common/CommonTypes.h"
#include "Common/Config/Config.h"
#include "Common/EnumMap.h"
#include "Common/FileUtil.h"
#include "Common/Logging/Log.h"
#include "Core/Config/MainSettings.h"
#include "Core/Core.h"
#include "Core/CoreTiming.h"
#include "Core/HW/EXI/EXI.h"
#include "Core/HW/EXI/EXI_Device.h"
#include "Core/HW/EXI/EXI_DeviceMemoryCard.h"
#include "Core/HW/GCMemcard/GCMemcard.h"
#include "Core/System.h"
#include "DolphinLibretro/Common/Options.h"
#include "DolphinLibretro/Common/VFile.h"
#include "VideoCommon/OnScreenDisplay.h"

namespace Libretro::Memcard
{
using ExpansionInterface::EXIDeviceType;
using ExpansionInterface::MAX_MEMCARD_SLOT;
using ExpansionInterface::Slot;

namespace
{
struct Seat
{
  EXIDeviceType device = EXIDeviceType::None;
  std::string path;

  bool operator==(const Seat& o) const { return device == o.device && path == o.path; }
};

// What is actually installed, so CheckForUpdates can tell a real change from
// the frontend merely saying "something changed".
Common::EnumMap<Seat, MAX_MEMCARD_SLOT> s_applied;

const char* OptionKey(Slot slot)
{
  return slot == Slot::A ? Libretro::Options::sysconf_gc::MEMCARD_A_PATH
                         : Libretro::Options::sysconf_gc::MEMCARD_B_PATH;
}

char SlotName(Slot slot)
{
  return slot == Slot::A ? 'A' : 'B';
}

void Refuse(Slot slot, const std::string& path, const std::string& why)
{
  const std::string msg =
      fmt::format("Memory Card {}: {} ({})", SlotName(slot), why, path);
  ERROR_LOG_FMT(EXPANSIONINTERFACE, "{}", msg);
  OSD::AddMessage(msg, OSD::Duration::VERY_LONG, OSD::Color::RED);
}

// Is this the byte size of a real GameCube card? Sizes are fixed by the
// hardware, so anything else is a truncated, padded or simply wrong file.
//
// ::Memcard, not Memcard: this namespace is Libretro::Memcard, which shadows
// the GCMemcard one that owns these constants.
bool IsValidCardSize(u64 size)
{
  for (const u16 mbit : {4, 8, 16, 32, 64, 128})
  {
    if (size == static_cast<u64>(mbit) * ::Memcard::MBIT_TO_BLOCKS * ::Memcard::BLOCK_SIZE)
      return true;
  }
  return false;
}

// Dolphin has no File::IsAbsolutePath, and a relative path here would resolve
// against a working directory the frontend does not control.
bool IsAbsolute(const std::string& path)
{
#ifdef _WIN32
  if (path.size() >= 2 && path[1] == ':')
    return true;
  return path.size() >= 2 && (path[0] == '\\' || path[0] == '/') &&
         (path[1] == '\\' || path[1] == '/');
#else
  return !path.empty() && path[0] == '/';
#endif
}

// What the option says this slot should hold.
//
// A path that cannot be honoured seats NOTHING rather than falling back. The
// alternative is that MemoryCard's constructor creates and formats a blank card
// at a path the frontend never asked to be written -- and FlushThread's exit
// branch then writes it out whether it is dirty or not. To the player that is
// indistinguishable from their saves having been wiped, so an unusable card is
// reported as no card and the game says so.
Seat Resolve(Slot slot, const Seat& other)
{
  const std::string raw =
      Libretro::Options::GetCached<std::string>(OptionKey(slot),
                                                slot == Slot::A ? "gci" : "none");
  if (raw.empty() || raw == "none")
    return {EXIDeviceType::None, {}};
  if (raw == "gci")
    return {EXIDeviceType::MemoryCardFolder, {}};

  const std::string path = Libretro::VFile::NormalizePath(raw);
  if (!IsAbsolute(path))
  {
    Refuse(slot, path, "not an absolute path, so no card was seated");
    return {EXIDeviceType::None, {}};
  }
  if (!File::Exists(path))
  {
    Refuse(slot, path, "no such file, so no card was seated");
    return {EXIDeviceType::None, {}};
  }
  if (!IsValidCardSize(File::GetSize(path)))
  {
    Refuse(slot, path, "not the size of a memory card, so no card was seated");
    return {EXIDeviceType::None, {}};
  }
  // Two MemoryCard objects over one file each run their own flush thread, and
  // the later flush wins whole-card. Dolphin's own GUI refuses this pairing for
  // the same reason.
  if (other.device == EXIDeviceType::MemoryCard && other.path == path)
  {
    Refuse(slot, path, "already in the other slot, so no card was seated");
    return {EXIDeviceType::None, {}};
  }
  return {EXIDeviceType::MemoryCard, path};
}

void Install(Slot slot, const Seat& seat)
{
  ExpansionInterface::SetExactMemcardPath(slot, seat.path);
  // SetCurrent, not SetBase: CurrentRun is the top config layer, so a per-game
  // INI carrying [Core] SlotA cannot overrule the card the player is holding.
  Config::SetCurrent(Config::GetInfoForEXIDevice(slot), seat.device);
  s_applied[slot] = seat;
}
}  // namespace

void ApplyCold()
{
  Seat a = Resolve(Slot::A, {});
  Install(Slot::A, a);
  Install(Slot::B, Resolve(Slot::B, a));
}

void CheckForUpdates()
{
  auto& system = Core::System::GetInstance();
  for (const Slot slot : ExpansionInterface::MEMCARD_SLOTS)
  {
    const Slot other = slot == Slot::A ? Slot::B : Slot::A;
    const Seat want = Resolve(slot, s_applied[other]);
    if (want == s_applied[slot])
      continue;

    // ChangeDevice schedules None at cycle 0 and the replacement one emulated
    // second later, which is a genuine eject and re-insert: the outgoing
    // CEXIMemoryCard is destroyed, and ~MemoryCard joins its flush thread, so
    // pulling a card IS its durability point.
    //
    // On the CPU thread, and never bare from here. ChangeDevice defaults to
    // FromThread::NON_CPU, and in single-core mode the libretro thread IS the
    // CPU thread -- which trips CoreTiming's from_cpu_thread assert in a debug
    // build and races m_event_queue against m_ts_queue in a release one.
    // retro_set_eject_state does this same dance for disc swap.
    Core::RunOnCPUThread(system, [&system, slot, want] {
      ExpansionInterface::SetExactMemcardPath(slot, want.path);
      Config::SetCurrent(Config::GetInfoForEXIDevice(slot), want.device);
      system.GetExpansionInterface().ChangeDevice(slot, want.device,
                                                 CoreTiming::FromThread::CPU);
    }, true);  // wait_for_completion = true

    s_applied[slot] = want;
    INFO_LOG_FMT(EXPANSIONINTERFACE, "Memory Card {}: now {}", SlotName(slot),
                 want.path.empty() ? "empty" : want.path);
  }
}

void Shutdown()
{
  for (const Slot slot : ExpansionInterface::MEMCARD_SLOTS)
  {
    ExpansionInterface::SetExactMemcardPath(slot, {});
    s_applied[slot] = Seat{};
  }
}
}  // namespace Libretro::Memcard
