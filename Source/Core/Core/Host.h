// Copyright 2008 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include <memory>
#include <span>
#include <string>
#include <vector>

#include "Common/CommonTypes.h"

// Host - defines an interface for the emulator core to communicate back to the
// OS-specific layer
//
// The emulator core is abstracted from the OS using 2 interfaces:
// Common and Host.
//
// Common simply provides OS-neutral implementations of things like threads, mutexes,
// INI file manipulation, memory mapping, etc.
//
// Host is an abstract interface for communicating things back to the host. The emulator
// core is treated as a library, not as a main program, because it is far easier to
// write GUI interfaces that control things than to squash GUI into some model that wasn't
// designed for it.
//
// The host can be just a command line app that opens a window, or a full blown debugger
// interface.

namespace HW::GBA
{
class Core;
}  // namespace HW::GBA

class GBAHostInterface
{
public:
  virtual ~GBAHostInterface() = default;
  virtual void GameChanged() = 0;
  virtual void FrameEnded(std::span<const u32> video_buffer) = 0;
};

// A way for a GameCube's serial port to reach a Game Boy Advance that is NOT
// inside this process's emulation -- a separate emulator the frontend is also
// running, sharing a bus the frontend hosts.
//
// Dolphin has always been able to do this over a pair of TCP sockets, which is
// what GBASockServer speaks, and a socket is a poor way to carry a link that
// polls a thousand times a second: there is no shared clock, so one end has to
// send the other a time slice, and the round trip costs more than the transfer
// it carries. A frontend running both emulators can do better, because it has a
// clock both of them are on.
//
// Returns nullptr in frontends that have no such bus, which is all of them
// except libretro; those keep the sockets.
class GBALinkTransport
{
public:
  virtual ~GBALinkTransport() = default;

  // Whether anything is on the other end. False makes the device behave exactly
  // as an unplugged socket does.
  virtual bool Connected() = 0;

  // Let the Game Boy Advance run up to `ticks`, in this GameCube's units, and
  // wait until it has. This is the shared clock: it replaces the time slice the
  // socket version has to send because TCP has none.
  virtual void RunUpTo(u64 ticks) = 0;

  virtual void Send(const u8* data, size_t len, u64 ticks) = 0;
  virtual int Receive(u8* data, u8 bytes) = 0;
};

std::unique_ptr<GBALinkTransport> Host_CreateGBALinkTransport(int device_number);

enum class HostMessageID
{
  // Begin at 10 in case there is already messages with wParam = 0, 1, 2 and so on
  WMUserStop = 10,
  WMUserJobDispatch,
};

std::vector<std::string> Host_GetPreferredLocales();
bool Host_UIBlocksControllerState();
bool Host_RendererHasFocus();
bool Host_RendererHasFullFocus();
bool Host_RendererIsFullscreen();
bool Host_TASInputHasFocus();

void Host_Message(HostMessageID id);
void Host_PPCSymbolsChanged();
void Host_PPCBreakpointsChanged();
void Host_RequestRenderWindowSize(int width, int height);
void Host_UpdateDisasmDialog();
void Host_JitCacheInvalidation();
void Host_JitProfileDataWiped();
void Host_UpdateTitle(const std::string& title);
void Host_YieldToUI();
void Host_TitleChanged();

void Host_UpdateDiscordClientID(const std::string& client_id = {});
bool Host_UpdateDiscordPresenceRaw(const std::string& details = {}, const std::string& state = {},
                                   const std::string& large_image_key = {},
                                   const std::string& large_image_text = {},
                                   const std::string& small_image_key = {},
                                   const std::string& small_image_text = {},
                                   const int64_t start_timestamp = 0,
                                   const int64_t end_timestamp = 0, const int party_size = 0,
                                   const int party_max = 0);

std::unique_ptr<GBAHostInterface> Host_CreateGBAHost(std::weak_ptr<HW::GBA::Core> core);
