// Copyright 2026 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

namespace Libretro::Memcard
{
// Seat the GameCube memory card slots from the frontend's options, before
// HW::Init runs. No ChangeDevice here: the expansion interface has not been
// constructed yet and reads MAIN_SLOT_A/B itself when it is.
//
// Must run BEFORE Libretro::Input::Init, which claims slot B for the GameCube
// microphone. A card in slot B wins, because naming a specific file is explicit
// and the microphone option defaults off.
void ApplyCold();

// Re-seat any slot whose option changed. Called every retro_run, after the
// emulation thread is up.
//
// Compares the RESOLVED seat itself rather than asking Options::IsUpdated,
// which is not a change detector: CheckForUpdatedVariables marks every key
// dirty whenever the frontend reports any change at all, so trusting it would
// eject both cards each time the player nudges EFB scale.
void CheckForUpdates();

// Forget the applied state and clear the path overrides. The core stays loaded
// between games, so a stale override would silently seat the last game's card.
void Shutdown();
}  // namespace Libretro::Memcard
