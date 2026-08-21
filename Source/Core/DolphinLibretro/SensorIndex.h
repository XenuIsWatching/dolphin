// Copyright 2026 RetroXR
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

// The sensor sub-device index, as a core sees it.
//
// A COPY, kept byte-identical with the block in libretro-godot's
// SensorIndex.hpp. Externals/libretro-common tracks upstream and must not be
// patched for an extension that is not upstream yet; when it lands there, this
// file goes away and the definitions come from <libretro.h> like everything
// else.
//
// Diff the two with:
//   extract() { awk '/^\/\* -+$/{f=1} f{print} f&&/^\/\* --- end/{exit}' "$1"; }
//   diff <(extract A) <(extract B)

#include <libretro.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Sensor sub-device index (retroXR extension; no environment number needed).
 *
 * Deliberately one contiguous block rather than following libretro.h's usual
 * layout, so that this copy and Dolphin's can be diffed directly:
 *
 *   extract() { awk '/^\/\* -+$/{f=1} f{print} f&&/^\/\* --- end/{exit}' "$1"; }
 *   diff <(extract A) <(extract B)
 * ------------------------------------------------------------------------- */

/* Some peripherals carry more than one sensor on a single controller port. A
 * Wii Remote with a Nunchuk is one player with two accelerometers; add
 * MotionPlus and it is two accelerometers and a gyroscope. Both belong to the
 * same player, so both have to arrive on that player's port, but the sensor
 * interface addresses exactly one accelerometer, one gyroscope and one
 * illuminance per port:
 *
 *   bool  set_sensor_state(unsigned port, enum retro_sensor_action action, unsigned rate);
 *   float get_sensor_input(unsigned port, unsigned id);
 *
 * A second port is not the answer, because ports mean players. Cores already
 * map port N to one emulated controller, and spending two ports on one player
 * would break player counts, port arbitration and every frontend's controller
 * UI.
 *
 * So carry a sub-device index in the high bits of the existing id, the same way
 * RETRO_DEVICE_SUBCLASS already encodes a subclass into a device id. Index 0 is
 * the controller itself and encodes to exactly the values in use today
 * (0 << 8 | id == id), so every existing core and frontend is bit for bit
 * unchanged. Index 1 and up are sub-devices, in whatever order the device type
 * implies: for a Wii Remote with a Nunchuk, index 0 is the remote and index 1
 * is the Nunchuk.
 *
 * This needs no new environment call and no capability flag, because the
 * existing contract already specifies the fallback. A core asks for index 1;
 * a frontend that does not implement this returns false from set_sensor_state
 * ("the given sensor is not available on the provided port") and 0 from
 * get_sensor_input ("will return 0 for invalid arguments"), and the core then
 * does exactly what it does today. Frontends that gain support need no core
 * changes to be useful, and vice versa.
 *
 * retro_sensor_action is an enum rather than a plain unsigned, so the
 * enable/disable call casts at the call site. RETRO_SENSOR_DUMMY = INT_MAX
 * already pins the underlying type to int, so an encoded value is in range and
 * well defined.
 */
#define RETRO_SENSOR_INDEX_SHIFT   8
#define RETRO_SENSOR_INDEX_MASK    ((1u << RETRO_SENSOR_INDEX_SHIFT) - 1u)

/* The sub-device an encoded id or action names. 0 is the controller itself. */
#define RETRO_SENSOR_INDEX(id)     ((unsigned)(id) >> RETRO_SENSOR_INDEX_SHIFT)

/* The plain RETRO_SENSOR_* id or retro_sensor_action inside an encoded value. */
#define RETRO_SENSOR_BASE(id)      ((unsigned)(id) & RETRO_SENSOR_INDEX_MASK)

/* Address a sensor on sub-device `index`. RETRO_SENSOR_ID(0, x) == x. */
#define RETRO_SENSOR_ID(index, id) \
    ((unsigned)((((unsigned)(index)) << RETRO_SENSOR_INDEX_SHIFT) | ((unsigned)(id))))

/* --- end of the sensor sub-device index block ---------------------------- */

#ifdef __cplusplus
}
#endif
