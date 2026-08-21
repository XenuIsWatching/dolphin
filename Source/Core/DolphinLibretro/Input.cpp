#include <algorithm>
#include <array>
#include <cassert>
#include <libretro.h>

#include "Common/Common.h"
#include "Common/CommonPaths.h"
#include "Common/CommonTypes.h"
#include "Common/FileUtil.h"
#include "Common/IniFile.h"
#include "Common/Logging/Log.h"
#include "Core/Config/MainSettings.h"
#include "Core/Config/WiimoteSettings.h"
#include "Core/ConfigManager.h"
#include "Core/FreeLookManager.h"
#include "Core/HW/EXI/EXI.h"
#include "Core/HW/EXI/EXI_Device.h"
#include "Core/HW/EXI/EXI_DeviceMic.h"
#include "Core/HW/GBAPad.h"
#include "Core/HW/GCKeyboard.h"
#include "Core/HW/GCPad.h"
#include "Core/HW/GCPadEmu.h"
#include "Core/HW/Wiimote.h"
#include "Core/HW/WiimoteEmu/Extension/Classic.h"
#include "Core/HW/WiimoteEmu/Extension/Nunchuk.h"
#include "Core/HW/WiimoteEmu/WiimoteEmu.h"
#include "Core/HW/WiimoteReal/WiimoteReal.h"
#include "Core/HW/SI/SI.h"
#include "Core/HW/SI/SI_Device.h"
#include "Core/IOS/USB/Bluetooth/BTReal.h"
#include "Core/Host.h"
#include "Core/System.h"
#include "Core/WiiUtils.h"
#include "DolphinLibretro/Input.h"
#include "DolphinLibretro/Common/Globals.h"
#include "DolphinLibretro/Common/Options.h"
#include "InputCommon/ControlReference/ControlReference.h"
#include "InputCommon/ControlReference/ExpressionParser.h"
#include "InputCommon/ControllerEmu/Control/Control.h"
#include "InputCommon/ControllerEmu/ControlGroup/Attachments.h"
#include "InputCommon/ControllerEmu/ControlGroup/IRPassthrough.h"
#include "InputCommon/ControllerEmu/Setting/NumericSetting.h"
#include "InputCommon/ControllerInterface/ControllerInterface.h"
#include "InputCommon/GCAdapter.h"
#include "InputCommon/GCPadStatus.h"
#include "InputCommon/InputConfig.h"
#include "InputCommon/InputProfile.h"

#define RETRO_DEVICE_WIIMOTE RETRO_DEVICE_JOYPAD
#define RETRO_DEVICE_WIIMOTE_SW ((2 << 8) | RETRO_DEVICE_JOYPAD)
#define RETRO_DEVICE_WIIMOTE_NC ((3 << 8) | RETRO_DEVICE_JOYPAD)
#define RETRO_DEVICE_WIIMOTE_CC ((4 << 8) | RETRO_DEVICE_JOYPAD)
#define RETRO_DEVICE_WIIMOTE_CC_PRO ((5 << 8) | RETRO_DEVICE_JOYPAD)
#define RETRO_DEVICE_GC_ON_WII ((6 << 8) | RETRO_DEVICE_JOYPAD)
#define RETRO_DEVICE_REAL_WIIMOTE ((6 << 8) | RETRO_DEVICE_NONE)
// A Game Boy Advance on the end of a GameCube lead: a machine of its own, not a
// controller, with its own screen and its own emulator running elsewhere in this
// process. Four Swords Adventures and its like put a whole player on one.
//
// A DEVICE TYPE rather than a core option, because that is exactly what libretro
// already means by "what is plugged into port N": it is per-port, it can change
// while the machine runs, and the room sets it by seating a cable rather than by
// opening a menu. A core option would have been a second way of saying the same
// thing, and one that could disagree with the first.
#define RETRO_DEVICE_GBA_LINK ((7 << 8) | RETRO_DEVICE_NONE)
// MotionPlus is a dongle in the expansion port, not a mode of the remote, so it
// gets device ids of its own rather than a core option: it is per-PORT hardware,
// and one player having it fitted while another does not is the ordinary case.
//
// It doubles the list rather than adding one entry, because the dongle PASSES THE
// PORT THROUGH: every extension still plugs in, into the dongle rather than into
// the remote. So each remote above has a with-MotionPlus twin, sideways included
// (that one is a way of holding the thing, not something plugged into it).
#define RETRO_DEVICE_WIIMOTE_MP ((7 << 8) | RETRO_DEVICE_JOYPAD)
#define RETRO_DEVICE_WIIMOTE_MP_SW ((8 << 8) | RETRO_DEVICE_JOYPAD)
#define RETRO_DEVICE_WIIMOTE_MP_NC ((9 << 8) | RETRO_DEVICE_JOYPAD)
#define RETRO_DEVICE_WIIMOTE_MP_CC ((10 << 8) | RETRO_DEVICE_JOYPAD)
#define RETRO_DEVICE_WIIMOTE_MP_CC_PRO ((11 << 8) | RETRO_DEVICE_JOYPAD)

/// The same remote with the dongle taken back off, or the id unchanged when it
/// never had one.
///
/// MotionPlus changes nothing about the buttons, the extension or the IR. It
/// sits between the remote and whatever else is plugged in and reports rotation.
/// So everything downstream stays written against the five original ids, and this
/// collapses the dongle out first rather than every branch having to name ten.
static inline unsigned wiimote_base_device(unsigned device)
{
  switch (device)
  {
  case RETRO_DEVICE_WIIMOTE_MP:
    return RETRO_DEVICE_WIIMOTE;
  case RETRO_DEVICE_WIIMOTE_MP_SW:
    return RETRO_DEVICE_WIIMOTE_SW;
  case RETRO_DEVICE_WIIMOTE_MP_NC:
    return RETRO_DEVICE_WIIMOTE_NC;
  case RETRO_DEVICE_WIIMOTE_MP_CC:
    return RETRO_DEVICE_WIIMOTE_CC;
  case RETRO_DEVICE_WIIMOTE_MP_CC_PRO:
    return RETRO_DEVICE_WIIMOTE_CC_PRO;
  default:
    return device;
  }
}

/// True when this device id carries a MotionPlus dongle. Defined as "the id
/// changes when the dongle is removed", so the two can never disagree about
/// which ids are twins.
static inline bool wiimote_has_motion_plus(unsigned device)
{
  return wiimote_base_device(device) != device;
}

typedef enum {
    SENSOR_ACCELEROMETER = 0,
    SENSOR_GYRO,
    SENSOR_COUNT
} sensor_type_t;

/* Which sub-device of a Wii Remote's port the Nunchuk's accelerometer is. The
 * remote itself is 0. A real Nunchuk has no gyroscope, so it is asked for an
 * accelerometer and nothing else. */
#define NUNCHUK_SENSOR_INDEX 1

void retro_set_controller_port_device_gc(unsigned port, unsigned device);
void retro_set_controller_port_device_wii(unsigned port, unsigned device);

namespace Libretro
{
extern retro_environment_t environ_cb;
namespace Input
{
static retro_input_poll_t poll_cb = nullptr;
static retro_input_state_t input_cb = nullptr;
static struct retro_rumble_interface rumble{};
static unsigned input_types[8];
static bool g_init_wiimotes = false;
static bool s_sensor_init_pending = false;
static bool sensor_enabled[NUM_CONTROLLERS_FOR_SENSORS][NUM_SENSOR_SUB_DEVICES][SENSOR_COUNT] = {};
static int port_max;
double g_accel_pos[NUM_CONTROLLERS_FOR_SENSORS][NUM_SENSOR_SUB_DEVICES][3] = {}; // x, y, z
double g_accel_neg[NUM_CONTROLLERS_FOR_SENSORS][NUM_SENSOR_SUB_DEVICES][3] = {}; // x, y, z
double g_gyro_pos[NUM_CONTROLLERS_FOR_SENSORS][NUM_SENSOR_SUB_DEVICES][3] = {};  // x, y, z
double g_gyro_neg[NUM_CONTROLLERS_FOR_SENSORS][NUM_SENSOR_SUB_DEVICES][3] = {};  // x, y, z

static struct retro_input_descriptor descGC[] = {
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_LEFT, "Left"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_UP, "Up"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_DOWN, "Down"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_RIGHT, "Right"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_B, "B"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_A, "A"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_X, "X"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_Y, "Y"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_L2, "L"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_R2, "R"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_L3, "L-Analog"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_R3, "R-Analog"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_R, "Z"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_L, "Triforce - Test"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_SELECT, "Triforce - Coin"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_START, "Start"},
    {0, RETRO_DEVICE_ANALOG, RETRO_DEVICE_INDEX_ANALOG_LEFT, RETRO_DEVICE_ID_ANALOG_X,
    "Control Stick X"},
    {0, RETRO_DEVICE_ANALOG, RETRO_DEVICE_INDEX_ANALOG_LEFT, RETRO_DEVICE_ID_ANALOG_Y,
    "Control Stick Y"},
    {0, RETRO_DEVICE_ANALOG, RETRO_DEVICE_INDEX_ANALOG_RIGHT, RETRO_DEVICE_ID_ANALOG_X,
    "C Buttons X"},
    {0, RETRO_DEVICE_ANALOG, RETRO_DEVICE_INDEX_ANALOG_RIGHT, RETRO_DEVICE_ID_ANALOG_Y,
    "C Buttons Y"},
    {0},
};

static struct retro_input_descriptor descWiimoteCC[] = {
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_LEFT, "Left"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_UP, "Up"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_DOWN, "Down"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_RIGHT, "Right"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_B, "B"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_A, "A"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_X, "X"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_Y, "Y"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_L2, "L"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_R2, "R"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_L, "ZL"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_R, "ZR"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_R3, "Home"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_START, "+"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_SELECT, "-"},
    {0, RETRO_DEVICE_ANALOG, RETRO_DEVICE_INDEX_ANALOG_LEFT, RETRO_DEVICE_ID_ANALOG_X,
    "Left Stick X"},
    {0, RETRO_DEVICE_ANALOG, RETRO_DEVICE_INDEX_ANALOG_LEFT, RETRO_DEVICE_ID_ANALOG_Y,
    "Left Stick Y"},
    {0, RETRO_DEVICE_ANALOG, RETRO_DEVICE_INDEX_ANALOG_RIGHT, RETRO_DEVICE_ID_ANALOG_X,
    "Right Stick X"},
    {0, RETRO_DEVICE_ANALOG, RETRO_DEVICE_INDEX_ANALOG_RIGHT, RETRO_DEVICE_ID_ANALOG_Y,
    "Right Stick Y"},
    {0},
};

static struct retro_input_descriptor descWiimoteCCPro[] = {
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_LEFT, "Left"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_UP, "Up"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_DOWN, "Down"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_RIGHT, "Right"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_B, "B"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_A, "A"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_X, "X"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_Y, "Y"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_L2, "ZL"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_R2, "ZR"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_L, "L"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_R, "R"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_R3, "Home"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_START, "+"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_SELECT, "-"},
    {0, RETRO_DEVICE_ANALOG, RETRO_DEVICE_INDEX_ANALOG_LEFT, RETRO_DEVICE_ID_ANALOG_X,
    "Left Stick X"},
    {0, RETRO_DEVICE_ANALOG, RETRO_DEVICE_INDEX_ANALOG_LEFT, RETRO_DEVICE_ID_ANALOG_Y,
    "Left Stick Y"},
    {0, RETRO_DEVICE_ANALOG, RETRO_DEVICE_INDEX_ANALOG_RIGHT, RETRO_DEVICE_ID_ANALOG_X,
    "Right Stick X"},
    {0, RETRO_DEVICE_ANALOG, RETRO_DEVICE_INDEX_ANALOG_RIGHT, RETRO_DEVICE_ID_ANALOG_Y,
    "Right Stick Y"},
    {0},
};

static struct retro_input_descriptor descWiimote[] = {
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_LEFT, "Left"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_UP, "Up"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_DOWN, "Down"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_RIGHT, "Right"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_B, "B"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_A, "A"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_X, "1"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_Y, "2"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_R2, "Shake Wiimote"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_START, "+"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_SELECT, "-"},
    //{0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_L3, "Sideways Toggle"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_R3, "Home"},
    {0, RETRO_DEVICE_ANALOG, RETRO_DEVICE_INDEX_ANALOG_LEFT, RETRO_DEVICE_ID_ANALOG_X,
    "Tilt Left/Right"},
    {0, RETRO_DEVICE_ANALOG, RETRO_DEVICE_INDEX_ANALOG_LEFT, RETRO_DEVICE_ID_ANALOG_Y,
    "Tilt Forward/Backward"},
    {0},
};

static struct retro_input_descriptor descWiimoteSideways[] = {
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_LEFT, "Left"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_UP, "Up"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_DOWN, "Down"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_RIGHT, "Right"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_B, "1"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_A, "2"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_X, "A"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_Y, "B"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_R2, "Shake Wiimote"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_START, "+"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_SELECT, "-"},
    //{0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_L3, "Sideways Toggle"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_R3, "Home"},
    {0, RETRO_DEVICE_ANALOG, RETRO_DEVICE_INDEX_ANALOG_LEFT, RETRO_DEVICE_ID_ANALOG_X,
    "Tilt Left/Right"},
    {0, RETRO_DEVICE_ANALOG, RETRO_DEVICE_INDEX_ANALOG_LEFT, RETRO_DEVICE_ID_ANALOG_Y,
    "Tilt Forward/Backward"},
    {0},
};

static struct retro_input_descriptor descWiimoteNunchuk[] = {
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_LEFT, "Left"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_UP, "Up"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_DOWN, "Down"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_RIGHT, "Right"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_B, "B"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_A, "A"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_X, "C"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_Y, "Z"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_L, "-"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_R, "+"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_R2, "Shake Wiimote"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_L2, "Shake Nunchuk"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_START, "1"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_SELECT, "2"},
    //{0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_L3, "Sideways Toggle"},
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_R3, "Home"},
    {0, RETRO_DEVICE_ANALOG, RETRO_DEVICE_INDEX_ANALOG_LEFT, RETRO_DEVICE_ID_ANALOG_X,
    "Nunchuk Stick X"},
    {0, RETRO_DEVICE_ANALOG, RETRO_DEVICE_INDEX_ANALOG_LEFT, RETRO_DEVICE_ID_ANALOG_Y,
    "Nunchuk Stick Y"},
    {0, RETRO_DEVICE_ANALOG, RETRO_DEVICE_INDEX_ANALOG_RIGHT, RETRO_DEVICE_ID_ANALOG_X,
    "Tilt Left/Right"},
    {0, RETRO_DEVICE_ANALOG, RETRO_DEVICE_INDEX_ANALOG_RIGHT, RETRO_DEVICE_ID_ANALOG_Y,
    "Tilt Forward/Backward"},
    {0},
};

static std::string GetDeviceName(unsigned device)
{
  switch (device)
  {
  case RETRO_DEVICE_JOYPAD:
    return "Joypad";
  case RETRO_DEVICE_ANALOG:
    return "Analog";
  case RETRO_DEVICE_MOUSE:
    return "Mouse";
  case RETRO_DEVICE_POINTER:
    return "Pointer";
  case RETRO_DEVICE_KEYBOARD:
    return "Keyboard";
  case RETRO_DEVICE_LIGHTGUN:
    return "Lightgun";
  }
  return "Unknown";
}

static std::string GetQualifiedName(unsigned port, unsigned device)
{
  return ciface::Core::DeviceQualifier(
      std::string(source),
      static_cast<int>(port),
      GetDeviceName(device)
  ).ToString();
}

class Device : public ciface::Core::Device
{
private:
  class Button : public ciface::Core::Device::Input
  {
  public:
    Button(unsigned port, unsigned device, unsigned index, unsigned id, const char* name)
        : m_port(port), m_device(device), m_index(index), m_id(id), m_name(name)
    {
    }
    std::string GetName() const override { return m_name; }
    ControlState GetState() const override { return input_cb(m_port, m_device, m_index, m_id); }

  private:
    const unsigned m_port;
    const unsigned m_device;
    const unsigned m_index;
    const unsigned m_id;
    const char* m_name;
  };

  class Axis : public ciface::Core::Device::Input
  {
  public:
    Axis(unsigned port, unsigned device, unsigned index, unsigned id, s16 range, const char* name)
        : m_port(port), m_device(device), m_index(index), m_id(id), m_range(range), m_name(name)
    {
    }
    std::string GetName() const override { return m_name; }
    ControlState GetState() const override
    {
      return std::max(0.0, input_cb(m_port, m_device, m_index, m_id) / m_range);
    }

  private:
    const unsigned m_port;
    const unsigned m_device;
    const unsigned m_index;
    const unsigned m_id;
    const ControlState m_range;
    const char* m_name;
  };

  class Motor : public ciface::Core::Device::Output
  {
  public:
    Motor(u8 port) : m_port(port) {}
    std::string GetName() const override { return "Rumble"; }
    void SetState(ControlState state) override
    {
      u16 str = std::min(std::max(0.0, state), 1.0) * 0xFFFF;

      if (rumble.set_rumble_state)
      {
        rumble.set_rumble_state(m_port, RETRO_RUMBLE_WEAK, str);
        rumble.set_rumble_state(m_port, RETRO_RUMBLE_STRONG, str);
      }
    }

  private:
    const u8 m_port;
  };
  void AddButton(unsigned id, const char* name, unsigned index = 0)
  {
    AddInput(new Button(m_port, m_device, index, id, name));
  }
  void AddAxis(unsigned id, s16 range, const char* name, unsigned index = 0)
  {
    AddInput(new Axis(m_port, m_device, index, id, range, name));
  }
  void AddMotor() { AddOutput(new Motor(m_port)); }

public:
  Device(unsigned device, unsigned port);
  ciface::Core::DeviceRemoval UpdateInput() override
  {
#if 0
    if (!Libretro::Input::poll_cb)
    {
      return ciface::Core::DeviceRemoval::Keep;
    }

    Libretro::Input::poll_cb();
#endif
    return ciface::Core::DeviceRemoval::Keep;
  }
  std::string GetName() const override { return GetDeviceName(m_device); }
  std::string GetSource() const override { return std::string(source); }
  unsigned int GetPort() const { return m_port; }

private:
  unsigned m_device;
  unsigned m_port;
};

Device::Device(unsigned device, unsigned p) : m_device(device), m_port(p)
{
  switch (device)
  {
  case RETRO_DEVICE_JOYPAD:
    AddButton(RETRO_DEVICE_ID_JOYPAD_B, "B");
    AddButton(RETRO_DEVICE_ID_JOYPAD_Y, "Y");
    AddButton(RETRO_DEVICE_ID_JOYPAD_SELECT, "Select");
    AddButton(RETRO_DEVICE_ID_JOYPAD_START, "Start");
    AddButton(RETRO_DEVICE_ID_JOYPAD_UP, "Up");
    AddButton(RETRO_DEVICE_ID_JOYPAD_DOWN, "Down");
    AddButton(RETRO_DEVICE_ID_JOYPAD_LEFT, "Left");
    AddButton(RETRO_DEVICE_ID_JOYPAD_RIGHT, "Right");
    AddButton(RETRO_DEVICE_ID_JOYPAD_A, "A");
    AddButton(RETRO_DEVICE_ID_JOYPAD_X, "X");
    AddButton(RETRO_DEVICE_ID_JOYPAD_L, "L");
    AddButton(RETRO_DEVICE_ID_JOYPAD_R, "R");
    AddButton(RETRO_DEVICE_ID_JOYPAD_L2, "L2");
    AddButton(RETRO_DEVICE_ID_JOYPAD_R2, "R2");
    AddButton(RETRO_DEVICE_ID_JOYPAD_L3, "L3");
    AddButton(RETRO_DEVICE_ID_JOYPAD_R3, "R3");
    AddMotor();
    return;
  case RETRO_DEVICE_ANALOG:
    AddAxis(RETRO_DEVICE_ID_ANALOG_X, -0x8000, "X0-", RETRO_DEVICE_INDEX_ANALOG_LEFT);
    AddAxis(RETRO_DEVICE_ID_ANALOG_X, 0x7FFF, "X0+", RETRO_DEVICE_INDEX_ANALOG_LEFT);
    AddAxis(RETRO_DEVICE_ID_ANALOG_Y, -0x8000, "Y0-", RETRO_DEVICE_INDEX_ANALOG_LEFT);
    AddAxis(RETRO_DEVICE_ID_ANALOG_Y, 0x7FFF, "Y0+", RETRO_DEVICE_INDEX_ANALOG_LEFT);
    AddAxis(RETRO_DEVICE_ID_ANALOG_X, -0x8000, "X1-", RETRO_DEVICE_INDEX_ANALOG_RIGHT);
    AddAxis(RETRO_DEVICE_ID_ANALOG_X, 0x7FFF, "X1+", RETRO_DEVICE_INDEX_ANALOG_RIGHT);
    AddAxis(RETRO_DEVICE_ID_ANALOG_Y, -0x8000, "Y1-", RETRO_DEVICE_INDEX_ANALOG_RIGHT);
    AddAxis(RETRO_DEVICE_ID_ANALOG_Y, 0x7FFF, "Y1+", RETRO_DEVICE_INDEX_ANALOG_RIGHT);
    AddAxis(RETRO_DEVICE_ID_JOYPAD_L2, 0x7FFF, "Trigger0+", RETRO_DEVICE_INDEX_ANALOG_BUTTON);
    AddAxis(RETRO_DEVICE_ID_JOYPAD_R2, 0x7FFF, "Trigger1+", RETRO_DEVICE_INDEX_ANALOG_BUTTON);
    return;
  case RETRO_DEVICE_MOUSE:
    // TODO: handle last poll_cb relative coordinates correctly.
    AddAxis(RETRO_DEVICE_ID_MOUSE_X, -0x8000, "X-");
    AddAxis(RETRO_DEVICE_ID_MOUSE_X, 0x7FFF, "X+");
    AddAxis(RETRO_DEVICE_ID_MOUSE_Y, -0x8000, "Y-");
    AddAxis(RETRO_DEVICE_ID_MOUSE_Y, 0x7FFF, "Y+");
    AddButton(RETRO_DEVICE_ID_MOUSE_LEFT, "Left");
    AddButton(RETRO_DEVICE_ID_MOUSE_RIGHT, "Right");
    AddButton(RETRO_DEVICE_ID_MOUSE_WHEELUP, "WheelUp");
    AddButton(RETRO_DEVICE_ID_MOUSE_WHEELDOWN, "WheelDown");
    AddButton(RETRO_DEVICE_ID_MOUSE_MIDDLE, "Middle");
    AddButton(RETRO_DEVICE_ID_MOUSE_HORIZ_WHEELUP, "HorizWheelUp");
    AddButton(RETRO_DEVICE_ID_MOUSE_HORIZ_WHEELDOWN, "HorizWheelDown");
    AddButton(RETRO_DEVICE_ID_MOUSE_BUTTON_4, "Button4");
    AddButton(RETRO_DEVICE_ID_MOUSE_BUTTON_5, "Button5");
    return;
  case RETRO_DEVICE_POINTER:
    // All four touch indices, not just the first. libretro's pointer is already
    // multi-touch, and IR passthrough needs four independent points, one per
    // object the Wiimote's camera can see, so the indices carry them rather
    // than inventing a device type for it. Index 0 keeps the names it had, so
    // every existing binding (the IR cursor in mouse mode) is untouched.
    {
      static const char* const kPressed[] = { "Pressed0", "Pressed1", "Pressed2", "Pressed3" };
      static const char* const kXNeg[]    = { "X0-", "X1-", "X2-", "X3-" };
      static const char* const kXPos[]    = { "X0+", "X1+", "X2+", "X3+" };
      static const char* const kYNeg[]    = { "Y0-", "Y1-", "Y2-", "Y3-" };
      static const char* const kYPos[]    = { "Y0+", "Y1+", "Y2+", "Y3+" };
      for (unsigned i = 0; i < 4; ++i)
      {
        AddButton(RETRO_DEVICE_ID_POINTER_PRESSED, kPressed[i], i);
        AddAxis(RETRO_DEVICE_ID_POINTER_X, -0x8000, kXNeg[i], i);
        AddAxis(RETRO_DEVICE_ID_POINTER_X, 0x7FFF, kXPos[i], i);
        AddAxis(RETRO_DEVICE_ID_POINTER_Y, -0x8000, kYNeg[i], i);
        AddAxis(RETRO_DEVICE_ID_POINTER_Y, 0x7FFF, kYPos[i], i);
      }
    }
    return;
  case RETRO_DEVICE_KEYBOARD:
    return;
  case RETRO_DEVICE_LIGHTGUN:
    // TODO: handle absolute coordinates correctly.
    AddAxis(RETRO_DEVICE_ID_LIGHTGUN_SCREEN_X, -320, "X-");
    AddAxis(RETRO_DEVICE_ID_LIGHTGUN_SCREEN_X, 320, "X+");
    AddAxis(RETRO_DEVICE_ID_LIGHTGUN_SCREEN_Y, -264, "Y-");
    AddAxis(RETRO_DEVICE_ID_LIGHTGUN_SCREEN_Y, 264, "Y+");
    AddButton(RETRO_DEVICE_ID_LIGHTGUN_IS_OFFSCREEN, "Offscreen");
    AddButton(RETRO_DEVICE_ID_LIGHTGUN_TRIGGER, "Trigger");
    AddButton(RETRO_DEVICE_ID_LIGHTGUN_RELOAD, "Reload");
    AddButton(RETRO_DEVICE_ID_LIGHTGUN_AUX_A, "A");
    AddButton(RETRO_DEVICE_ID_LIGHTGUN_AUX_B, "B");
    AddButton(RETRO_DEVICE_ID_LIGHTGUN_START, "Start");
    AddButton(RETRO_DEVICE_ID_LIGHTGUN_SELECT, "Select");
    AddButton(RETRO_DEVICE_ID_LIGHTGUN_AUX_C, "C");
    AddButton(RETRO_DEVICE_ID_LIGHTGUN_DPAD_UP, "Up");
    AddButton(RETRO_DEVICE_ID_LIGHTGUN_DPAD_DOWN, "Down");
    AddButton(RETRO_DEVICE_ID_LIGHTGUN_DPAD_LEFT, "Left");
    AddButton(RETRO_DEVICE_ID_LIGHTGUN_DPAD_RIGHT, "Right");
    return;
  }
}

static void AddDevicesForPort(unsigned port)
{
  g_controller_interface.AddDevice(std::make_shared<Device>(RETRO_DEVICE_JOYPAD, port));
  g_controller_interface.AddDevice(std::make_shared<Device>(RETRO_DEVICE_ANALOG, port));
  g_controller_interface.AddDevice(std::make_shared<Device>(RETRO_DEVICE_MOUSE, port));
  g_controller_interface.AddDevice(std::make_shared<Device>(RETRO_DEVICE_POINTER, port));
}

#if 0
/* Disabled as it messes with the controllers index, so player 2 may end up controlling player 3, etc.
 * if controllers are not plugged back in the correct order. */
static void RemoveDevicesForPort(unsigned port)
{
  g_controller_interface.RemoveDevice([&port](const auto& device) {
    return device->GetSource() == source &&
           (device->GetName() == GetDeviceName(RETRO_DEVICE_ANALOG) ||
            device->GetName() == GetDeviceName(RETRO_DEVICE_JOYPAD) ||
            device->GetName() == GetDeviceName(RETRO_DEVICE_MOUSE) ||
            device->GetName() == GetDeviceName(RETRO_DEVICE_POINTER)) &&
           dynamic_cast<const Device*>(device)->GetPort() == port;
  });
}
#endif

void Init(const WindowSystemInfo& wsi)
{
  if (!environ_cb(RETRO_ENVIRONMENT_GET_RUMBLE_INTERFACE, &rumble))
  {
    WARN_LOG_FMT(BOOT, "RetroArch does not support RETRO_ENVIRONMENT_GET_RUMBLE_INTERFACE.");
  }

  if (!environ_cb(RETRO_ENVIRONMENT_GET_SENSOR_INTERFACE, &sensor_interface))
  {
    WARN_LOG_FMT(BOOT, "RetroArch does not support RETRO_ENVIRONMENT_GET_SENSOR_INTERFACE.");
  }

  retro_microphone_interface mic_iface{RETRO_MICROPHONE_INTERFACE_VERSION};

  if (environ_cb(RETRO_ENVIRONMENT_GET_MICROPHONE_INTERFACE, &mic_iface) &&
      mic_iface.interface_version == RETRO_MICROPHONE_INTERFACE_VERSION)
  {
    Libretro::Input::g_microphone_interface = mic_iface;
    Libretro::Input::g_has_microphone_support = true;

    bool gcMicEnable = Libretro::Options::GetCached<bool>(
      Libretro::Options::sysconf_gc::ENABLE_GAMECUBE_MIC);

    if (!Core::System::GetInstance().IsWii() && gcMicEnable)
      Config::SetCurrent(Config::MAIN_SLOT_B, ExpansionInterface::EXIDeviceType::Microphone);
  }
  else
  {
    WARN_LOG_FMT(BOOT, "Microphone interface NOT available");
  }

  g_controller_interface.Initialize(wsi);
  g_controller_interface.AddDevice(std::make_shared<Device>(RETRO_DEVICE_KEYBOARD, 0));

  GCAdapter::Init();
  Pad::Initialize();
  Pad::InitializeGBA();
  Keyboard::Initialize();
  FreeLook::Initialize();

  port_max = (Core::System::GetInstance().IsWii() &&
    Libretro::Options::GetCached<int>(Libretro::Options::sysconf::ALT_GC_PORTS_ON_WII)) ? 8 : 4;
  for (int i = 0; i < port_max; i++)
    Libretro::Input::AddDevicesForPort(i);

  g_init_wiimotes = true;
  s_sensor_init_pending = true;

  Wiimote::Initialize(Wiimote::InitializeMode::DO_NOT_WAIT_FOR_WIIMOTES);
}

void InitStage2()
{
  static const struct retro_controller_description gcpad_desc[] = {
      {"GameCube Controller", RETRO_DEVICE_JOYPAD},
  };

  if (Core::System::GetInstance().IsWii() && !Config::Get(Config::MAIN_BLUETOOTH_PASSTHROUGH_ENABLED))
  {
    // Wii devices listed in ports 1-4, GC controllers in ports 5-8
    if (Libretro::Options::GetCached<int>(Libretro::Options::sysconf::ALT_GC_PORTS_ON_WII))
    {
      static struct retro_controller_description wiimote_desc[] = {
          {"WiiMote", RETRO_DEVICE_WIIMOTE},
          {"WiiMote (sideways)", RETRO_DEVICE_WIIMOTE_SW},
          {"WiiMote + Nunchuk", RETRO_DEVICE_WIIMOTE_NC},
          {"WiiMote + Classic Controller", RETRO_DEVICE_WIIMOTE_CC},
          {"WiiMote + Classic Controller Pro", RETRO_DEVICE_WIIMOTE_CC_PRO},
          {"WiiMote + MotionPlus", RETRO_DEVICE_WIIMOTE_MP},
          {"WiiMote + MotionPlus (sideways)", RETRO_DEVICE_WIIMOTE_MP_SW},
          {"WiiMote + MotionPlus + Nunchuk", RETRO_DEVICE_WIIMOTE_MP_NC},
          {"WiiMote + MotionPlus + Classic Controller", RETRO_DEVICE_WIIMOTE_MP_CC},
          {"WiiMote + MotionPlus + Classic Controller Pro", RETRO_DEVICE_WIIMOTE_MP_CC_PRO},
          {"Real WiiMote", RETRO_DEVICE_REAL_WIIMOTE},
      };

      const struct retro_controller_info ports[] = {
          {wiimote_desc, sizeof(wiimote_desc) / sizeof(*wiimote_desc)},
          {wiimote_desc, sizeof(wiimote_desc) / sizeof(*wiimote_desc)},
          {wiimote_desc, sizeof(wiimote_desc) / sizeof(*wiimote_desc)},
          {wiimote_desc, sizeof(wiimote_desc) / sizeof(*wiimote_desc)},
          {gcpad_desc, sizeof(gcpad_desc) / sizeof(*gcpad_desc)},
          {gcpad_desc, sizeof(gcpad_desc) / sizeof(*gcpad_desc)},
          {gcpad_desc, sizeof(gcpad_desc) / sizeof(*gcpad_desc)},
          {gcpad_desc, sizeof(gcpad_desc) / sizeof(*gcpad_desc)},
          {0},
      };

      if (!environ_cb(RETRO_ENVIRONMENT_SET_CONTROLLER_INFO, (void*)ports))
        WARN_LOG_FMT(BOOT, "RetroArch does not support RETRO_ENVIRONMENT_SET_CONTROLLER_INFO.");
    }
    else // Both Wii devices and GC controllers listed in ports 1-4, ports 5-8 are unused
    {
      static struct retro_controller_description wii_and_gc_desc[] = {
          {"WiiMote", RETRO_DEVICE_WIIMOTE},
          {"WiiMote (sideways)", RETRO_DEVICE_WIIMOTE_SW},
          {"WiiMote + Nunchuk", RETRO_DEVICE_WIIMOTE_NC},
          {"WiiMote + Classic Controller", RETRO_DEVICE_WIIMOTE_CC},
          {"WiiMote + Classic Controller Pro", RETRO_DEVICE_WIIMOTE_CC_PRO},
          {"WiiMote + MotionPlus", RETRO_DEVICE_WIIMOTE_MP},
          {"WiiMote + MotionPlus (sideways)", RETRO_DEVICE_WIIMOTE_MP_SW},
          {"WiiMote + MotionPlus + Nunchuk", RETRO_DEVICE_WIIMOTE_MP_NC},
          {"WiiMote + MotionPlus + Classic Controller", RETRO_DEVICE_WIIMOTE_MP_CC},
          {"WiiMote + MotionPlus + Classic Controller Pro", RETRO_DEVICE_WIIMOTE_MP_CC_PRO},
          {"Real WiiMote", RETRO_DEVICE_REAL_WIIMOTE},
          {"GameCube Controller", RETRO_DEVICE_GC_ON_WII},
          {"Game Boy Advance (link cable)", RETRO_DEVICE_GBA_LINK},
      };

      const struct retro_controller_info ports[] = {
          {wii_and_gc_desc, sizeof(wii_and_gc_desc) / sizeof(*wii_and_gc_desc)},
          {wii_and_gc_desc, sizeof(wii_and_gc_desc) / sizeof(*wii_and_gc_desc)},
          {wii_and_gc_desc, sizeof(wii_and_gc_desc) / sizeof(*wii_and_gc_desc)},
          {wii_and_gc_desc, sizeof(wii_and_gc_desc) / sizeof(*wii_and_gc_desc)},
          {0},
      };

      if (!environ_cb(RETRO_ENVIRONMENT_SET_CONTROLLER_INFO, (void*)ports))
        WARN_LOG_FMT(BOOT, "RetroArch does not support RETRO_ENVIRONMENT_SET_CONTROLLER_INFO.");
    }
  }
  else
  {
    static const struct retro_controller_info ports[] = {
        {gcpad_desc, sizeof(gcpad_desc) / sizeof(*gcpad_desc)},
        {gcpad_desc, sizeof(gcpad_desc) / sizeof(*gcpad_desc)},
        {gcpad_desc, sizeof(gcpad_desc) / sizeof(*gcpad_desc)},
        {gcpad_desc, sizeof(gcpad_desc) / sizeof(*gcpad_desc)},
        {0},
    };

    if (!environ_cb(RETRO_ENVIRONMENT_SET_CONTROLLER_INFO, (void*)ports))
      WARN_LOG_FMT(BOOT, "RetroArch does not support RETRO_ENVIRONMENT_SET_CONTROLLER_INFO.");
  }
}

void InitSensors()
{
  if (!s_sensor_init_pending)
    return;

  // sensors do not apply to GC, bluetooth passthrough and neither does default controller
  if (!Core::System::GetInstance().IsWii() || Config::Get(Config::MAIN_BLUETOOTH_PASSTHROUGH_ENABLED))
  {
    s_sensor_init_pending = false;
    return;
  }

  port_max = (Core::System::GetInstance().IsWii() &&
    Libretro::Options::GetCached<int>(Libretro::Options::sysconf::ALT_GC_PORTS_ON_WII)) ? 8 : 4;

  for (int i = 0; i < port_max; i++)
  {
    if (sensor_interface.set_sensor_state)
    {
      for (unsigned s = 0; s < NUM_SENSOR_SUB_DEVICES; s++)
      {
        // The sub-device rides in the high bits of the action. Asking a
        // frontend that has never heard of sub-devices for anything but 0 gets
        // false back, which is the fallback rather than a failure: the flag
        // stays clear, no device is registered, and nothing downstream binds.
        const retro_sensor_action accel_on = static_cast<retro_sensor_action>(
            RETRO_SENSOR_ID(s, RETRO_SENSOR_ACCELEROMETER_ENABLE));
        sensor_enabled[i][s][SENSOR_ACCELEROMETER] =
            sensor_interface.set_sensor_state(i, accel_on, 60);

        // Only the remote itself has a gyroscope to ask for; a Nunchuk has none.
        if (s == 0)
        {
          sensor_enabled[i][s][SENSOR_GYRO] =
              sensor_interface.set_sensor_state(i, RETRO_SENSOR_GYROSCOPE_ENABLE, 60);
        }

        if (sensor_enabled[i][s][SENSOR_ACCELEROMETER] || sensor_enabled[i][s][SENSOR_GYRO])
        {
          auto sensor = std::make_shared<SensorDevice>(i, s);
          sensor->RegisterAll();
          g_controller_interface.AddDevice(sensor);
        }

        INFO_LOG_FMT(BOOT, "Sensor interface: Port: {} Sub-device: {} ACCELEROMETER: {} GYROSCOPE: {}",
          i, s, sensor_enabled[i][s][SENSOR_ACCELEROMETER], sensor_enabled[i][s][SENSOR_GYRO]);
      }
    }
  }

  s_sensor_init_pending = false;

  ResetControllers(WiimoteUpdateFlags{});
}

void Shutdown()
{
  if (g_init_wiimotes)
  {
    Wiimote::ResetAllWiimotes();
    Wiimote::Shutdown();
    g_init_wiimotes = false;
  }

  s_sensor_init_pending = false;

#if defined(__LIBUSB__)
  GCAdapter::ResetRumble();
#endif
  for (int i = 0; i < port_max; ++i)
  {
    Pad::ResetRumble(i);

    // Each sensor turns off the one it actually turned on, on the port it was
    // enabled for. This used to cross the two over and address port 0 every
    // time, so a multi-remote session left every sensor but port 0's running.
    for (unsigned s = 0; s < NUM_SENSOR_SUB_DEVICES; s++)
    {
      if (sensor_enabled[i][s][SENSOR_ACCELEROMETER])
        sensor_interface.set_sensor_state(i, static_cast<retro_sensor_action>(
            RETRO_SENSOR_ID(s, RETRO_SENSOR_ACCELEROMETER_DISABLE)), 0);

      if (sensor_enabled[i][s][SENSOR_GYRO])
        sensor_interface.set_sensor_state(i, static_cast<retro_sensor_action>(
            RETRO_SENSOR_ID(s, RETRO_SENSOR_GYROSCOPE_DISABLE)), 0);

      sensor_enabled[i][s][SENSOR_ACCELEROMETER] = false;
      sensor_enabled[i][s][SENSOR_GYRO] = false;
    }
  }

  Keyboard::Shutdown();
  Pad::Shutdown();
  g_controller_interface.Shutdown();

  rumble.set_rumble_state = nullptr;
}

void UpdateAccelerometer(unsigned port, unsigned index)
{
  if (!sensor_enabled[port][index][SENSOR_ACCELEROMETER] || !sensor_interface.get_sensor_input)
    return;

  static const float G = 9.80665f;

  // read raw sensor values
  float ax = sensor_interface.get_sensor_input(
      port, RETRO_SENSOR_ID(index, RETRO_SENSOR_ACCELEROMETER_X)) * G;
  float ay = sensor_interface.get_sensor_input(
      port, RETRO_SENSOR_ID(index, RETRO_SENSOR_ACCELEROMETER_Y)) * G;
  float az = sensor_interface.get_sensor_input(
      port, RETRO_SENSOR_ID(index, RETRO_SENSOR_ACCELEROMETER_Z)) * G;

  // Collapsed, so a sideways remote with the dongle fitted still turns: holding
  // it sideways rotates what its sensors read, and MotionPlus does not change
  // which way up it is being held.
  //
  // The remote only. Holding a REMOTE sideways is a way of playing; a Nunchuk
  // is held the one way round whatever the remote is doing, so rotating its
  // reading too would tilt the wrong hand.
  if (index == 0 && wiimote_base_device(input_types[port]) == RETRO_DEVICE_WIIMOTE_SW)
  {
    float rx = -ay;   // rotate 90° clockwise
    float ry =  ax;
    ax = rx;
    ay = ry;
  }

  // write rotated values
  g_accel_pos[port][index][0] = std::max(0.0f, ax);
  g_accel_neg[port][index][0] = std::max(0.0f, -ax);

  g_accel_pos[port][index][1] = std::max(0.0f, ay);
  g_accel_neg[port][index][1] = std::max(0.0f, -ay);

  g_accel_pos[port][index][2] = std::max(0.0f, az);
  g_accel_neg[port][index][2] = std::max(0.0f, -az);
}

void UpdateGyro(unsigned port)
{
  // Sub-device 0 only: the gyroscope is the remote's, by way of MotionPlus.
  if (!sensor_enabled[port][0][SENSOR_GYRO] || !sensor_interface.get_sensor_input)
    return;

  // Angular velocity in rad/s about the remote's own axes, same frame the
  // accelerometer above arrives in: +X left, +Y back, +Z up.
  float gx = sensor_interface.get_sensor_input(port, RETRO_SENSOR_GYROSCOPE_X);
  float gy = sensor_interface.get_sensor_input(port, RETRO_SENSOR_GYROSCOPE_Y);
  float gz = sensor_interface.get_sensor_input(port, RETRO_SENSOR_GYROSCOPE_Z);

  if (wiimote_base_device(input_types[port]) == RETRO_DEVICE_WIIMOTE_SW)
  {
    float rx = -gy;   // rotate 90° clockwise
    float ry =  gx;
    gx = rx;
    gy = ry;
  }

  // Split each axis across a one-sided pair. A control cannot carry a negative
  // value through the expression parser (see SensorDevice::RegisterAll), so the
  // sign lives in WHICH slot is non-zero rather than in the number itself.
  g_gyro_pos[port][0][0] = std::max(0.0f, gx);
  g_gyro_neg[port][0][0] = std::max(0.0f, -gx);

  g_gyro_pos[port][0][1] = std::max(0.0f, gy);
  g_gyro_neg[port][0][1] = std::max(0.0f, -gy);

  g_gyro_pos[port][0][2] = std::max(0.0f, gz);
  g_gyro_neg[port][0][2] = std::max(0.0f, -gz);
}

void ResetControllers(const WiimoteUpdateFlags& f)
{
  // only update what has changed
  if (f.any())
  {
    auto& system = Core::System::GetInstance();
    if (!system.IsWii())
    {
      for (int port = 0; port < 4; port++)
        UpdateGCMappings(f, port, input_types[port]);
      return;
    }

    for (int port = 0; port < 4; port++)
      UpdateWiimoteMappings(f, port, input_types[port]);
    return;
  }

  for (int port = 0; port < port_max; port++)
    retro_set_controller_port_device(port, input_types[port]);
}

void BluetoothPassthroughBind()
{
  static bool was_pressed = false;
  bool sync = Libretro::Input::input_cb(0, RETRO_DEVICE_KEYBOARD, 0, RETROK_F12);

  if (sync && !was_pressed)
  {
    auto bt_real = WiiUtils::GetBluetoothRealDevice();
    if (bt_real)
      bt_real->TriggerSyncButtonPressedEvent();
  }

  was_pressed = sync;
}

static int GetRetroButtonId(const std::string& name)
{
  if (name == "A") return RETRO_DEVICE_ID_JOYPAD_A;
  if (name == "B") return RETRO_DEVICE_ID_JOYPAD_B;
  if (name == "X") return RETRO_DEVICE_ID_JOYPAD_X;
  if (name == "Y") return RETRO_DEVICE_ID_JOYPAD_Y;
  if (name == "L") return RETRO_DEVICE_ID_JOYPAD_L2;
  if (name == "R") return RETRO_DEVICE_ID_JOYPAD_R2;
  if (name == "L2") return RETRO_DEVICE_ID_JOYPAD_L;
  if (name == "R2") return RETRO_DEVICE_ID_JOYPAD_R;
  if (name == "L3") return RETRO_DEVICE_ID_JOYPAD_L3;
  if (name == "R3") return RETRO_DEVICE_ID_JOYPAD_R3;
  if (name == "Start") return RETRO_DEVICE_ID_JOYPAD_START;
  if (name == "Select") return RETRO_DEVICE_ID_JOYPAD_SELECT;
  return -1;
}

void Update()
{
  if (poll_cb)
    poll_cb();
#ifdef __ANDROID__
  /* Android doesn't support input polling on all threads by default
   * this will force the poll for this frame to happen in the main thread
   * in case the frontend is doing late-polling */
  if (input_cb)
    input_cb(0, 0, 0, 0);
#endif

  for (int i = 0; i < port_max; ++i)
  {
    for (unsigned s = 0; s < NUM_SENSOR_SUB_DEVICES; ++s)
      UpdateAccelerometer(i, s);
    UpdateGyro(i);
  }

  auto& system = Core::System::GetInstance();
  bool gcMicEnable = Libretro::Options::GetCached<bool>(Libretro::Options::sysconf_gc::ENABLE_GAMECUBE_MIC);

  if (!system.IsWii() && Libretro::Input::g_has_microphone_support && gcMicEnable)
  {
    std::string micHotkey = Libretro::Options::GetCached<std::string>(
      Libretro::Options::sysconf_gc::HOTKEY_ACTIVATE_MICROPHONE);
    int micButtonId = GetRetroButtonId(micHotkey);

    for (int i = 0; i < 4; i++)
    {
      g_gc_mic_button[i] = (micButtonId >= 0) ?
        (input_cb(i, RETRO_DEVICE_JOYPAD, 0, micButtonId) != 0) : false;
    }
  }
}

static std::string GetQualifiedNameSensor(unsigned port, unsigned index)
{
  return ciface::Core::DeviceQualifier(
      std::string(Libretro::Input::source),
      static_cast<int>(port),
      Libretro::Input::SensorDeviceName(index)
  ).ToString();
}

// can be called from retro_run, do not reset all settings because one thing changed
void UpdateWiimoteMappings(const WiimoteUpdateFlags& f, unsigned port, unsigned device)
{
  // Nothing below is affected by the dongle, and ResetControllers hands us
  // input_types[port] verbatim, which may name a MotionPlus variant.
  device = wiimote_base_device(device);

  if (!f.any() || device == RETRO_DEVICE_REAL_WIIMOTE || device == RETRO_DEVICE_WIIMOTE_CC ||
    device == RETRO_DEVICE_WIIMOTE_CC_PRO)
    return;

  auto& system = Core::System::GetInstance();
  bool altGCPorts = Libretro::Options::GetCached<bool>(Libretro::Options::sysconf::ALT_GC_PORTS_ON_WII);

  if (((!system.IsWii() || !altGCPorts) && port > 3) || port > 7)
    return;

  // modifier wrapper
  auto wrap = [](const std::string& mod, const std::string& expr) {
    if (mod == MODIFIER_NO_MODIFIER || mod.empty())
      return expr;                   // no modifier - always active
    return mod + " & " + expr;       // modifier held - active
  };

  WiimoteEmu::Wiimote* wm = (WiimoteEmu::Wiimote*)Wiimote::GetConfig()->GetController(port);

  // IR‑related updates
  if (f.irModifier ||
      f.irMode ||
      f.irOffset ||
      f.irYaw ||
      f.irPitch ||
      f.irDeadzone ||
      f.swingModifier)
  {
    ControllerEmu::ControlGroup* wmIR = wm->GetWiimoteGroup(WiimoteEmu::WiimoteGroup::Point);
    const int irMode = Libretro::Options::GetCached<int>(Libretro::Options::wiimote::IR_MODE);

    if (f.irOffset)
    {
      const int irCenter = Libretro::Options::GetCached<int>(Libretro::Options::wiimote::IR_OFFSET);
      static_cast<ControllerEmu::NumericSetting<double>*>(wmIR->numeric_settings[1].get())
        ->SetValue(irCenter); // IR Vertical Offset
    }

    if (f.irYaw)
    {
      const int irWidth = Libretro::Options::GetCached<int>(Libretro::Options::wiimote::IR_YAW);
      static_cast<ControllerEmu::NumericSetting<double>*>(wmIR->numeric_settings[2].get())
        ->SetValue(irWidth);  // IR Total Yaw
    }

    if (f.irPitch)
    {
      const int irHeight = Libretro::Options::GetCached<int>(Libretro::Options::wiimote::IR_PITCH);
      static_cast<ControllerEmu::NumericSetting<double>*>(wmIR->numeric_settings[3].get())
        ->SetValue(irHeight); // IR Total Pitch
    }


    if (f.irMode || f.irModifier || f.swingModifier)
    {
      std::string devAnalog = Libretro::Input::GetQualifiedName(port, RETRO_DEVICE_ANALOG);
      std::string devPointer = Libretro::Input::GetQualifiedName(port, RETRO_DEVICE_POINTER);

      if (irMode == 0 || irMode == 1)
      {
        // Set right stick to control the IR
        std::string irMod = Libretro::Options::GetCached<std::string>(Libretro::Options::wiimote::IR_MODIFIER);
        wmIR->SetControlExpression(0, wrap(irMod, "`" + devAnalog + ":Y1-`")); // Up
        wmIR->SetControlExpression(1, wrap(irMod, "`" + devAnalog + ":Y1+`")); // Down
        wmIR->SetControlExpression(2, wrap(irMod, "`" + devAnalog + ":X1-`")); // Left
        wmIR->SetControlExpression(3, wrap(irMod, "`" + devAnalog + ":X1+`")); // Right

        static_cast<ControllerEmu::NumericSetting<bool>*>(wmIR->numeric_settings[4].get())
          ->SetValue(irMode == 0);                                    // Relative input
        static_cast<ControllerEmu::NumericSetting<bool>*>(wmIR->numeric_settings[5].get())
          ->SetValue(true);                                           // Auto hide

        // Swing‑related updates
        ControllerEmu::ControlGroup* wmSwing = wm->GetWiimoteGroup(WiimoteEmu::WiimoteGroup::Swing);

        // modifier controls swing plus analog
        std::string swingModifier = Libretro::Options::GetCached<std::string>(Libretro::Options::wiimote::SWING_MODIFIER);

        if (swingModifier != MODIFIER_DISABLED_CONTROL)
        {
          // we don't want whatever the irMod is set to (e.g. L3) to interfere with swinging
          if (swingModifier == MODIFIER_NO_MODIFIER && irMod != swingModifier)
            swingModifier = "!" + irMod;

          wmSwing->SetControlExpression(0, wrap(swingModifier, "`" + devAnalog + ":Y1-`"));  // Up
          wmSwing->SetControlExpression(1, wrap(swingModifier, "`" + devAnalog + ":Y1+`"));  // Down
          wmSwing->SetControlExpression(2, wrap(swingModifier, "`" + devAnalog + ":X1-`"));  // Left
          wmSwing->SetControlExpression(3, wrap(swingModifier, "`" + devAnalog + ":X1+`"));  // Right
        }
        else
        {
          wmSwing->SetControlExpression(0, "");
          wmSwing->SetControlExpression(1, "");
          wmSwing->SetControlExpression(2, "");
          wmSwing->SetControlExpression(3, "");
        }
      }
      else
      {
        // Mouse controls IR
        wmIR->SetControlExpression(0, "`" + devPointer + ":Y0-`");    // Up
        wmIR->SetControlExpression(1, "`" + devPointer + ":Y0+`");    // Down
        wmIR->SetControlExpression(2, "`" + devPointer + ":X0-`");    // Left
        wmIR->SetControlExpression(3, "`" + devPointer + ":X0+`");    // Right
        static_cast<ControllerEmu::NumericSetting<bool>*>(wmIR->numeric_settings[4].get())
          ->SetValue(false);                                          // Relative input
        static_cast<ControllerEmu::NumericSetting<bool>*>(wmIR->numeric_settings[5].get())
          ->SetValue(false);                                          // Auto hide
      }

      if (f.irMode && device == RETRO_DEVICE_WIIMOTE_NC)
      {
        // mouse
        if (irMode != 0 && irMode != 1)
        {
          ControllerEmu::ControlGroup* wmTilt = wm->GetWiimoteGroup(WiimoteEmu::WiimoteGroup::Tilt);
          wmTilt->SetControlExpression(0, "`" + devAnalog + ":Y1-`");  // Forward
          wmTilt->SetControlExpression(1, "`" + devAnalog + ":Y1+`");  // Backward
          wmTilt->SetControlExpression(2, "`" + devAnalog + ":X1-`");  // Left
          wmTilt->SetControlExpression(3, "`" + devAnalog + ":X1+`");  // Right
        }
      }
    }

    if (f.irDeadzone)
    {
      const int irDeadzone = Libretro::Options::GetCached<int>(Libretro::Options::wiimote::IR_DEADZONE);
      static_cast<ControllerEmu::NumericSetting<double>*>(wmIR->numeric_settings[0].get())
        ->SetValue(irDeadzone); // IR DeadZone
    }
  }

  // Raw IR. When this is on, the frontend hands us the camera's actual view of
  // the sensor bar and BuildDesiredWiimoteState uses it verbatim. The Point
  // group, Total Yaw/Pitch, the vertical offset and the sensor-bar position are
  // all bypassed (WiimoteEmu.cpp, "if m_ir_passthrough->enabled").
  //
  // That is the whole point. The cursor path parks a notional remote two metres
  // from the bar and rotates it by a scale nobody can derive, so where the game
  // draws its hand depends on a constant fitted per game. A frontend that knows
  // the real geometry can compute the dots outright, and gets roll and distance
  // for free, neither of which two angles can express.
  //
  // Objects arrive on pointer indices 0-3: X and Y over the camera's 0..1 field
  // (so the frontend sends the POSITIVE half of the pointer range, 0..32767),
  // and PRESSED says the object is visible. Size is a small constant rather than
  // a channel of its own, because nothing here has a fifth axis to spare, and games
  // read it to reject noise rather than to measure anything.
  if (f.irPassthrough)
  {
    auto* wmIRPass = static_cast<ControllerEmu::IRPassthrough*>(
      wm->GetWiimoteGroup(WiimoteEmu::WiimoteGroup::IRPassthrough));
    const bool passthrough =
      Libretro::Options::GetCached<bool>(Libretro::Options::wiimote::IR_PASSTHROUGH);

    if (wmIRPass)
    {
      // Its own copy: the one above is scoped to the cursor-mode branch, which
      // this path deliberately does not run through.
      const std::string devPointer =
        Libretro::Input::GetQualifiedName(port, RETRO_DEVICE_POINTER);
      wmIRPass->enabled.SetValue(passthrough);
      static const char* const kObj[] = { "0", "1", "2", "3" };
      for (int i = 0; i < 4; ++i)
      {
        // Cleared rather than left bound when off: AreInputsBound() is half of
        // what selects this path, so a stale binding would keep the cursor route
        // switched off after the option was turned back off.
        const std::string idx = kObj[i];
        wmIRPass->SetControlExpression(i * 3 + 0,
          passthrough ? "`" + devPointer + ":X" + idx + "+`" : "");
        wmIRPass->SetControlExpression(i * 3 + 1,
          passthrough ? "`" + devPointer + ":Y" + idx + "+`" : "");
        wmIRPass->SetControlExpression(i * 3 + 2,
          passthrough ? "`" + devPointer + ":Pressed" + idx + "` * 0.2" : "");
      }
    }
  }

  if (f.swingAngle)
  {
    ControllerEmu::ControlGroup* wmSwing = wm->GetWiimoteGroup(WiimoteEmu::WiimoteGroup::Swing);

    const int swingAngle = Libretro::Options::GetCached<int>(Libretro::Options::wiimote::SWING_ANGLE);
    static_cast<ControllerEmu::NumericSetting<double>*>(wmSwing->numeric_settings[0].get())
      ->SetValue(swingAngle);                                           // Swing/Angle
  }

  // Sideways toggle
  if (f.sideways)
  {
    ControllerEmu::ControlGroup* wmHotkeys = wm->GetWiimoteGroup(WiimoteEmu::WiimoteGroup::Hotkeys);
    std::string sidewaysToggle =
      Libretro::Options::GetCached<std::string>(Libretro::Options::wiimote::HOTKEY_SIDEWAYS_TOGGLE);

    wmHotkeys->SetControlExpression(0,
      sidewaysToggle != MODIFIER_DISABLED_CONTROL ? sidewaysToggle : "");  // Sideways Toggle (L3 default)

#if 0
    wmHotkeys->SetControlExpression(1, "");  // Upright Toggle
    wmHotkeys->SetControlExpression(2, "");  // Sideways Hold
    wmHotkeys->SetControlExpression(3, "");  // Upright Hold
#endif
  }

  // Rumble
  if (f.rumble)
  {
    ControllerEmu::ControlGroup* wmRumble = wm->GetWiimoteGroup(WiimoteEmu::WiimoteGroup::Rumble);
    bool enableRumble = Libretro::Options::GetCached<bool>(Libretro::Options::sysconf::ENABLE_RUMBLE);
    if (enableRumble)
      wmRumble->SetControlExpression(0, "Rumble");
  }

  if (f.any())
  {
    wm->UpdateReferences(g_controller_interface);

    bool saveOrLoad = Libretro::Options::GetCached<bool>(Libretro::Options::retroarch_core::SAVE_LOAD_SETTINGS);
    if (!saveOrLoad)
      ::Wiimote::GetConfig()->SaveConfig();
  }
}

void UpdateGCMappings(const WiimoteUpdateFlags& f, unsigned port, unsigned device)
{
  if (!f.any())
    return;

  auto& system = Core::System::GetInstance();

  if (system.IsWii())
    return;

  if (f.gcMicBtn && Libretro::Input::g_has_microphone_support)
  {
    GCPad* gcPad = (GCPad*)Pad::GetConfig()->GetController(port > 3 ? port - 4 : port);
    ControllerEmu::ControlGroup* gcMic = gcPad->GetGroup(PadGroup::Mic);
    std::string gcMicButton = Libretro::Options::GetCached<std::string>
      (Libretro::Options::sysconf_gc::HOTKEY_ACTIVATE_MICROPHONE);

    if (!gcMicButton.empty())
      gcMic->SetControlExpression(0, gcMicButton);
  }
}

}  // namespace Input
}  // namespace Libretro

void retro_set_input_poll(retro_input_poll_t cb)
{
  Libretro::Input::poll_cb = cb;
}

void retro_set_input_state(retro_input_state_t cb)
{
  Libretro::Input::input_cb = cb;
}

void retro_set_controller_port_device(unsigned port, unsigned device)
{
  auto& system = Core::System::GetInstance();
  bool altGCPorts = Libretro::Options::GetCached<bool>(Libretro::Options::sysconf::ALT_GC_PORTS_ON_WII);

  if (((!system.IsWii() || !altGCPorts) && port > 3) || port > 7)
    return;

  Libretro::Input::input_types[port] = device;
  auto& si = Core::System::GetInstance().GetSerialInterface();

  if (device == RETRO_DEVICE_NONE)
  {
    if (system.IsWii() && port < 4)
    {
      Config::SetBaseOrCurrent(Config::GetInfoForWiimoteSource(port), WiimoteSource::None);
      WiimoteCommon::OnSourceChanged(port, WiimoteSource::None);
    }

    if (!system.IsWii() || !altGCPorts)
    {
      Config::SetBaseOrCurrent(Config::GetInfoForSIDevice(port), SerialInterface::SIDEVICE_NONE);
      si.ChangeDevice(Config::Get(Config::GetInfoForSIDevice(port)), port);
    }
    else if (port > 3)
    {
      Config::SetBaseOrCurrent(Config::GetInfoForSIDevice(port - 4), SerialInterface::SIDEVICE_NONE);
      si.ChangeDevice(Config::Get(Config::GetInfoForSIDevice(port - 4)), port - 4);
    }
  }
  else if (!system.IsWii() || device == RETRO_DEVICE_GC_ON_WII || port > 3)
  {
    retro_set_controller_port_device_gc(port, device);
  }
  else if (!Config::Get(Config::MAIN_BLUETOOTH_PASSTHROUGH_ENABLED))
  {
    retro_set_controller_port_device_wii(port, device);
  }

  std::vector<retro_input_descriptor> all_descs;

  int port_max = (system.IsWii() && altGCPorts) ? 8 : 4;

  for (int i = 0; i < port_max; i++)
  {
    retro_input_descriptor* desc;

    // Through the same collapse as everywhere else: the dongle adds no buttons,
    // so a MotionPlus remote wants its twin's descriptors. Left raw, every
    // MotionPlus id would fall to default and a Nunchuk's labels would go
    // missing.
    switch (wiimote_base_device(Libretro::Input::input_types[i]))
    {
    case RETRO_DEVICE_WIIMOTE_SW:
      desc = Libretro::Input::descWiimoteSideways;
      break;

    case RETRO_DEVICE_WIIMOTE_NC:
      desc = Libretro::Input::descWiimoteNunchuk;
      break;

    case RETRO_DEVICE_WIIMOTE_CC:
      desc = Libretro::Input::descWiimoteCC;
      break;

    case RETRO_DEVICE_WIIMOTE_CC_PRO:
      desc = Libretro::Input::descWiimoteCCPro;
      break;

    case RETRO_DEVICE_REAL_WIIMOTE:
    case RETRO_DEVICE_NONE:
      continue;

    case RETRO_DEVICE_GC_ON_WII:
      desc = Libretro::Input::descGC;
      break;

    case RETRO_DEVICE_GBA_LINK:
      // Its buttons belong to the machine on the other end of the lead.
      continue;

    default:
      if (!system.IsWii() || i > 3)
      {
        desc = Libretro::Input::descGC;
      }
      else
      {
        desc = Libretro::Input::descWiimote;
      }
      break;
    }

    for (int j = 0; desc[j].description != NULL; j++)
    {
      retro_input_descriptor new_desc = desc[j];
      new_desc.port = i;
      all_descs.push_back(new_desc);
    }
  }

  all_descs.push_back({0});

  if(!Libretro::environ_cb(RETRO_ENVIRONMENT_SET_INPUT_DESCRIPTORS, &all_descs[0]))
  {
    WARN_LOG_FMT(COMMON, "RetroArch does not support RETRO_ENVIRONMENT_SET_INPUT_DESCRIPTORS.");
  }
}

void refresh_all_wiimote_flags(unsigned port, unsigned device)
{
  WiimoteUpdateFlags f;
  f.sideways     = true;
  f.irModifier   = true;
  f.swingModifier = true;
  f.irMode       = true;
  f.irOffset     = true;
  f.irYaw        = true;
  f.irPitch      = true;
  f.irDeadzone   = true;
  f.swingAngle   = true;
  f.rumble       = true;
  Libretro::Input::UpdateWiimoteMappings(f, port, device);
}

// Mirror InputConfig::LoadConfig() for a single port.
//  1) game-specific input profile (game ini [Controls] WiimoteProfile{n}),
//  2) WiimoteNew.ini [Wiimote{n}] section,
// Returns the configured Wiimote pointer if a saved config was found and loaded,
// nullptr if the caller should apply hard-coded defaults instead.
// SetDefaultDevice is always called before UpdateReferences so profiles without
// a Device= line still bind to the correct libretro joypad.
static WiimoteEmu::Wiimote* load_saved_controller_config(unsigned port, unsigned device)
{
  bool saveOrLoad = Libretro::Options::GetCached<bool>(Libretro::Options::retroarch_core::SAVE_LOAD_SETTINGS);
  if (!saveOrLoad)
    return nullptr;

  WiimoteEmu::Wiimote* wm =
      (WiimoteEmu::Wiimote*)Wiimote::GetConfig()->GetController(port);

  // Rebuild devJoypad here so SetDefaultDevice is always called inside this
  // function, before UpdateReferences, regardless of which load path fires.
  const std::string devJoypad = Libretro::Input::GetQualifiedName(port, RETRO_DEVICE_JOYPAD);

  const std::string profile_directory = ::Wiimote::GetConfig()->GetUserProfileDirectoryPath();
  const std::string profile_key =
      fmt::format("{}Profile{}", ::Wiimote::GetConfig()->GetProfileKey(), port + 1);

  // Step 1: game ini [Controls] WiimoteProfile{n} -> Profiles/Wiimote/<name>.ini [Profile]
  // LoadGameIni() already resolves the priority chain: S.ini, SOE.ini, SOUE01.ini, SOUE01r{n}.ini
  Common::IniFile game_ini = SConfig::GetInstance().LoadGameIni();
  const auto* control_section = game_ini.GetOrCreateSection("Controls");
  std::string profile_setting;
  if (control_section->Get(profile_key, &profile_setting))
  {
    const auto profiles = InputProfile::GetProfilesFromSetting(profile_setting, profile_directory);
    if (!profiles.empty())
    {
      Common::IniFile profile_ini;
      if (profile_ini.Load(profiles[0]))
      {
        Common::IniFile::Section* profile_sec = profile_ini.GetOrCreateSection("Profile");
        wm->LoadConfig(profile_sec);
        wm->SetDefaultDevice(devJoypad);
        wm->UpdateReferences(g_controller_interface);
        refresh_all_wiimote_flags(port, device);
        ::Wiimote::GetConfig()->SaveConfig();
        return wm;  // Per-game profile loaded
      }
    }
  }

  // Step 2: WiimoteNew.ini [Wiimote{n}] - GetSection not GetOrCreateSection so a
  // missing section doesn't silently bypass the hard-coded defaults in the caller.
  const std::string wiimote_ini_path =
      File::GetUserPath(D_CONFIG_IDX) + WIIMOTE_INI_NAME ".ini";
  Common::IniFile wiimote_new_ini;
  if (wiimote_new_ini.Load(wiimote_ini_path))
  {
    Common::IniFile::Section* wm_sec = wiimote_new_ini.GetSection(wm->GetName());
    if (wm_sec)
    {
      wm->LoadConfig(wm_sec);
      wm->SetDefaultDevice(devJoypad);
      wm->UpdateReferences(g_controller_interface);
      refresh_all_wiimote_flags(port, device);
      ::Wiimote::GetConfig()->SaveConfig();
      return wm;  // WiimoteNew.ini loaded
    }
  }

  return nullptr;  // nothing found, applies hard-coded defaults
}

// returns retropad_expr on its own, with additional mouse bindings
static std::string bindMouse(const std::string& retropad_expr, const std::string& mouse_target)
{
  static const bool enable_default_mouse_bindings =
    Libretro::Options::GetCached<bool>(Libretro::Options::retroarch_core::ENABLE_DEFAULT_MOUSE_BINDINGS, /*def=*/true);

  if (!enable_default_mouse_bindings)
    return retropad_expr;

  return retropad_expr + " | `" + mouse_target + "`";
}

void retro_set_controller_port_device_gc(unsigned port, unsigned device)
{
  std::string devJoypad = Libretro::Input::GetQualifiedName(port, RETRO_DEVICE_JOYPAD);
  std::string devAnalog = Libretro::Input::GetQualifiedName(port, RETRO_DEVICE_ANALOG);
  std::string devMouse = Libretro::Input::GetQualifiedName(port, RETRO_DEVICE_MOUSE);
  std::string devPointer = Libretro::Input::GetQualifiedName(port, RETRO_DEVICE_POINTER);
#if 0
  std::string devKeyboard = Libretro::Input::GetQualifiedName(port, RETRO_DEVICE_KEYBOARD);
  std::string devLightgun = Libretro::Input::GetQualifiedName(port, RETRO_DEVICE_LIGHTGUN);
#endif
  auto& si = Core::System::GetInstance().GetSerialInterface();

  if (device == RETRO_DEVICE_GC_ON_WII) // Disconnect Wii device if we're using GC controller as device type to avoid conflict
    WiimoteCommon::OnSourceChanged(port, WiimoteSource::None);

  Core::System& system = Core::System::GetInstance();

  const int si_port = port > 3 ? port - 4 : port;
  SerialInterface::SIDevices si_device =
      system.IsTriforce() ? SerialInterface::SIDEVICE_AM_BASEBOARD :
                            SerialInterface::SIDEVICE_GC_CONTROLLER;
  if (device == RETRO_DEVICE_GBA_LINK)
  {
    // SIDEVICE_GC_GBA, not SIDEVICE_GC_GBA_EMULATED. The emulated one runs a Game
    // Boy Advance inside this process, which is the arrangement that cannot
    // compose:
    // it has no screen anybody can see and no way to be a machine in a room. This
    // one reaches an emulator running BESIDE this one, which is a console a
    // player can pick up.
    si_device = SerialInterface::SIDEVICE_GC_GBA;
  }

  Config::SetBaseOrCurrent(Config::GetInfoForSIDevice(si_port), si_device);
  si.ChangeDevice(Config::Get(Config::GetInfoForSIDevice(si_port)), si_port);

  if (device == RETRO_DEVICE_GBA_LINK)
  {
    // Nothing else to set up: a GBA on the end of a lead has its own buttons on
    // its own machine, so this port has no pad to configure.
    return;
  }

  GCPad* gcPad = (GCPad*)Pad::GetConfig()->GetController(port > 3 ? port - 4 : port);
  // load an empty inifile section, clears everything
  Common::IniFile::Section sec;
  gcPad->LoadConfig(&sec);
  gcPad->SetDefaultDevice(devJoypad);

  ControllerEmu::ControlGroup* gcButtons = gcPad->GetGroup(PadGroup::Buttons);
  ControllerEmu::ControlGroup* gcMainStick = gcPad->GetGroup(PadGroup::MainStick);
  ControllerEmu::ControlGroup* gcCStick = gcPad->GetGroup(PadGroup::CStick);
  ControllerEmu::ControlGroup* gcDPad = gcPad->GetGroup(PadGroup::DPad);
  ControllerEmu::ControlGroup* gcTriggers = gcPad->GetGroup(PadGroup::Triggers);
  ControllerEmu::ControlGroup* gcRumble = gcPad->GetGroup(PadGroup::Rumble);
  ControllerEmu::ControlGroup* gcOptions = gcPad->GetGroup(PadGroup::Options);
  ControllerEmu::ControlGroup* gcTriforce = gcPad->GetGroup(PadGroup::Triforce);

  if (Libretro::Input::g_has_microphone_support)
  {
    WiimoteUpdateFlags f;
    f.gcMicBtn = true;
    Libretro::Input::UpdateGCMappings(f, port, device);
  }

  gcButtons->SetControlExpression(0, "A");                          // A
  gcButtons->SetControlExpression(1, "B");                          // B
  gcButtons->SetControlExpression(2, "X");                          // X
  gcButtons->SetControlExpression(3, "Y");                          // Y
  gcButtons->SetControlExpression(4, "R");                          // Z
  gcButtons->SetControlExpression(5, "Start");                      // Start
  gcMainStick->SetControlExpression(0, "`" + devAnalog + ":Y0-`");  // Up
  gcMainStick->SetControlExpression(1, "`" + devAnalog + ":Y0+`");  // Down
  gcMainStick->SetControlExpression(2, "`" + devAnalog + ":X0-`");  // Left
  gcMainStick->SetControlExpression(3, "`" + devAnalog + ":X0+`");  // Right
  gcCStick->SetControlExpression(0, "`" + devAnalog + ":Y1-`");     // Up
  gcCStick->SetControlExpression(1, "`" + devAnalog + ":Y1+`");     // Down
  gcCStick->SetControlExpression(2, "`" + devAnalog + ":X1-`");     // Left
  gcCStick->SetControlExpression(3, "`" + devAnalog + ":X1+`");     // Right
  gcDPad->SetControlExpression(0, "Up");                            // Up
  gcDPad->SetControlExpression(1, "Down");                          // Down
  gcDPad->SetControlExpression(2, "Left");                          // Left
  gcDPad->SetControlExpression(3, "Right");                         // Right
  gcTriggers->SetControlExpression(0, "`" + devAnalog + ":Trigger0+`|(L2&!`" + devAnalog +
                                      ":Trigger0+`)");  // L-trigger Full Press
  gcTriggers->SetControlExpression(1, "`" + devAnalog + ":Trigger1+`|(R2&!`" + devAnalog +
                                      ":Trigger1+`)");  // R-trigger Full Press
  gcTriggers->SetControlExpression(2,
                                   "`" + devAnalog + ":Trigger0+`|L3");  // L-trigger Soft Press
  gcTriggers->SetControlExpression(3,
                                   "`" + devAnalog + ":Trigger1+`|R3");  // R-trigger Soft Press
  gcTriforce->SetControlExpression(0, "L");  // Test
  gcTriforce->SetControlExpression(1, "L3&R3"); // Service
  gcTriforce->SetControlExpression(2, "Select"); // Coin

  bool enableRumble = Libretro::Options::GetCached<bool>(Libretro::Options::sysconf::ENABLE_RUMBLE);
  if (enableRumble)
    gcRumble->SetControlExpression(0, "Rumble");
  static_cast<ControllerEmu::NumericSetting<bool>*>(gcOptions->numeric_settings[0].get())
    ->SetValue(true); // Always Connected

  gcPad->UpdateReferences(g_controller_interface);

  bool saveOrLoad = Libretro::Options::GetCached<bool>(Libretro::Options::retroarch_core::SAVE_LOAD_SETTINGS);
  if (!saveOrLoad)
    return Pad::GetConfig()->SaveConfig();
}

void retro_set_controller_port_device_wii(unsigned port, unsigned device)
{
  std::string devJoypad = Libretro::Input::GetQualifiedName(port, RETRO_DEVICE_JOYPAD);
  std::string devAnalog = Libretro::Input::GetQualifiedName(port, RETRO_DEVICE_ANALOG);
  std::string devMouse = Libretro::Input::GetQualifiedName(port, RETRO_DEVICE_MOUSE);
  std::string devPointer = Libretro::Input::GetQualifiedName(port, RETRO_DEVICE_POINTER);
#if 0
  std::string devKeyboard = Libretro::Input::GetQualifiedName(port, RETRO_DEVICE_KEYBOARD);
  std::string devLightgun = Libretro::Input::GetQualifiedName(port, RETRO_DEVICE_LIGHTGUN);
#endif
  auto& si = Core::System::GetInstance().GetSerialInterface();

  // Take the dongle off the id and remember it separately, so every branch below
  // only ever sees the four remotes it was written for. The flag is applied once,
  // beside the extension selection it belongs with.
  const bool wantMotionPlus = wiimote_has_motion_plus(device);
  device = wiimote_base_device(device);

  if (Wiimote::GetConfig()->ControllersNeedToBeCreated())
  {
    WARN_LOG_FMT(COMMON, "No controllers have been created yet");
    return;
  }

  bool altGCPorts = Libretro::Options::GetCached<bool>(Libretro::Options::sysconf::ALT_GC_PORTS_ON_WII);
  if (!altGCPorts) // Disconnect GC controller to avoid conflict with Wii device
  {
    Config::SetBaseOrCurrent(Config::GetInfoForSIDevice(port), SerialInterface::SIDEVICE_NONE);
    si.ChangeDevice(Config::Get(Config::GetInfoForSIDevice(port)), port);
  }

  WiimoteEmu::Wiimote* wm = load_saved_controller_config(port, device);

  // defaults already found, so don't overwrite
  if (wm)
    return;

  // nothing found — start fresh and apply hard-coded defaults
  wm = (WiimoteEmu::Wiimote*)Wiimote::GetConfig()->GetController(port);

  // load an empty inifile section, clears everything
  Common::IniFile::Section sec;
  wm->LoadConfig(&sec);
  wm->SetDefaultDevice(devJoypad);

  WiimoteUpdateFlags f;

  using namespace WiimoteEmu;
  if (device == RETRO_DEVICE_WIIMOTE_CC || device == RETRO_DEVICE_WIIMOTE_CC_PRO)
  {
    ControllerEmu::ControlGroup* ccButtons = wm->GetClassicGroup(ClassicGroup::Buttons);
    ControllerEmu::ControlGroup* ccTriggers = wm->GetClassicGroup(ClassicGroup::Triggers);
    ControllerEmu::ControlGroup* ccDpad = wm->GetClassicGroup(ClassicGroup::DPad);
    ControllerEmu::ControlGroup* ccLeftStick = wm->GetClassicGroup(ClassicGroup::LeftStick);
    ControllerEmu::ControlGroup* ccRightStick = wm->GetClassicGroup(ClassicGroup::RightStick);

    ccButtons->SetControlExpression(0, "A");                               // A
    ccButtons->SetControlExpression(1, "B");                               // B
    ccButtons->SetControlExpression(2, "X");                               // X
    ccButtons->SetControlExpression(3, "Y");                               // Y
    ccButtons->SetControlExpression(6, "Select");                          // -
    ccButtons->SetControlExpression(7, "Start");                           // +
    ccButtons->SetControlExpression(8, "R3");                              // Home
    if (device == RETRO_DEVICE_WIIMOTE_CC)
    {
      ccButtons->SetControlExpression(4, "L");                               // ZL
      ccButtons->SetControlExpression(5, "R");                               // ZR
      ccTriggers->SetControlExpression(0, "`" + devAnalog + ":Trigger0+`");  // L-trigger
      ccTriggers->SetControlExpression(1, "`" + devAnalog + ":Trigger1+`");  // R-trigger
      ccTriggers->SetControlExpression(2, "`" + devAnalog + ":Trigger0+`");  // L-trigger Analog
      ccTriggers->SetControlExpression(3, "`" + devAnalog + ":Trigger1+`");  // R-trigger Analog
    }
    else // Classic Controller Pro doesn't have analog triggers and L/R should be swapped with ZL/ZR
    {
      ccButtons->SetControlExpression(4, "L2");                            // ZL
      ccButtons->SetControlExpression(5, "R2");                            // ZR
      ccTriggers->SetControlExpression(0, "L");                            // L
      ccTriggers->SetControlExpression(1, "R");                            // R
    }
    ccDpad->SetControlExpression(0, "Up");                                 // Up
    ccDpad->SetControlExpression(1, "Down");                               // Down
    ccDpad->SetControlExpression(2, "Left");                               // Left
    ccDpad->SetControlExpression(3, "Right");                              // Right
    ccLeftStick->SetControlExpression(0, "`" + devAnalog + ":Y0-`");       // Up
    ccLeftStick->SetControlExpression(1, "`" + devAnalog + ":Y0+`");       // Down
    ccLeftStick->SetControlExpression(2, "`" + devAnalog + ":X0-`");       // Left
    ccLeftStick->SetControlExpression(3, "`" + devAnalog + ":X0+`");       // Right
    ccRightStick->SetControlExpression(0, "`" + devAnalog + ":Y1-`");      // Up
    ccRightStick->SetControlExpression(1, "`" + devAnalog + ":Y1+`");      // Down
    ccRightStick->SetControlExpression(2, "`" + devAnalog + ":X1-`");      // Left
    ccRightStick->SetControlExpression(3, "`" + devAnalog + ":X1+`");      // Right
  }
  else if (device != RETRO_DEVICE_REAL_WIIMOTE)
  {
    ControllerEmu::ControlGroup* wmButtons = wm->GetWiimoteGroup(WiimoteGroup::Buttons);
    ControllerEmu::ControlGroup* wmDPad = wm->GetWiimoteGroup(WiimoteGroup::DPad);
    ControllerEmu::ControlGroup* wmShake = wm->GetWiimoteGroup(WiimoteGroup::Shake);
    ControllerEmu::ControlGroup* wmTilt = wm->GetWiimoteGroup(WiimoteGroup::Tilt);

    f.irOffset = true;
    f.irYaw = true;
    f.irPitch = true;
    f.irDeadzone = true;
    f.swingAngle = true;

    if (device == RETRO_DEVICE_WIIMOTE_NC)
    {
      ControllerEmu::ControlGroup* ncButtons = wm->GetNunchukGroup(NunchukGroup::Buttons);
      ControllerEmu::ControlGroup* ncStick = wm->GetNunchukGroup(NunchukGroup::Stick);
      ControllerEmu::ControlGroup* ncShake = wm->GetNunchukGroup(NunchukGroup::Shake);
      // The Nunchuk's Tilt and Swing groups are deliberately left unbound. Its
      // motion arrives below as a real accelerometer, and there is no spare
      // stick to fall back on when one is not offered: the left stick is the
      // Nunchuk's own and the right one is the remote's tilt.
      ncButtons->SetControlExpression(0, "X");                             // C
      ncButtons->SetControlExpression(1, "Y");                             // Z
      ncStick->SetControlExpression(0, "`" + devAnalog + ":Y0-`");         // Up
      ncStick->SetControlExpression(1, "`" + devAnalog + ":Y0+`");         // Down
      ncStick->SetControlExpression(2, "`" + devAnalog + ":X0-`");         // Left
      ncStick->SetControlExpression(3, "`" + devAnalog + ":X0+`");         // Right
      ncShake->SetControlExpression(0, bindMouse("L2", devMouse + ":Middle"));  // Nunchuk shake X
      ncShake->SetControlExpression(1, bindMouse("L2", devMouse + ":Middle"));  // Nunchuk shake Y
      ncShake->SetControlExpression(2, bindMouse("L2", devMouse + ":Middle"));  // Nunchuk shake Z

      wmButtons->SetControlExpression(0, bindMouse("A", devMouse + ":Left"));   // A
      wmButtons->SetControlExpression(1, bindMouse("B", devMouse + ":Right"));  // B
      wmButtons->SetControlExpression(2, "Start");                         // 1
      wmButtons->SetControlExpression(3, "Select");                        // 2
      wmButtons->SetControlExpression(4, "L");                             // -
      wmButtons->SetControlExpression(5, "R");                             // +

      f.irMode = true;
    }
    else
    {
      if (device == RETRO_DEVICE_WIIMOTE)
      {
        wmButtons->SetControlExpression(0, bindMouse("A", devMouse + ":Left"));   // A
        wmButtons->SetControlExpression(1, bindMouse("B", devMouse + ":Right"));  // B
        wmButtons->SetControlExpression(2, "X");                             // 1
        wmButtons->SetControlExpression(3, "Y");                             // 2
      }
      else
      {
        wmButtons->SetControlExpression(0, "X");  // A
        wmButtons->SetControlExpression(1, "Y");  // B
        wmButtons->SetControlExpression(2, "B");  // 1
        wmButtons->SetControlExpression(3, "A");  // 2
      }

      wmButtons->SetControlExpression(4, "Select");                // -
      wmButtons->SetControlExpression(5, "Start");                 // +
    }

    // Motion, for every remote configuration rather than only the ones with
    // nothing on the expansion port. This used to sit inside the else above,
    // so seating a Nunchuk unbound the REMOTE's own accelerometer and the
    // MotionPlus gyroscope with it, and skipped the analog fallback too:
    // both hands went dead the moment the second one was plugged in.
    if (Libretro::Input::sensor_enabled[port][0][SENSOR_ACCELEROMETER] ||
        Libretro::Input::sensor_enabled[port][0][SENSOR_GYRO])
    {
      std::string devSensor = Libretro::Input::GetQualifiedNameSensor(port, 0);

      if (Libretro::Input::sensor_enabled[port][0][SENSOR_ACCELEROMETER])
      {
        // Accelerometer (6 inputs: Up, Down, Left, Right, Forward, Backward)
        // Indices must match WiimoteEmu::LoadDefaults ordering (0..5)
        auto* wmAccel = static_cast<ControllerEmu::IMUAccelerometer*>(
          wm->GetWiimoteGroup(WiimoteEmu::WiimoteGroup::IMUAccelerometer));
        if (wmAccel)
        {
          wmAccel->SetControlExpression(0, "`" + devSensor + ":AccelZ+`");  // Up
          wmAccel->SetControlExpression(1, "`" + devSensor + ":AccelZ-`");  // Down
          wmAccel->SetControlExpression(2, "`" + devSensor + ":AccelX+`");  // Left
          wmAccel->SetControlExpression(3, "`" + devSensor + ":AccelX-`");  // Right
          wmAccel->SetControlExpression(4, "`" + devSensor + ":AccelY-`");  // Forward
          wmAccel->SetControlExpression(5, "`" + devSensor + ":AccelY+`");  // Backward
        }
      }

      // A sibling of the accelerometer branch, not a child of it. Nested, a
      // frontend that offered gyro but no accelerometer bound neither.
      if (Libretro::Input::sensor_enabled[port][0][SENSOR_GYRO])
      {
        // Gyroscope (6 inputs: PitchUp/Down, RollLeft/Right, YawLeft/Right)
        auto* wmGyro = static_cast<ControllerEmu::IMUGyroscope*>(
          wm->GetWiimoteGroup(WiimoteEmu::WiimoteGroup::IMUGyroscope));
        if (wmGyro)
        {
          // Angular velocity about the remote's own axes, the same frame the
          // accelerometer uses: +X left, +Y back, +Z up, right-hand rule. That
          // fixes which direction each axis names: about +X the nose drops, so
          // +X is pitch DOWN; about +Y the top rolls left; about +Z the nose
          // swings left.
          //
          // GetRawState() reads these as [1]-[0], [2]-[3], [4]-[5], so pairing
          // them this way hands back exactly the signed value that arrived.
          wmGyro->SetControlExpression(0, "`" + devSensor + ":GyroX-`");  // Pitch Up
          wmGyro->SetControlExpression(1, "`" + devSensor + ":GyroX+`");  // Pitch Down
          wmGyro->SetControlExpression(2, "`" + devSensor + ":GyroY+`");  // Roll Left
          wmGyro->SetControlExpression(3, "`" + devSensor + ":GyroY-`");  // Roll Right
          wmGyro->SetControlExpression(4, "`" + devSensor + ":GyroZ+`");  // Yaw Left
          wmGyro->SetControlExpression(5, "`" + devSensor + ":GyroZ-`");  // Yaw Right
        }
      }
    }
    else
    {
      // No sensors on offer, so tilt falls back to a stick. With a Nunchuk
      // fitted the left one IS the Nunchuk's, so it moves to the right one,
      // which is what descWiimoteNunchuk has been advertising to frontends all
      // along with nothing behind it.
      const std::string tiltAxis = (device == RETRO_DEVICE_WIIMOTE_NC) ? "1" : "0";
      wmTilt->SetControlExpression(0, "`" + devAnalog + ":Y" + tiltAxis + "-`");  // Forward
      wmTilt->SetControlExpression(1, "`" + devAnalog + ":Y" + tiltAxis + "+`");  // Backward
      wmTilt->SetControlExpression(2, "`" + devAnalog + ":X" + tiltAxis + "-`");  // Left
      wmTilt->SetControlExpression(3, "`" + devAnalog + ":X" + tiltAxis + "+`");  // Right
    }

    // And the Nunchuk's own accelerometer, on the sub-device the frontend was
    // asked for. Purely additive: Nunchuk::BuildDesiredExtensionState composes
    // this group with swing, tilt and shake, and substitutes a device lying
    // flat when nothing is bound, so the L2 shake and every gamepad player are
    // exactly as they were.
    if (device == RETRO_DEVICE_WIIMOTE_NC &&
        Libretro::Input::sensor_enabled[port][NUNCHUK_SENSOR_INDEX][SENSOR_ACCELEROMETER])
    {
      auto* ncAccel = static_cast<ControllerEmu::IMUAccelerometer*>(
        wm->GetNunchukGroup(NunchukGroup::IMUAccelerometer));
      if (ncAccel)
      {
        // Same six-index order and the same axes as the remote's above: the
        // Nunchuk reports in the identical frame, +X left, +Y back, +Z up.
        const std::string devNunchuk =
            Libretro::Input::GetQualifiedNameSensor(port, NUNCHUK_SENSOR_INDEX);
        ncAccel->SetControlExpression(0, "`" + devNunchuk + ":AccelZ+`");  // Up
        ncAccel->SetControlExpression(1, "`" + devNunchuk + ":AccelZ-`");  // Down
        ncAccel->SetControlExpression(2, "`" + devNunchuk + ":AccelX+`");  // Left
        ncAccel->SetControlExpression(3, "`" + devNunchuk + ":AccelX-`");  // Right
        ncAccel->SetControlExpression(4, "`" + devNunchuk + ":AccelY-`");  // Forward
        ncAccel->SetControlExpression(5, "`" + devNunchuk + ":AccelY+`");  // Backward
      }
    }

    wmButtons->SetControlExpression(6, "R3");  // Home
    wmDPad->SetControlExpression(0, "Up");     // Up
    wmDPad->SetControlExpression(1, "Down");   // Down
    wmDPad->SetControlExpression(2, "Left");   // Left
    wmDPad->SetControlExpression(3, "Right");  // Right

    f.irModifier = true;
    f.swingModifier = true;
    f.sideways = true;
    // Raised at setup so the option takes effect on a cold boot. Without it the
    // binding would only ever happen if the value CHANGED while running, which
    // is the trap the IR offset/yaw/pitch settings already sit in, where their
    // declared defaults never reach the emulated remote at all.
    f.irPassthrough = true;
    Libretro::Input::UpdateWiimoteMappings(f, port, device);

    // R2, which is what every Wiimote descriptor advertises as "Shake Wiimote".
    // This listened on L2, so the remote's shake button did nothing at all, and
    // with a Nunchuk fitted L2 shook both the remote and the Nunchuk at once.
    wmShake->SetControlExpression(0, bindMouse("R2", devMouse + ":Middle"));  // Wiimote shake X
    wmShake->SetControlExpression(1, bindMouse("R2", devMouse + ":Middle"));  // Wiimote shake Y
    wmShake->SetControlExpression(2, bindMouse("R2", devMouse + ":Middle"));  // Wiimote shake Z
  }

  ControllerEmu::ControlGroup* wmOptions = wm->GetWiimoteGroup(WiimoteGroup::Options);
  ControllerEmu::Attachments* wmExtension =
      (ControllerEmu::Attachments*)wm->GetWiimoteGroup(WiimoteGroup::Attachments);

  // Fit or remove the dongle. Index 0 is "Attach MotionPlus", the only entry in
  // this group's numeric_settings. The attachment SELECTOR is deliberately kept
  // out of that list (Attachments.h), so it cannot be what gets written here.
  //
  // Dolphin defaults this to true for every remote, so it has to be written on
  // BOTH paths rather than only when the dongle is wanted: a port re-announced
  // without one would otherwise keep whatever the last remote left behind.
  if (!wmExtension->numeric_settings.empty())
    static_cast<ControllerEmu::NumericSetting<bool>*>(wmExtension->numeric_settings[0].get())
        ->SetValue(wantMotionPlus);

  static_cast<ControllerEmu::NumericSetting<double>*>(wmOptions->numeric_settings[0].get())
      ->SetValue(0);  // Speaker Pan [-100, 100]
  static_cast<ControllerEmu::NumericSetting<double>*>(wmOptions->numeric_settings[1].get())
      ->SetValue(95);  // Battery [0, 100]
  static_cast<ControllerEmu::NumericSetting<bool>*>(wmOptions->numeric_settings[2].get())
      ->SetValue(false);  // Upright Wiimote
  static_cast<ControllerEmu::NumericSetting<bool>*>(wmOptions->numeric_settings[3].get())
      ->SetValue(false);  // Sideways Wiimote

  ControllerEmu::ControlGroup* wmRumble = wm->GetWiimoteGroup(WiimoteEmu::WiimoteGroup::Rumble);
  bool enableRumble = Libretro::Options::GetCached<bool>(Libretro::Options::sysconf::ENABLE_RUMBLE);
  if (enableRumble)
    wmRumble->SetControlExpression(0, "Rumble");

  switch (device)
  {
  case RETRO_DEVICE_WIIMOTE:
    wmExtension->SetSelectedAttachment(ExtensionNumber::NONE);
    Config::SetBaseOrCurrent(Config::GetInfoForWiimoteSource(port), WiimoteSource::Emulated);
    WiimoteCommon::OnSourceChanged(port, WiimoteSource::Emulated);
    break;

  case RETRO_DEVICE_WIIMOTE_SW:
    wmExtension->SetSelectedAttachment(ExtensionNumber::NONE);
    static_cast<ControllerEmu::NumericSetting<bool>*>(wmOptions->numeric_settings[3].get())
      ->SetValue(true);  // Sideways Wiimote
    Config::SetBaseOrCurrent(Config::GetInfoForWiimoteSource(port), WiimoteSource::Emulated);
    WiimoteCommon::OnSourceChanged(port, WiimoteSource::Emulated);
    break;

  case RETRO_DEVICE_WIIMOTE_NC:
    wmExtension->SetSelectedAttachment(ExtensionNumber::NUNCHUK);
    Config::SetBaseOrCurrent(Config::GetInfoForWiimoteSource(port), WiimoteSource::Emulated);
    WiimoteCommon::OnSourceChanged(port, WiimoteSource::Emulated);
    break;

  case RETRO_DEVICE_WIIMOTE_CC:
  case RETRO_DEVICE_WIIMOTE_CC_PRO:
    wmExtension->SetSelectedAttachment(ExtensionNumber::CLASSIC);
    Config::SetBaseOrCurrent(Config::GetInfoForWiimoteSource(port), WiimoteSource::Emulated);
    WiimoteCommon::OnSourceChanged(port, WiimoteSource::Emulated);
    break;

  case RETRO_DEVICE_REAL_WIIMOTE:
    Config::SetBaseOrCurrent(Config::GetInfoForWiimoteSource(port), WiimoteSource::Real);
    WiimoteCommon::OnSourceChanged(port, WiimoteSource::Real);
    break;
  }

  wm->UpdateReferences(g_controller_interface);
  bool saveOrLoad = Libretro::Options::GetCached<bool>(Libretro::Options::retroarch_core::SAVE_LOAD_SETTINGS);
  if (!saveOrLoad)
    ::Wiimote::GetConfig()->SaveConfig();
}

void poll_microphone()
{
  static bool s_wii_speak_enabled = false;
  static bool s_logi_microphone_enabled = false;
  static bool s_gc_mic_enabled = false;

  if (Libretro::Options::IsUpdated(Libretro::Options::sysconf::WII_SPEAK_ENABLE))
    s_wii_speak_enabled = Libretro::Options::GetCached<bool>(Libretro::Options::sysconf::WII_SPEAK_ENABLE);

  if (Libretro::Options::IsUpdated(Libretro::Options::sysconf::WII_LOGI_MICROPHONE_ENABLE))
    s_logi_microphone_enabled = Libretro::Options::GetCached<bool>(Libretro::Options::sysconf::WII_LOGI_MICROPHONE_ENABLE);

  if (Libretro::Options::IsUpdated(Libretro::Options::sysconf_gc::ENABLE_GAMECUBE_MIC))
    s_gc_mic_enabled = Libretro::Options::GetCached<bool>(Libretro::Options::sysconf_gc::ENABLE_GAMECUBE_MIC);

  Core::System& system = Core::System::GetInstance();

  if (system.IsWii() && (s_wii_speak_enabled || s_logi_microphone_enabled))
  {
    for (auto* mic : Libretro::Input::g_active_microphones)
      mic->PollRetroArchMic();
  }
  else if(s_gc_mic_enabled)
  {
    // Poll GC mic
    auto* exi_device = system.GetExpansionInterface().GetDevice(ExpansionInterface::Slot::B);
    if (exi_device && Config::Get(Config::GetInfoForEXIDevice(ExpansionInterface::Slot::B))
      == ExpansionInterface::EXIDeviceType::Microphone)
    {
      auto* gc_mic = static_cast<ExpansionInterface::CEXIMic*>(exi_device);
      gc_mic->PollLibretroMic();
    }
  }
}
