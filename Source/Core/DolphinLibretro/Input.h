#pragma once

#include <string>

#include <libretro.h>
#include "Common/WindowSystemInfo.h"
#include "DolphinLibretro/SensorIndex.h"

// only 4 support sensors, but the array may be upto 8 if connected GC controllers
#define NUM_CONTROLLERS_FOR_SENSORS 8

// Sensors addressable on one port. Index 0 is the controller itself; index 1 is
// whatever is plugged into it, which for a Wii Remote is the Nunchuk and its own
// accelerometer. A player is one port, so the pair cannot be spread over two.
#define NUM_SENSOR_SUB_DEVICES 2

struct WiimoteUpdateFlags
{
    bool irMode        = false;
    bool irOffset      = false; // center
    bool irYaw         = false; // width
    bool irPitch       = false; // height
    bool irDeadzone    = false;
    bool irModifier    = false;
    bool swingModifier = false;
    bool swingAngle    = false;
    bool sideways      = false;
    bool rumble        = false;
    bool gcMicBtn      = false;
    bool irPassthrough = false;

    bool any() const {
        return irMode || irOffset || irYaw || irPitch ||
               irDeadzone || irModifier || swingModifier ||
               swingAngle || sideways || rumble || gcMicBtn || irPassthrough;
    }
};

void refresh_all_wiimote_flags(unsigned port, unsigned device);
void poll_microphone();

namespace Libretro
{
namespace Input
{
constexpr std::string_view source = "Libretro";
extern double g_accel_pos[NUM_CONTROLLERS_FOR_SENSORS][NUM_SENSOR_SUB_DEVICES][3];
extern double g_accel_neg[NUM_CONTROLLERS_FOR_SENSORS][NUM_SENSOR_SUB_DEVICES][3];
extern double g_gyro_pos[NUM_CONTROLLERS_FOR_SENSORS][NUM_SENSOR_SUB_DEVICES][3];
extern double g_gyro_neg[NUM_CONTROLLERS_FOR_SENSORS][NUM_SENSOR_SUB_DEVICES][3];

static retro_sensor_interface sensor_interface = {0};

/// The ciface device name for one port's sub-device. Index 0 keeps the plain
/// "Sensor" it has always had, so every control expression written before
/// sub-devices existed still resolves to exactly the device it always did.
inline std::string SensorDeviceName(unsigned index)
{
  return index == 0 ? std::string("Sensor") : "Sensor" + std::to_string(index);
}

void Init(const WindowSystemInfo& wsi);
void InitStage2();
void InitSensors();
void UpdateAccelerometer(unsigned port, unsigned index);
void UpdateGyro(unsigned port);
void Update();
void Shutdown();
void ResetControllers(const WiimoteUpdateFlags& f);
void BluetoothPassthroughBind();
void UpdateWiimoteMappings(const WiimoteUpdateFlags& f, unsigned port, unsigned device);
void UpdateGCMappings(const WiimoteUpdateFlags& f, unsigned port, unsigned device);
} // namespace Input
} // namespace Libretro

class SensorDevice : public ciface::Core::Device
{
public:
  /// `index` is the sub-device on this port, matching the one the frontend is
  /// asked for. Both are needed: the port picks the player, the index picks
  /// which of that player's sensors this device publishes.
  SensorDevice(unsigned port, unsigned index)
      : m_port(port), m_index(index), m_name(Libretro::Input::SensorDeviceName(index))
  {
  }

  std::string GetName() const override { return m_name; }
  std::string GetSource() const override { return std::string(Libretro::Input::source); }
  unsigned int GetPort() const { return m_port; }
  ciface::Core::DeviceRemoval UpdateInput() override { return ciface::Core::DeviceRemoval::Keep; }

private:
  class ScalarInput : public ciface::Core::Device::Input
  {
  public:
    ScalarInput(const char* name, const double* slot) : m_name(name), m_slot(slot) {}
    std::string GetName() const override { return m_name; }
    ControlState GetState() const override { return *m_slot; }
  private:
    const char* m_name;
    const double* m_slot;
  };

public:
  /// Every axis is published as a PAIR of one-sided inputs, never as one signed
  /// input. ControlExpression::GetValueIgnoringSuppression() clamps a control to
  /// >= 0 (ExpressionParser.cpp: "We clamp off the negative values here"), so a
  /// lone signed input loses half its travel outright. The IMU groups are built
  /// for exactly this shape, each one subtracting one direction's control from
  /// the other's, so handing them a matched +/- pair reconstructs the signed
  /// value the sensor actually reported.
  void RegisterAll()
  {
    AddInput(new ScalarInput("GyroX+", &Libretro::Input::g_gyro_pos[m_port][m_index][0]));
    AddInput(new ScalarInput("GyroX-", &Libretro::Input::g_gyro_neg[m_port][m_index][0]));
    AddInput(new ScalarInput("GyroY+", &Libretro::Input::g_gyro_pos[m_port][m_index][1]));
    AddInput(new ScalarInput("GyroY-", &Libretro::Input::g_gyro_neg[m_port][m_index][1]));
    AddInput(new ScalarInput("GyroZ+", &Libretro::Input::g_gyro_pos[m_port][m_index][2]));
    AddInput(new ScalarInput("GyroZ-", &Libretro::Input::g_gyro_neg[m_port][m_index][2]));
    AddInput(new ScalarInput("AccelX+", &Libretro::Input::g_accel_pos[m_port][m_index][0]));
    AddInput(new ScalarInput("AccelX-", &Libretro::Input::g_accel_neg[m_port][m_index][0]));
    AddInput(new ScalarInput("AccelY+", &Libretro::Input::g_accel_pos[m_port][m_index][1]));
    AddInput(new ScalarInput("AccelY-", &Libretro::Input::g_accel_neg[m_port][m_index][1]));
    AddInput(new ScalarInput("AccelZ+", &Libretro::Input::g_accel_pos[m_port][m_index][2]));
    AddInput(new ScalarInput("AccelZ-", &Libretro::Input::g_accel_neg[m_port][m_index][2]));
  }

private:
  unsigned m_port;
  unsigned m_index;
  std::string m_name;
};

class GyroDevice : public ciface::Core::Device
{
private:
  class GyroAxis : public ciface::Core::Device::Input
  {
  public:
    enum Axis { PITCH, ROLL, YAW };

    GyroAxis(unsigned port, Axis axis, const char* name)
        : m_port(port), m_axis(axis), m_name(name) {}

    std::string GetName() const override { return m_name; }

    ControlState GetState() const override
    {
      if (!Libretro::Input::sensor_interface.get_sensor_input)
        return 0.0;

      switch (m_axis)
      {
      case PITCH:
        return Libretro::Input::sensor_interface.get_sensor_input(m_port, RETRO_SENSOR_GYROSCOPE_Y);
      case ROLL:
        return Libretro::Input::sensor_interface.get_sensor_input(m_port, RETRO_SENSOR_GYROSCOPE_X);
      case YAW:
        return Libretro::Input::sensor_interface.get_sensor_input(m_port, RETRO_SENSOR_GYROSCOPE_Z);
      }
      return 0.0;
    }

  private:
    const unsigned m_port;
    const Axis m_axis;
    const char* m_name;
  };

public:
  GyroDevice(unsigned port)
  {
    AddInput(new GyroAxis(port, GyroAxis::PITCH, "Pitch"));
    AddInput(new GyroAxis(port, GyroAxis::ROLL, "Roll"));
    AddInput(new GyroAxis(port, GyroAxis::YAW, "Yaw"));
  }

  std::string GetName() const override { return "Gyroscope"; }
  std::string GetSource() const override { return std::string(Libretro::Input::source); }
};
