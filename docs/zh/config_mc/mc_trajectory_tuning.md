# 多旋翼设定值调整（轨迹生成器）

This document provides an overview of the multicopter tuning parameters that change the _user experience_: how fast the vehicle reacts to stick movements or direction changes in missions, the maximum allowed velocity, etc.

In other words, this topic explains how to tune the parameters that affect the value of a _desired setpoint_ rather than those that affect how well the vehicle _tracks_ the setpoint).

生成这些设定点的算法称为“轨迹生成器”。

:::warning
This guide is for advanced users/experts.
:::

:::tip
Follow the instructions in the [Multicopter PID Tuning Guide](../config_mc/pid_tuning_guide_multicopter.md) _before_ doing any of the tuning described here.
请不要试图通过调整这些参数来修复错误的轨迹或抖动。
:::

## 综述

The input to the P/PID controller is a _desired setpoint_ that the vehicle should attempt to track.
[PID Tuning](../config_mc/pid_tuning_guide_multicopter.md) ("Lower level tuning") aims to reduce the error between the desired setpoint and the estimate of the vehicle state.

The _desired setpoint_ passed to the P/PID controller is itself calculated from a _demanded setpoint_ based on a stick position (in RC modes) or from a mission command.
要求设定值可能会改变得很快（例如，如果用户“一下子”将摇杆从零移动到最大值）。
如果缓慢调整相应的目标设定值, 飞行器的特性就会更好。

_Setpoint value tuning_ ("higher level tuning") is used to specify the mapping between the _demanded_ and the _desired_ setpoints - i.e. defining the "ramp" at which the desired setpoint follows the demanded setpoint.

:::tip
Poorly tuned [P/PID Gains](../config_mc/pid_tuning_guide_multicopter.md) can lead to instability.
Poorly tuned _setpoint values_ cannot result in instability, but may result in either very jerky or very unresponsive reactions to setpoint changes.
:::

<a id="modes"></a>

## 飞行模式轨迹支持

[Mission mode](../flight_modes_mc/mission.md) used the [Jerk-limited](../config_mc/mc_jerk_limited_type_trajectory.md) trajectory all the time.

[Position mode](../flight_modes_mc/position.md) supports the [implementations](#position-mode-implementations) listed below.
It uses the acceleration based mapping by default; other types can be set using [MPC_POS_MODE](../advanced_config/parameter_reference.md#MPC_POS_MODE).

[Altitude mode](../flight_modes_mc/altitude.md) similarly supports the [implementations](#altitude-mode-implementations) selected by [MPC_POS_MODE](../advanced_config/parameter_reference.md#MPC_POS_MODE), but _only_ for smoothing the vertical component (i.e. when controlling the altitude).

其他模式不支持轨迹调整。

## Position Mode Implementations

The following list provides an _overview_ of the different implementations of how the stick input is interpreted and turned into trajectory setpoints:

- Acceleration based (Default)
  - Horizontal stick input mapped to acceleration setpoints.
  - Intuitive stick feel because it's like pushing the vehicle around.
  - No unexpected tilt changes upon reaching travel speed velocity.
  - Vertical stick input mapped with jerk-limited trajectory.
  - Set in position mode using `MPC_POS_MODE=Acceleration based`.
- [Jerk-limited](../config_mc/mc_jerk_limited_type_trajectory.md)
  - Used when smooth motion is required (e.g.: filming, mapping, cargo).
  - Generates symmetric smooth S-curves where the jerk and acceleration limits are always guaranteed.
  - May not be suitable for vehicles/use-cases that require a faster response - e.g. race quads.
  - Set in position mode using `MPC_POS_MODE=Smoothed velocity`.
- **Simple position control**
  - Sticks map directly to velocity setpoints without smoothing.
  - Useful for velocity control tuning.
  - Set in position mode using `MPC_POS_MODE=Direct velocity`.

## Altitude Mode Implementations

Analogously to [position mode implementations](#position-mode-implementations) these are the implementations for interpreting vertical stick input:

- [Jerk-limited](../config_mc/mc_jerk_limited_type_trajectory.md)
  - Smoothed vertical input.
  - Set in altitude mode with `MPC_POS_MODE` Smoothed velocity or Acceleration based.
- **Simple altitude control**
  - Unsmoothed vertical input.
  - Set in altitude mode only when using `MPC_POS_MODE=Direct velocity`.
