# dds_topics.yaml — PX4 Topics Exposed to ROS 2

::: info
This document is [auto-generated](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/msg/generate_msg_docs.py) from the source code.
:::

The [dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml) file specifies which uORB message definitions are compiled into the [uxrce_dds_client](../modules/modules_system.md#uxrce-dds-client) module when [PX4 is built](../middleware/uxrce_dds.md#code-generation), and hence which topics are available for ROS 2 applications to subscribe or publish (by default).

This document shows a markdown-rendered version of [dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml), listing the publications, subscriptions, and so on.

## Publications

| Topic                                    | Type                                                                                   | Rate Limit |
| ---------------------------------------- | -------------------------------------------------------------------------------------- | ---------- |
| `/fmu/out/register_ext_component_reply`  | [px4_msgs::msg::RegisterExtComponentReply](../msg_docs/RegisterExtComponentReply.md)   |
| `/fmu/out/arming_check_request`          | [px4_msgs::msg::ArmingCheckRequest](../msg_docs/ArmingCheckRequest.md)                 | 5.0        |
| `/fmu/out/mode_completed`                | [px4_msgs::msg::ModeCompleted](../msg_docs/ModeCompleted.md)                           | 50.0       |
| `/fmu/out/battery_status`                | [px4_msgs::msg::BatteryStatus](../msg_docs/BatteryStatus.md)                           | 1.0        |
| `/fmu/out/collision_constraints`         | [px4_msgs::msg::CollisionConstraints](../msg_docs/CollisionConstraints.md)             | 50.0       |
| `/fmu/out/estimator_status_flags`        | [px4_msgs::msg::EstimatorStatusFlags](../msg_docs/EstimatorStatusFlags.md)             | 5.0        |
| `/fmu/out/failsafe_flags`                | [px4_msgs::msg::FailsafeFlags](../msg_docs/FailsafeFlags.md)                           | 5.0        |
| `/fmu/out/manual_control_setpoint`       | [px4_msgs::msg::ManualControlSetpoint](../msg_docs/ManualControlSetpoint.md)           | 25.0       |
| `/fmu/out/message_format_response`       | [px4_msgs::msg::MessageFormatResponse](../msg_docs/MessageFormatResponse.md)           |
| `/fmu/out/position_setpoint_triplet`     | [px4_msgs::msg::PositionSetpointTriplet](../msg_docs/PositionSetpointTriplet.md)       | 5.0        |
| `/fmu/out/sensor_combined`               | [px4_msgs::msg::SensorCombined](../msg_docs/SensorCombined.md)                         |
| `/fmu/out/timesync_status`               | [px4_msgs::msg::TimesyncStatus](../msg_docs/TimesyncStatus.md)                         | 10.0       |
| `/fmu/out/transponder_report`            | [px4_msgs::msg::TransponderReport](../msg_docs/TransponderReport.md)                   |
| `/fmu/out/vehicle_land_detected`         | [px4_msgs::msg::VehicleLandDetected](../msg_docs/VehicleLandDetected.md)               | 5.0        |
| `/fmu/out/vehicle_attitude`              | [px4_msgs::msg::VehicleAttitude](../msg_docs/VehicleAttitude.md)                       | 50.0       |
| `/fmu/out/vehicle_control_mode`          | [px4_msgs::msg::VehicleControlMode](../msg_docs/VehicleControlMode.md)                 | 50.0       |
| `/fmu/out/vehicle_command_ack`           | [px4_msgs::msg::VehicleCommandAck](../msg_docs/VehicleCommandAck.md)                   |
| `/fmu/out/vehicle_global_position`       | [px4_msgs::msg::VehicleGlobalPosition](../msg_docs/VehicleGlobalPosition.md)           | 50.0       |
| `/fmu/out/vehicle_gps_position`          | [px4_msgs::msg::SensorGps](../msg_docs/SensorGps.md)                                   | 50.0       |
| `/fmu/out/vehicle_local_position`        | [px4_msgs::msg::VehicleLocalPosition](../msg_docs/VehicleLocalPosition.md)             | 50.0       |
| `/fmu/out/vehicle_odometry`              | [px4_msgs::msg::VehicleOdometry](../msg_docs/VehicleOdometry.md)                       | 100.0      |
| `/fmu/out/vehicle_status`                | [px4_msgs::msg::VehicleStatus](../msg_docs/VehicleStatus.md)                           | 5.0        |
| `/fmu/out/airspeed_validated`            | [px4_msgs::msg::AirspeedValidated](../msg_docs/AirspeedValidated.md)                   | 50.0       |
| `/fmu/out/vtol_vehicle_status`           | [px4_msgs::msg::VtolVehicleStatus](../msg_docs/VtolVehicleStatus.md)                   |
| `/fmu/out/home_position`                 | [px4_msgs::msg::HomePosition](../msg_docs/HomePosition.md)                             | 5.0        |
| `/fmu/out/wind`                          | [px4_msgs::msg::Wind](../msg_docs/Wind.md)                                             | 1.0        |
| `/fmu/out/gimbal_device_attitude_status` | [px4_msgs::msg::GimbalDeviceAttitudeStatus](../msg_docs/GimbalDeviceAttitudeStatus.md) | 20.0       |

## Subscriptions

| Topic                                      | Type                                                                                               |
| ------------------------------------------ | -------------------------------------------------------------------------------------------------- |
| /fmu/in/register_ext_component_request     | [px4_msgs::msg::RegisterExtComponentRequest](../msg_docs/RegisterExtComponentRequest.md)           |
| /fmu/in/unregister_ext_component           | [px4_msgs::msg::UnregisterExtComponent](../msg_docs/UnregisterExtComponent.md)                     |
| /fmu/in/config_overrides_request           | [px4_msgs::msg::ConfigOverrides](../msg_docs/ConfigOverrides.md)                                   |
| /fmu/in/arming_check_reply                 | [px4_msgs::msg::ArmingCheckReply](../msg_docs/ArmingCheckReply.md)                                 |
| /fmu/in/message_format_request             | [px4_msgs::msg::MessageFormatRequest](../msg_docs/MessageFormatRequest.md)                         |
| /fmu/in/mode_completed                     | [px4_msgs::msg::ModeCompleted](../msg_docs/ModeCompleted.md)                                       |
| /fmu/in/config_control_setpoints           | [px4_msgs::msg::VehicleControlMode](../msg_docs/VehicleControlMode.md)                             |
| /fmu/in/distance_sensor                    | [px4_msgs::msg::DistanceSensor](../msg_docs/DistanceSensor.md)                                     |
| /fmu/in/manual_control_input               | [px4_msgs::msg::ManualControlSetpoint](../msg_docs/ManualControlSetpoint.md)                       |
| /fmu/in/offboard_control_mode              | [px4_msgs::msg::OffboardControlMode](../msg_docs/OffboardControlMode.md)                           |
| /fmu/in/onboard_computer_status            | [px4_msgs::msg::OnboardComputerStatus](../msg_docs/OnboardComputerStatus.md)                       |
| /fmu/in/obstacle_distance                  | [px4_msgs::msg::ObstacleDistance](../msg_docs/ObstacleDistance.md)                                 |
| /fmu/in/sensor_optical_flow                | [px4_msgs::msg::SensorOpticalFlow](../msg_docs/SensorOpticalFlow.md)                               |
| /fmu/in/goto_setpoint                      | [px4_msgs::msg::GotoSetpoint](../msg_docs/GotoSetpoint.md)                                         |
| /fmu/in/telemetry_status                   | [px4_msgs::msg::TelemetryStatus](../msg_docs/TelemetryStatus.md)                                   |
| /fmu/in/trajectory_setpoint                | [px4_msgs::msg::TrajectorySetpoint](../msg_docs/TrajectorySetpoint.md)                             |
| /fmu/in/vehicle_attitude_setpoint          | [px4_msgs::msg::VehicleAttitudeSetpoint](../msg_docs/VehicleAttitudeSetpoint.md)                   |
| /fmu/in/vehicle_mocap_odometry             | [px4_msgs::msg::VehicleOdometry](../msg_docs/VehicleOdometry.md)                                   |
| /fmu/in/vehicle_rates_setpoint             | [px4_msgs::msg::VehicleRatesSetpoint](../msg_docs/VehicleRatesSetpoint.md)                         |
| /fmu/in/vehicle_visual_odometry            | [px4_msgs::msg::VehicleOdometry](../msg_docs/VehicleOdometry.md)                                   |
| /fmu/in/vehicle_command                    | [px4_msgs::msg::VehicleCommand](../msg_docs/VehicleCommand.md)                                     |
| /fmu/in/vehicle_command_mode_executor      | [px4_msgs::msg::VehicleCommand](../msg_docs/VehicleCommand.md)                                     |
| /fmu/in/vehicle_thrust_setpoint            | [px4_msgs::msg::VehicleThrustSetpoint](../msg_docs/VehicleThrustSetpoint.md)                       |
| /fmu/in/vehicle_torque_setpoint            | [px4_msgs::msg::VehicleTorqueSetpoint](../msg_docs/VehicleTorqueSetpoint.md)                       |
| /fmu/in/actuator_motors                    | [px4_msgs::msg::ActuatorMotors](../msg_docs/ActuatorMotors.md)                                     |
| /fmu/in/actuator_servos                    | [px4_msgs::msg::ActuatorServos](../msg_docs/ActuatorServos.md)                                     |
| /fmu/in/fixed_wing_longitudinal_setpoint   | [px4_msgs::msg::FixedWingLongitudinalSetpoint](../msg_docs/FixedWingLongitudinalSetpoint.md)       |
| /fmu/in/fixed_wing_lateral_setpoint        | [px4_msgs::msg::FixedWingLateralSetpoint](../msg_docs/FixedWingLateralSetpoint.md)                 |
| /fmu/in/longitudinal_control_configuration | [px4_msgs::msg::LongitudinalControlConfiguration](../msg_docs/LongitudinalControlConfiguration.md) |
| /fmu/in/lateral_control_configuration      | [px4_msgs::msg::LateralControlConfiguration](../msg_docs/LateralControlConfiguration.md)           |
| /fmu/in/rover_position_setpoint            | [px4_msgs::msg::RoverPositionSetpoint](../msg_docs/RoverPositionSetpoint.md)                       |
| /fmu/in/rover_speed_setpoint               | [px4_msgs::msg::RoverSpeedSetpoint](../msg_docs/RoverSpeedSetpoint.md)                             |
| /fmu/in/rover_attitude_setpoint            | [px4_msgs::msg::RoverAttitudeSetpoint](../msg_docs/RoverAttitudeSetpoint.md)                       |
| /fmu/in/rover_rate_setpoint                | [px4_msgs::msg::RoverRateSetpoint](../msg_docs/RoverRateSetpoint.md)                               |
| /fmu/in/rover_throttle_setpoint            | [px4_msgs::msg::RoverThrottleSetpoint](../msg_docs/RoverThrottleSetpoint.md)                       |
| /fmu/in/rover_steering_setpoint            | [px4_msgs::msg::RoverSteeringSetpoint](../msg_docs/RoverSteeringSetpoint.md)                       |
| /fmu/in/landing_gear                       | [px4_msgs::msg::LandingGear](../msg_docs/LandingGear.md)                                           |

## Subscriptions Multi

| Topic                       | Type                                                                 | Route Field | Max Instances |
| --------------------------- | -------------------------------------------------------------------- | ----------- | ------------- |
| /fmu/in/aux_global_position | [px4_msgs::msg::AuxGlobalPosition](../msg_docs/AuxGlobalPosition.md) | `id`        | 4             |

## Not Exported

These messages are not listed in the yaml file.
They are not build into the module, and hence are neither published or subscribed.

::: details See messages

- [VehicleLocalPositionV0](../msg_docs/VehicleLocalPositionV0.md)
- [OrbTest](../msg_docs/OrbTest.md)
- [EstimatorAidSource1d](../msg_docs/EstimatorAidSource1d.md)
- [PowerButtonState](../msg_docs/PowerButtonState.md)
- [GimbalManagerStatus](../msg_docs/GimbalManagerStatus.md)
- [FailureDetectorStatus](../msg_docs/FailureDetectorStatus.md)
- [QshellReq](../msg_docs/QshellReq.md)
- [Gripper](../msg_docs/Gripper.md)
- [RoverAttitudeStatus](../msg_docs/RoverAttitudeStatus.md)
- [SensorGyro](../msg_docs/SensorGyro.md)
- [LaunchDetectionStatus](../msg_docs/LaunchDetectionStatus.md)
- [DatamanRequest](../msg_docs/DatamanRequest.md)
- [DebugKeyValue](../msg_docs/DebugKeyValue.md)
- [SensorPreflightMag](../msg_docs/SensorPreflightMag.md)
- [GeofenceStatus](../msg_docs/GeofenceStatus.md)
- [NavigatorMissionItem](../msg_docs/NavigatorMissionItem.md)
- [BatteryStatusV0](../msg_docs/BatteryStatusV0.md)
- [TuneControl](../msg_docs/TuneControl.md)
- [OpenDroneIdOperatorId](../msg_docs/OpenDroneIdOperatorId.md)
- [CameraTrigger](../msg_docs/CameraTrigger.md)
- [PositionSetpoint](../msg_docs/PositionSetpoint.md)
- [EstimatorBias3d](../msg_docs/EstimatorBias3d.md)
- [InputRc](../msg_docs/InputRc.md)
- [UavcanParameterValue](../msg_docs/UavcanParameterValue.md)
- [EstimatorAidSource2d](../msg_docs/EstimatorAidSource2d.md)
- [SensorUwb](../msg_docs/SensorUwb.md)
- [CellularStatus](../msg_docs/CellularStatus.md)
- [GainCompression](../msg_docs/GainCompression.md)
- [SensorsStatus](../msg_docs/SensorsStatus.md)
- [LedControl](../msg_docs/LedControl.md)
- [VehicleAcceleration](../msg_docs/VehicleAcceleration.md)
- [VehicleStatusV3](../msg_docs/VehicleStatusV3.md)
- [RaptorStatus](../msg_docs/RaptorStatus.md)
- [NeuralControl](../msg_docs/NeuralControl.md)
- [OrbitStatus](../msg_docs/OrbitStatus.md)
- [Ping](../msg_docs/Ping.md)
- [EstimatorSensorBias](../msg_docs/EstimatorSensorBias.md)
- [VehicleConstraints](../msg_docs/VehicleConstraints.md)
- [WheelEncoders](../msg_docs/WheelEncoders.md)
- [ConfigOverridesV0](../msg_docs/ConfigOverridesV0.md)
- [EstimatorEventFlags](../msg_docs/EstimatorEventFlags.md)
- [ActuatorServosTrim](../msg_docs/ActuatorServosTrim.md)
- [PurePursuitStatus](../msg_docs/PurePursuitStatus.md)
- [PositionControllerLandingStatus](../msg_docs/PositionControllerLandingStatus.md)
- [EscEepromRead](../msg_docs/EscEepromRead.md)
- [OpenDroneIdArmStatus](../msg_docs/OpenDroneIdArmStatus.md)
- [EscReport](../msg_docs/EscReport.md)
- [DifferentialPressure](../msg_docs/DifferentialPressure.md)
- [AdcReport](../msg_docs/AdcReport.md)
- [MavlinkLog](../msg_docs/MavlinkLog.md)
- [ParameterSetUsedRequest](../msg_docs/ParameterSetUsedRequest.md)
- [SensorBaro](../msg_docs/SensorBaro.md)
- [LandingTargetInnovations](../msg_docs/LandingTargetInnovations.md)
- [NavigatorStatus](../msg_docs/NavigatorStatus.md)
- [Airspeed](../msg_docs/Airspeed.md)
- [OrbTestMedium](../msg_docs/OrbTestMedium.md)
- [ActuatorControlsStatus](../msg_docs/ActuatorControlsStatus.md)
- [DronecanNodeStatus](../msg_docs/DronecanNodeStatus.md)
- [LoggerStatus](../msg_docs/LoggerStatus.md)
- [GimbalDeviceInformation](../msg_docs/GimbalDeviceInformation.md)
- [EstimatorBias](../msg_docs/EstimatorBias.md)
- [Mission](../msg_docs/Mission.md)
- [GimbalControls](../msg_docs/GimbalControls.md)
- [GpioIn](../msg_docs/GpioIn.md)
- [ButtonEvent](../msg_docs/ButtonEvent.md)
- [EstimatorSelectorStatus](../msg_docs/EstimatorSelectorStatus.md)
- [RadioStatus](../msg_docs/RadioStatus.md)
- [DebugVect](../msg_docs/DebugVect.md)
- [PwmInput](../msg_docs/PwmInput.md)
- [ArmingCheckRequestV0](../msg_docs/ArmingCheckRequestV0.md)
- [CanInterfaceStatus](../msg_docs/CanInterfaceStatus.md)
- [SensorSelection](../msg_docs/SensorSelection.md)
- [VehicleGlobalPositionV0](../msg_docs/VehicleGlobalPositionV0.md)
- [Rpm](../msg_docs/Rpm.md)
- [VehicleRoi](../msg_docs/VehicleRoi.md)
- [EstimatorAidSource3d](../msg_docs/EstimatorAidSource3d.md)
- [FixedWingLateralGuidanceStatus](../msg_docs/FixedWingLateralGuidanceStatus.md)
- [Vtx](../msg_docs/Vtx.md)
- [PowerMonitor](../msg_docs/PowerMonitor.md)
- [PositionControllerStatus](../msg_docs/PositionControllerStatus.md)
- [ActuatorArmed](../msg_docs/ActuatorArmed.md)
- [MissionResult](../msg_docs/MissionResult.md)
- [UavcanParameterRequest](../msg_docs/UavcanParameterRequest.md)
- [FixedWingRunwayControl](../msg_docs/FixedWingRunwayControl.md)
- [TakeoffStatus](../msg_docs/TakeoffStatus.md)
- [SensorGnssRelative](../msg_docs/SensorGnssRelative.md)
- [VehicleOpticalFlow](../msg_docs/VehicleOpticalFlow.md)
- [EstimatorStatus](../msg_docs/EstimatorStatus.md)
- [ParameterResetRequest](../msg_docs/ParameterResetRequest.md)
- [RoverRateStatus](../msg_docs/RoverRateStatus.md)
- [LandingGearWheel](../msg_docs/LandingGearWheel.md)
- [VehicleAngularAccelerationSetpoint](../msg_docs/VehicleAngularAccelerationSetpoint.md)
- [ParameterSetValueRequest](../msg_docs/ParameterSetValueRequest.md)
- [EstimatorFusionControl](../msg_docs/EstimatorFusionControl.md)
- [FollowTarget](../msg_docs/FollowTarget.md)
- [ActuatorTest](../msg_docs/ActuatorTest.md)
- [VehicleStatusV2](../msg_docs/VehicleStatusV2.md)
- [EstimatorInnovations](../msg_docs/EstimatorInnovations.md)
- [VehicleAttitudeSetpointV0](../msg_docs/VehicleAttitudeSetpointV0.md)
- [OpenDroneIdSystem](../msg_docs/OpenDroneIdSystem.md)
- [VehicleAirData](../msg_docs/VehicleAirData.md)
- [VehicleStatusV0](../msg_docs/VehicleStatusV0.md)
- [FlightPhaseEstimation](../msg_docs/FlightPhaseEstimation.md)
- [HomePositionV0](../msg_docs/HomePositionV0.md)
- [YawEstimatorStatus](../msg_docs/YawEstimatorStatus.md)
- [GeofenceResult](../msg_docs/GeofenceResult.md)
- [ArmingCheckReplyV0](../msg_docs/ArmingCheckReplyV0.md)
- [BatteryInfo](../msg_docs/BatteryInfo.md)
- [InternalCombustionEngineControl](../msg_docs/InternalCombustionEngineControl.md)
- [CameraStatus](../msg_docs/CameraStatus.md)
- [SensorsStatusImu](../msg_docs/SensorsStatusImu.md)
- [SensorAirflow](../msg_docs/SensorAirflow.md)
- [TaskStackInfo](../msg_docs/TaskStackInfo.md)
- [AirspeedWind](../msg_docs/AirspeedWind.md)
- [EstimatorStates](../msg_docs/EstimatorStates.md)
- [SensorGyroFifo](../msg_docs/SensorGyroFifo.md)
- [EscStatus](../msg_docs/EscStatus.md)
- [RtlStatus](../msg_docs/RtlStatus.md)
- [IridiumsbdStatus](../msg_docs/IridiumsbdStatus.md)
- [FixedWingLateralStatus](../msg_docs/FixedWingLateralStatus.md)
- [OrbTestLarge](../msg_docs/OrbTestLarge.md)
- [GpsInjectData](../msg_docs/GpsInjectData.md)
- [SensorMag](../msg_docs/SensorMag.md)
- [OpenDroneIdSelfId](../msg_docs/OpenDroneIdSelfId.md)
- [SensorGnssStatus](../msg_docs/SensorGnssStatus.md)
- [HealthReport](../msg_docs/HealthReport.md)
- [PpsCapture](../msg_docs/PpsCapture.md)
- [VehicleCommandAckV0](../msg_docs/VehicleCommandAckV0.md)
- [RaptorInput](../msg_docs/RaptorInput.md)
- [ManualControlSwitches](../msg_docs/ManualControlSwitches.md)
- [InternalCombustionEngineStatus](../msg_docs/InternalCombustionEngineStatus.md)
- [ActuatorOutputs](../msg_docs/ActuatorOutputs.md)
- [LogMessage](../msg_docs/LogMessage.md)
- [VehicleAngularVelocity](../msg_docs/VehicleAngularVelocity.md)
- [HeaterStatus](../msg_docs/HeaterStatus.md)
- [RangingBeacon](../msg_docs/RangingBeacon.md)
- [SensorCorrection](../msg_docs/SensorCorrection.md)
- [EventV0](../msg_docs/EventV0.md)
- [ParameterUpdate](../msg_docs/ParameterUpdate.md)
- [DatamanResponse](../msg_docs/DatamanResponse.md)
- [AutotuneAttitudeControlStatus](../msg_docs/AutotuneAttitudeControlStatus.md)
- [VehicleImuStatus](../msg_docs/VehicleImuStatus.md)
- [VehicleMagnetometer](../msg_docs/VehicleMagnetometer.md)
- [TecsStatus](../msg_docs/TecsStatus.md)
- [SensorGyroFft](../msg_docs/SensorGyroFft.md)
- [VehicleImu](../msg_docs/VehicleImu.md)
- [SatelliteInfo](../msg_docs/SatelliteInfo.md)
- [DebugArray](../msg_docs/DebugArray.md)
- [QshellRetval](../msg_docs/QshellRetval.md)
- [FuelTankStatus](../msg_docs/FuelTankStatus.md)
- [RegisterExtComponentRequestV1](../msg_docs/RegisterExtComponentRequestV1.md)
- [TiltrotorExtraControls](../msg_docs/TiltrotorExtraControls.md)
- [Ekf2Timestamps](../msg_docs/Ekf2Timestamps.md)
- [GeneratorStatus](../msg_docs/GeneratorStatus.md)
- [AirspeedValidatedV0](../msg_docs/AirspeedValidatedV0.md)
- [GpioOut](../msg_docs/GpioOut.md)
- [RegisterExtComponentRequestV0](../msg_docs/RegisterExtComponentRequestV0.md)
- [VehicleOpticalFlowVel](../msg_docs/VehicleOpticalFlowVel.md)
- [ActionRequest](../msg_docs/ActionRequest.md)
- [Cpuload](../msg_docs/Cpuload.md)
- [EscEepromWrite](../msg_docs/EscEepromWrite.md)
- [RcParameterMap](../msg_docs/RcParameterMap.md)
- [VehicleStatusV1](../msg_docs/VehicleStatusV1.md)
- [SensorAccel](../msg_docs/SensorAccel.md)
- [HoverThrustEstimate](../msg_docs/HoverThrustEstimate.md)
- [FigureEightStatus](../msg_docs/FigureEightStatus.md)
- [LandingTargetPose](../msg_docs/LandingTargetPose.md)
- [GimbalManagerSetAttitude](../msg_docs/GimbalManagerSetAttitude.md)
- [MountOrientation](../msg_docs/MountOrientation.md)
- [RoverSpeedStatus](../msg_docs/RoverSpeedStatus.md)
- [VelocityLimits](../msg_docs/VelocityLimits.md)
- [TrajectorySetpoint6dof](../msg_docs/TrajectorySetpoint6dof.md)
- [DistanceSensorModeChangeRequest](../msg_docs/DistanceSensorModeChangeRequest.md)
- [ControlAllocatorStatus](../msg_docs/ControlAllocatorStatus.md)
- [MagWorkerData](../msg_docs/MagWorkerData.md)
- [MagnetometerBiasEstimate](../msg_docs/MagnetometerBiasEstimate.md)
- [GpsDump](../msg_docs/GpsDump.md)
- [CameraCapture](../msg_docs/CameraCapture.md)
- [GimbalManagerSetManualControl](../msg_docs/GimbalManagerSetManualControl.md)
- [MavlinkTunnel](../msg_docs/MavlinkTunnel.md)
- [IrlockReport](../msg_docs/IrlockReport.md)
- [GpioConfig](../msg_docs/GpioConfig.md)
- [DeviceInformation](../msg_docs/DeviceInformation.md)
- [DebugValue](../msg_docs/DebugValue.md)
- [GimbalManagerInformation](../msg_docs/GimbalManagerInformation.md)
- [EstimatorGpsStatus](../msg_docs/EstimatorGpsStatus.md)
- [ParameterSetValueResponse](../msg_docs/ParameterSetValueResponse.md)
- [Event](../msg_docs/Event.md)
- [GpioRequest](../msg_docs/GpioRequest.md)
- [UlogStreamAck](../msg_docs/UlogStreamAck.md)
- [VehicleLocalPositionSetpoint](../msg_docs/VehicleLocalPositionSetpoint.md)
- [Px4ioStatus](../msg_docs/Px4ioStatus.md)
- [RegisterExtComponentReplyV0](../msg_docs/RegisterExtComponentReplyV0.md)
- [SensorHygrometer](../msg_docs/SensorHygrometer.md)
- [SensorAccelFifo](../msg_docs/SensorAccelFifo.md)
- [NormalizedUnsignedSetpoint](../msg_docs/NormalizedUnsignedSetpoint.md)
- [FollowTargetEstimator](../msg_docs/FollowTargetEstimator.md)
- [RtlTimeEstimate](../msg_docs/RtlTimeEstimate.md)
- [RateCtrlStatus](../msg_docs/RateCtrlStatus.md)
- [SensorTemp](../msg_docs/SensorTemp.md)
- [FollowTargetStatus](../msg_docs/FollowTargetStatus.md)
- [SystemPower](../msg_docs/SystemPower.md)
- [RcChannels](../msg_docs/RcChannels.md)
- [GimbalDeviceSetAttitude](../msg_docs/GimbalDeviceSetAttitude.md)
- [UlogStream](../msg_docs/UlogStream.md)
  :::
