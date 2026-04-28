// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      m_constants;

  // Hardware objects
  private final TalonFX m_driveTalon;
  private final TalonFX m_turnTalon;
  private final CANcoder m_cancoder;

  // Voltage control requests
  private final VoltageOut m_voltageRequest = new VoltageOut(0);
  private final PositionVoltage m_positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage m_velocityVoltageRequest = new VelocityVoltage(0.0);

  // Torque-current control requests
  private final TorqueCurrentFOC m_torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC m_positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC m_velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  // Timestamp inputs from Phoenix thread
  private final Queue<Double> m_timestampQueue;

  // Inputs from drive motor
  private final StatusSignal<Angle> m_drivePosition;
  private final Queue<Double> m_drivePositionQueue;
  private final StatusSignal<AngularVelocity> m_driveVelocity;
  private final StatusSignal<Voltage> m_driveAppliedVolts;
  private final StatusSignal<Current> m_driveCurrent;

  // Inputs from turn motor
  private final StatusSignal<Angle> m_turnAbsolutePosition;
  private final StatusSignal<Angle> m_turnPosition;
  private final Queue<Double> m_turnPositionQueue;
  private final StatusSignal<AngularVelocity> m_turnVelocity;
  private final StatusSignal<Voltage> m_turnAppliedVolts;
  private final StatusSignal<Current> m_turnCurrent;

  // Connection debouncers
  private final Debouncer m_driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer m_turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer m_turnEncoderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ModuleIOTalonFX(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.m_constants = constants;
    m_driveTalon = new TalonFX(m_constants.DriveMotorId, TunerConstants.kCANBus);
    m_turnTalon = new TalonFX(m_constants.SteerMotorId, TunerConstants.kCANBus);
    m_cancoder = new CANcoder(m_constants.EncoderId, TunerConstants.kCANBus);

    // Configure drive motor
    var driveConfig = m_constants.DriveMotorInitialConfigs;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 = m_constants.DriveMotorGains;
    driveConfig.Feedback.SensorToMechanismRatio = m_constants.DriveMotorGearRatio;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = m_constants.SlipCurrent;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -m_constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimit = m_constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted =
        m_constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> m_driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> m_driveTalon.setPosition(0.0, 0.25));

    // Configure turn motor
    var turnConfig = new TalonFXConfiguration();
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0 = m_constants.SteerMotorGains;
    turnConfig.Feedback.FeedbackRemoteSensorID = m_constants.EncoderId;
    turnConfig.Feedback.FeedbackSensorSource =
        switch (m_constants.FeedbackSource) {
          case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
          case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
          case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
          default -> throw new RuntimeException(
              "You have selected a turn feedback source that is not supported by the default implementation of ModuleIOTalonFX. Please check the AdvantageKit documentation for more information on alternative configurations: https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations");
        };
    turnConfig.Feedback.RotorToSensorRatio = m_constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / m_constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicAcceleration =
        turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * m_constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.MotorOutput.Inverted =
        m_constants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> m_turnTalon.getConfigurator().apply(turnConfig, 0.25));

    // Configure CANCoder
    CANcoderConfiguration m_cancoderConfig = m_constants.EncoderInitialConfigs;
    m_cancoderConfig.MagnetSensor.MagnetOffset = m_constants.EncoderOffset;
    m_cancoderConfig.MagnetSensor.SensorDirection =
        m_constants.EncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    m_cancoder.getConfigurator().apply(m_cancoderConfig);

    // Create timestamp queue
    m_timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    // Create drive status signals
    m_drivePosition = m_driveTalon.getPosition();
    m_drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(m_drivePosition.clone());
    m_driveVelocity = m_driveTalon.getVelocity();
    m_driveAppliedVolts = m_driveTalon.getMotorVoltage();
    m_driveCurrent = m_driveTalon.getStatorCurrent();

    // Create turn status signals
    m_turnAbsolutePosition = m_cancoder.getAbsolutePosition();
    m_turnPosition = m_turnTalon.getPosition();
    m_turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(m_turnPosition.clone());
    m_turnVelocity = m_turnTalon.getVelocity();
    m_turnAppliedVolts = m_turnTalon.getMotorVoltage();
    m_turnCurrent = m_turnTalon.getStatorCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        Drive.ODOMETRY_FREQUENCY, m_drivePosition, m_turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        m_driveVelocity,
        m_driveAppliedVolts,
        m_driveCurrent,
        m_turnAbsolutePosition,
        m_turnVelocity,
        m_turnAppliedVolts,
        m_turnCurrent);
    ParentDevice.optimizeBusUtilizationForAll(m_driveTalon, m_turnTalon);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Refresh all signals
    var driveStatus =
        BaseStatusSignal.refreshAll(
            m_drivePosition, m_driveVelocity, m_driveAppliedVolts, m_driveCurrent);
    var turnStatus =
        BaseStatusSignal.refreshAll(
            m_turnPosition, m_turnVelocity, m_turnAppliedVolts, m_turnCurrent);
    var turnEncoderStatus = BaseStatusSignal.refreshAll(m_turnAbsolutePosition);

    // Update drive inputs
    inputs.driveConnected = m_driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePositionRad = Units.rotationsToRadians(m_drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(m_driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = m_driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = m_driveCurrent.getValueAsDouble();

    // Update turn inputs
    inputs.turnConnected = m_turnConnectedDebounce.calculate(turnStatus.isOK());
    inputs.turnEncoderConnected =
        m_turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(m_turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(m_turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(m_turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = m_turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = m_turnCurrent.getValueAsDouble();

    // Update odometry inputs
    inputs.odometryTimestamps =
        m_timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        m_drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
    inputs.odometryTurnPositions =
        m_turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    m_timestampQueue.clear();
    m_drivePositionQueue.clear();
    m_turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    m_driveTalon.setControl(
        switch (m_constants.DriveMotorClosedLoopOutput) {
          case Voltage -> m_voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> m_torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setTurnOpenLoop(double output) {
    m_turnTalon.setControl(
        switch (m_constants.SteerMotorClosedLoopOutput) {
          case Voltage -> m_voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> m_torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    m_driveTalon.setControl(
        switch (m_constants.DriveMotorClosedLoopOutput) {
          case Voltage -> m_velocityVoltageRequest.withVelocity(velocityRotPerSec);
          case TorqueCurrentFOC -> m_velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
        });
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    m_turnTalon.setControl(
        switch (m_constants.SteerMotorClosedLoopOutput) {
          case Voltage -> m_positionVoltageRequest.withPosition(rotation.getRotations());
          case TorqueCurrentFOC -> m_positionTorqueCurrentRequest.withPosition(
              rotation.getRotations());
        });
  }
}
