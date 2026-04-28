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
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.BrushedMotorWiringValue;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
 * Module IO implementation for Talon FXS drive motor controller, Talon FXS turn motor controller,
 * and CANdi (PWM 1). Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOTalonFXS implements ModuleIO {
  // Hardware objects
  private final TalonFXS m_driveTalon;
  private final TalonFXS m_turnTalon;
  private final CANdi m_candi;

  // Voltage control requests
  private final VoltageOut m_voltageRequest = new VoltageOut(0);
  private final PositionVoltage m_positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage m_velocityVoltageRequest = new VelocityVoltage(0.0);

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

  public ModuleIOTalonFXS(
      SwerveModuleConstants<TalonFXSConfiguration, TalonFXSConfiguration, CANdiConfiguration>
          constants) {
    m_driveTalon = new TalonFXS(constants.DriveMotorId, TunerConstants.kCANBus);
    m_turnTalon = new TalonFXS(constants.SteerMotorId, TunerConstants.kCANBus);
    m_candi = new CANdi(constants.EncoderId, TunerConstants.kCANBus);

    // Configure drive motor
    var driveConfig = constants.DriveMotorInitialConfigs;
    driveConfig.Commutation.MotorArrangement =
        switch (constants.DriveMotorType) {
          case TalonFXS_NEO_JST -> MotorArrangementValue.NEO_JST;
          case TalonFXS_VORTEX_JST -> MotorArrangementValue.VORTEX_JST;
          default -> MotorArrangementValue.Disabled;
        };
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 = constants.DriveMotorGains;
    driveConfig.ExternalFeedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
    driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted =
        constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> m_driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> m_driveTalon.setPosition(0.0, 0.25));

    // Configure turn motor
    var turnConfig = new TalonFXSConfiguration();
    turnConfig.Commutation.MotorArrangement =
        switch (constants.SteerMotorType) {
          case TalonFXS_Minion_JST -> MotorArrangementValue.Minion_JST;
          case TalonFXS_NEO_JST -> MotorArrangementValue.NEO_JST;
          case TalonFXS_VORTEX_JST -> MotorArrangementValue.VORTEX_JST;
          case TalonFXS_NEO550_JST -> MotorArrangementValue.NEO550_JST;
          case TalonFXS_Brushed_AB,
              TalonFXS_Brushed_AC,
              TalonFXS_Brushed_BC -> MotorArrangementValue.Brushed_DC;
          default -> MotorArrangementValue.Disabled;
        };
    turnConfig.Commutation.BrushedMotorWiring =
        switch (constants.SteerMotorType) {
          case TalonFXS_Brushed_AC -> BrushedMotorWiringValue.Leads_A_and_C;
          case TalonFXS_Brushed_BC -> BrushedMotorWiringValue.Leads_B_and_C;
          default -> BrushedMotorWiringValue.Leads_A_and_B;
        };
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0 = constants.SteerMotorGains;
    turnConfig.ExternalFeedback.FeedbackRemoteSensorID = constants.EncoderId;
    turnConfig.ExternalFeedback.ExternalFeedbackSensorSource =
        switch (constants.FeedbackSource) {
          case RemoteCANdiPWM1 -> ExternalFeedbackSensorSourceValue.RemoteCANdiPWM1;
          case FusedCANdiPWM1 -> ExternalFeedbackSensorSourceValue.FusedCANdiPWM1;
          case SyncCANdiPWM1 -> ExternalFeedbackSensorSourceValue.SyncCANdiPWM1;
          default -> throw new RuntimeException(
              "You have selected a turn feedback source that is not supported by the default implementation of ModuleIOTalonFXS (CANdi PWM 1). Please check the AdvantageKit documentation for more information on alternative configurations: https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations");
        };
    turnConfig.ExternalFeedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicAcceleration =
        turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.MotorOutput.Inverted =
        constants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> m_turnTalon.getConfigurator().apply(turnConfig, 0.25));

    // Configure CANdi
    CANdiConfiguration m_candiConfig = constants.EncoderInitialConfigs;
    m_candiConfig.PWM1.AbsoluteSensorOffset = constants.EncoderOffset;
    m_candiConfig.PWM1.SensorDirection = constants.EncoderInverted;
    m_candi.getConfigurator().apply(m_candiConfig);

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
    m_turnAbsolutePosition = m_candi.getPWM1Position();
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
    m_driveTalon.setControl(m_voltageRequest.withOutput(output));
  }

  @Override
  public void setTurnOpenLoop(double output) {
    m_turnTalon.setControl(m_voltageRequest.withOutput(output));
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    m_driveTalon.setControl(m_velocityVoltageRequest.withVelocity(velocityRotPerSec));
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    m_turnTalon.setControl(m_positionVoltageRequest.withPosition(rotation.getRotations()));
  }
}
