// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Constants.ShooterConstants;

/**
 * This superstructure implementation is for Talon FXs driving motors like the Falcon 500, Kraken
 * X44, or Kraken X60.
 */
public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX m_middleShooter = new TalonFX(ShooterConstants.MIDDLE_CAN_ID, "Drivetrain");
  private final TalonFX m_rightShooter = new TalonFX(ShooterConstants.RIGHT_CAN_ID, "Drivetrain");
  private final TalonFX m_leftShooter = new TalonFX(ShooterConstants.LEFT_CAN_ID, "Drivetrain");

  private final StatusSignal<AngularVelocity> m_middleShooterVelocityRotPerSec =
      m_middleShooter.getVelocity();
  private final StatusSignal<Voltage> m_middleShooterAppliedVolts =
      m_middleShooter.getMotorVoltage();
  private final StatusSignal<Current> m_middleShooterCurrentAmps =
      m_middleShooter.getSupplyCurrent();

  private final StatusSignal<AngularVelocity> m_rightShooterVelocityRotPerSec =
      m_rightShooter.getVelocity();
  private final StatusSignal<Voltage> m_rightShooterAppliedVolts = m_rightShooter.getMotorVoltage();
  private final StatusSignal<Current> m_rightShooterCurrentAmps = m_rightShooter.getSupplyCurrent();

  private final StatusSignal<AngularVelocity> m_leftShooterVelocityRotPerSec =
      m_leftShooter.getVelocity();
  private final StatusSignal<Voltage> m_leftShooterAppliedVolts = m_leftShooter.getMotorVoltage();
  private final StatusSignal<Current> m_leftShooterCurrentAmps = m_leftShooter.getSupplyCurrent();

  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);

  // Track the last velocity setpoint
  private double m_targetVelocityRPS = 0.0;

  private int m_loopCounter = 0;

  // SmartDashboard PID tuning cache
  private double m_lastKPL = ShooterConstants.kPL;
  private double m_lastKIL = ShooterConstants.kIL;
  private double m_lastKDL = ShooterConstants.kDL;
  private double m_lastKVL = ShooterConstants.kVL;

  private double m_lastKPM = ShooterConstants.kPM;
  private double m_lastKIM = ShooterConstants.kIM;
  private double m_lastKDM = ShooterConstants.kDM;
  private double m_lastKVM = ShooterConstants.kVM;

  private double m_lastKPR = ShooterConstants.kPR;
  private double m_lastKIR = ShooterConstants.kIR;
  private double m_lastKDR = ShooterConstants.kDR;
  private double m_lastKVR = ShooterConstants.kVR;

  // private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public ShooterIOTalonFX() {

    // Config for left shooter motor
    TalonFXConfiguration leftConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withVoltage(new VoltageConfigs().withPeakReverseVoltage(0))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(ShooterConstants.STATOR_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(ShooterConstants.CURRENT_LIMIT)
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(
                new Slot0Configs()
                    .withKP(ShooterConstants.kPL)
                    .withKI(ShooterConstants.kIL)
                    .withKD(ShooterConstants.kDL)
                    .withKV(ShooterConstants.kVL));

    // Config for middle shooter motor
    TalonFXConfiguration middleConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withVoltage(new VoltageConfigs().withPeakReverseVoltage(0))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(ShooterConstants.STATOR_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(ShooterConstants.CURRENT_LIMIT)
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(
                new Slot0Configs()
                    .withKP(ShooterConstants.kPM)
                    .withKI(ShooterConstants.kIM)
                    .withKD(ShooterConstants.kDM)
                    .withKV(ShooterConstants.kVM));

    // config for right shooter motor
    TalonFXConfiguration rightConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withVoltage(new VoltageConfigs().withPeakReverseVoltage(0))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(ShooterConstants.STATOR_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(ShooterConstants.CURRENT_LIMIT)
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(
                new Slot0Configs()
                    .withKP(ShooterConstants.kPR)
                    .withKI(ShooterConstants.kIR)
                    .withKD(ShooterConstants.kDR)
                    .withKV(ShooterConstants.kVR));

    // Middle axle — invert if spinning wrong direction: change to ClockWise_Positive
    tryUntilOk(5, () -> m_middleShooter.getConfigurator().apply(middleConfig, 0.25));
    // Left axle — invert if spinning wrong direction: change to ClockWise_Positive
    tryUntilOk(5, () -> m_leftShooter.getConfigurator().apply(leftConfig, 0.25));
    // Right axle — invert if spinning wrong direction: change to ClockWise_Positive
    tryUntilOk(5, () -> m_rightShooter.getConfigurator().apply(rightConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        RobotStateConstants.UPDATE_FREQUENCY_HZ,
        m_middleShooterVelocityRotPerSec,
        m_middleShooterAppliedVolts,
        m_middleShooterCurrentAmps,
        m_rightShooterVelocityRotPerSec,
        m_rightShooterAppliedVolts,
        m_rightShooterCurrentAmps,
        m_leftShooterVelocityRotPerSec,
        m_leftShooterAppliedVolts,
        m_leftShooterCurrentAmps);

    m_middleShooter.optimizeBusUtilization();
    m_rightShooter.optimizeBusUtilization();
    m_leftShooter.optimizeBusUtilization();

    // Initialize SmartDashboard PID tuning values
    SmartDashboard.putNumber("Shooter/kP", ShooterConstants.kPM);
    SmartDashboard.putNumber("Shooter/kI", ShooterConstants.kIM);
    SmartDashboard.putNumber("Shooter/kD", ShooterConstants.kDM);
    SmartDashboard.putNumber("Shooter/kV", ShooterConstants.kVM);

    SmartDashboard.putNumber("Shooter/kP", ShooterConstants.kPL);
    SmartDashboard.putNumber("Shooter/kI", ShooterConstants.kIL);
    SmartDashboard.putNumber("Shooter/kD", ShooterConstants.kDL);
    SmartDashboard.putNumber("Shooter/kV", ShooterConstants.kVL);

    SmartDashboard.putNumber("Shooter/kP", ShooterConstants.kPR);
    SmartDashboard.putNumber("Shooter/kI", ShooterConstants.kIR);
    SmartDashboard.putNumber("Shooter/kD", ShooterConstants.kDR);
    SmartDashboard.putNumber("Shooter/kV", ShooterConstants.kVR);
  }

  private void updatePIDFromDashboard() {

    double kPL = SmartDashboard.getNumber("Shooter/kP", m_lastKPL);
    double kIL = SmartDashboard.getNumber("Shooter/kI", m_lastKIL);
    double kDL = SmartDashboard.getNumber("Shooter/kD", m_lastKDL);
    double kVL = SmartDashboard.getNumber("Shooter/kV", m_lastKVL);

    double kPM = SmartDashboard.getNumber("Shooter/kP", m_lastKPM);
    double kIM = SmartDashboard.getNumber("Shooter/kI", m_lastKIM);
    double kDM = SmartDashboard.getNumber("Shooter/kD", m_lastKDM);
    double kVM = SmartDashboard.getNumber("Shooter/kV", m_lastKVM);

    double kPR = SmartDashboard.getNumber("Shooter/kP", m_lastKPR);
    double kIR = SmartDashboard.getNumber("Shooter/kI", m_lastKIR);
    double kDR = SmartDashboard.getNumber("Shooter/kD", m_lastKDR);
    double kVR = SmartDashboard.getNumber("Shooter/kV", m_lastKVR);

    if (kPL != m_lastKPL
        || kIL != m_lastKIL
        || kDL != m_lastKDL
        || kVL != m_lastKVL
        || kPM != m_lastKPM
        || kIM != m_lastKIM
        || kDM != m_lastKDM
        || kVM != m_lastKVM
        || kPR != m_lastKPR
        || kIR != m_lastKIR
        || kDR != m_lastKDR
        || kVR != m_lastKVR) {

      var leftSlotConfig = new Slot0Configs().withKP(kPL).withKI(kIL).withKD(kDL).withKV(kVL);
      var middleSlotConfig = new Slot0Configs().withKP(kPM).withKI(kIM).withKD(kDM).withKV(kVM);
      var rightSlotConfig = new Slot0Configs().withKP(kPR).withKI(kIR).withKD(kDR).withKV(kVR);

      tryUntilOk(5, () -> m_middleShooter.getConfigurator().apply(middleSlotConfig, 0.25));
      tryUntilOk(5, () -> m_rightShooter.getConfigurator().apply(rightSlotConfig, 0.25));
      tryUntilOk(5, () -> m_leftShooter.getConfigurator().apply(leftSlotConfig, 0.25));

      m_lastKPL = kPL;
      m_lastKIL = kIL;
      m_lastKDL = kDL;
      m_lastKVL = kVL;

      m_lastKPM = kPM;
      m_lastKIM = kIM;
      m_lastKDM = kDM;
      m_lastKVM = kVM;

      m_lastKPR = kPR;
      m_lastKIR = kIR;
      m_lastKDR = kDR;
      m_lastKVR = kVR;
    }
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    m_loopCounter++;

    // PID tuning — poll SmartDashboard every 50 loops (~1 second)
    if (m_loopCounter % 50 == 0) {
      updatePIDFromDashboard();
      m_loopCounter = 0;
    }

    BaseStatusSignal.refreshAll(
        m_middleShooterVelocityRotPerSec,
        m_middleShooterAppliedVolts,
        m_middleShooterCurrentAmps,
        m_rightShooterVelocityRotPerSec,
        m_rightShooterAppliedVolts,
        m_rightShooterCurrentAmps,
        m_leftShooterVelocityRotPerSec,
        m_leftShooterAppliedVolts,
        m_leftShooterCurrentAmps);

    // Motor rotations -> shooter rotations * gear ratio
    inputs.middleShooterRPS =
        m_middleShooterVelocityRotPerSec.getValueAsDouble() * ShooterConstants.GEAR_RATIO;
    inputs.middleShooterAppliedVolts = m_middleShooterAppliedVolts.getValueAsDouble();
    inputs.middleShooterCurrentAmps = m_middleShooterCurrentAmps.getValueAsDouble();
    inputs.velocityErrorRPS =
        m_targetVelocityRPS - inputs.middleShooterRPS; // Compute the velocity error

    inputs.rightShooterRPS =
        m_rightShooterVelocityRotPerSec.getValueAsDouble() * ShooterConstants.GEAR_RATIO;
    inputs.rightShooterAppliedVolts = m_rightShooterAppliedVolts.getValueAsDouble();
    inputs.rightShooterCurrentAmps = m_rightShooterCurrentAmps.getValueAsDouble();

    inputs.leftShooterRPS =
        m_leftShooterVelocityRotPerSec.getValueAsDouble() * ShooterConstants.GEAR_RATIO;
    inputs.leftShooterAppliedVolts = m_leftShooterAppliedVolts.getValueAsDouble();
    inputs.leftShooterCurrentAmps = m_leftShooterCurrentAmps.getValueAsDouble();
  }

  @Override
  public void setVelocity(double velocityRPM) {
    m_targetVelocityRPS = velocityRPM / 60.0;

    var request = m_velocityRequest.withVelocity(m_targetVelocityRPS).withSlot(0);

    m_middleShooter.setControl(request);
    m_rightShooter.setControl(request);
    m_leftShooter.setControl(request);
  }
}
