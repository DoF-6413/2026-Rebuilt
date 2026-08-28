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

  // private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public ShooterIOTalonFX() {

    // Config for left shooter motor
    TalonFXConfiguration leftConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    // Invert if spinning wrong direction: change to ClockWise_Positive
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
                    // Invert if spinning wrong direction: change to ClockWise_Positive
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
                    // Invert if spinning wrong direction: change to CounterClockWise_Positive
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

    tryUntilOk(5, () -> m_middleShooter.getConfigurator().apply(middleConfig, 0.25));
    tryUntilOk(5, () -> m_leftShooter.getConfigurator().apply(leftConfig, 0.25));
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
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

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

  /**
   * Sets the shooter wheel speeds to the specified velocity in RPM.
   *
   * @param velocityRPM - The desired wheel velocity in RPM (double)
   */
  @Override
  public void setVelocity(double velocityRPM) {
    // Convert RPM to RPS for the CTRE libraries use
    m_targetVelocityRPS = velocityRPM / 60.0;

    var request = m_velocityRequest.withVelocity(m_targetVelocityRPS).withSlot(0);

    m_middleShooter.setControl(request);
    m_rightShooter.setControl(request);
    m_leftShooter.setControl(request);
  }

  /**
   * Sets the shooter wheel speeds to the specified voltages.
   *
   * @param volts - The desired wheel volts (double)
   */
  @Override
  public void setVoltage(double volts) {
    m_middleShooter.setVoltage(volts);
    m_rightShooter.setVoltage(volts);
    m_leftShooter.setVoltage(volts);
  }
}
