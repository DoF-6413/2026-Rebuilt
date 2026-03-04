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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Constants.ShooterConstants;

/**
 * This superstructure implementation is for Talon FXs driving motors like the Falon 500, Kraken
 * X44, or Kraken X60.
 */
public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX m_middleShooter = new TalonFX(ShooterConstants.MIDDLE_CAN_ID, "Drivetrain");
  private final TalonFX m_rightShooter = new TalonFX(ShooterConstants.RIGHT_CAN_ID, "Drivetrain");
  private final TalonFX m_leftShooter = new TalonFX(ShooterConstants.LEFT_CAN_ID, "Drivetrain");

  private final StatusSignal<AngularVelocity> middleShooterVelocityRotPerSec =
      m_middleShooter.getVelocity();
  private final StatusSignal<Voltage> middleShooterAppliedVolts = m_middleShooter.getMotorVoltage();
  private final StatusSignal<Current> middleShooterCurrentAmps = m_middleShooter.getSupplyCurrent();
  private final StatusSignal<Temperature> middleShooterTempCelsius =
      m_middleShooter.getDeviceTemp();

  private final StatusSignal<AngularVelocity> rightShooterVelocityRotPerSec =
      m_rightShooter.getVelocity();
  private final StatusSignal<Voltage> rightShooterAppliedVolts = m_rightShooter.getMotorVoltage();
  private final StatusSignal<Current> rightShooterCurrentAmps = m_rightShooter.getSupplyCurrent();
  private final StatusSignal<Temperature> rightShooterTempCelsius = m_rightShooter.getDeviceTemp();

  private final StatusSignal<AngularVelocity> leftShooterVelocityRotPerSec =
      m_leftShooter.getVelocity();
  private final StatusSignal<Voltage> leftShooterAppliedVolts = m_leftShooter.getMotorVoltage();
  private final StatusSignal<Current> leftShooterCurrentAmps = m_leftShooter.getSupplyCurrent();
  private final StatusSignal<Temperature> leftShooterTempCelsius = m_leftShooter.getDeviceTemp();

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  //   private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public ShooterIOTalonFX() {
    var shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.Inverted =
        InvertedValue.CounterClockwise_Positive; // TODO: migrate to init
    shooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.CURRENT_LIMIT;
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // tryUntilOk(5, () -> m_middleShooter.getConfigurator().apply(shooterConfig, 0.25));
    tryUntilOk(5, () -> m_rightShooter.getConfigurator().apply(shooterConfig, 0.25));
    tryUntilOk(5, () -> m_leftShooter.getConfigurator().apply(shooterConfig, 0.25));

    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withVoltage(new VoltageConfigs().withPeakReverseVoltage(0))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(120)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(70)
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(
                new Slot0Configs()
                    .withKP(ShooterConstants.kP)
                    .withKI(ShooterConstants.kI) // 2
                    .withKD(ShooterConstants.kD)
                    .withKV(ShooterConstants.kV) // 12 volts when requesting max RPS
                );

    tryUntilOk(5, () -> m_middleShooter.getConfigurator().apply(config, 0.25));

    m_rightShooter.setControl(
        new Follower(
            m_middleShooter.getDeviceID(),
            MotorAlignmentValue.Opposed)); // * Mounted inverted! Keep opposite */
    m_leftShooter.setControl(
        new Follower(m_middleShooter.getDeviceID(), MotorAlignmentValue.Aligned));

    BaseStatusSignal.setUpdateFrequencyForAll(
        RobotStateConstants.UPDATE_FREQUENCY_HZ,
        middleShooterVelocityRotPerSec,
        middleShooterAppliedVolts,
        middleShooterCurrentAmps,
        middleShooterTempCelsius,
        rightShooterVelocityRotPerSec,
        rightShooterAppliedVolts,
        rightShooterCurrentAmps,
        rightShooterTempCelsius,
        leftShooterVelocityRotPerSec,
        leftShooterAppliedVolts,
        leftShooterCurrentAmps,
        leftShooterTempCelsius);

    m_middleShooter.optimizeBusUtilization();
    m_rightShooter.optimizeBusUtilization();
    m_leftShooter.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        middleShooterVelocityRotPerSec,
        middleShooterAppliedVolts,
        middleShooterCurrentAmps,
        middleShooterTempCelsius,
        rightShooterVelocityRotPerSec,
        rightShooterAppliedVolts,
        rightShooterCurrentAmps,
        rightShooterTempCelsius,
        leftShooterVelocityRotPerSec,
        leftShooterAppliedVolts,
        leftShooterCurrentAmps,
        leftShooterTempCelsius);

    // Motor rotations -> feeder rotations * gear ratio
    inputs.middleShooterRPS =
        middleShooterVelocityRotPerSec.getValueAsDouble() * ShooterConstants.GEAR_RATIO;
    inputs.middleShooterAppliedVolts = middleShooterAppliedVolts.getValueAsDouble();
    inputs.middleShooterCurrentAmps = middleShooterCurrentAmps.getValueAsDouble();
    inputs.middleShooterTempCelsius = middleShooterTempCelsius.getValueAsDouble();

    inputs.rightShooterRPS =
        rightShooterVelocityRotPerSec.getValueAsDouble() * ShooterConstants.GEAR_RATIO;
    inputs.rightShooterAppliedVolts = rightShooterAppliedVolts.getValueAsDouble();
    inputs.rightShooterCurrentAmps = rightShooterCurrentAmps.getValueAsDouble();
    inputs.rightShooterTempCelsius = rightShooterTempCelsius.getValueAsDouble();

    inputs.leftShooterRPS =
        leftShooterVelocityRotPerSec.getValueAsDouble() * ShooterConstants.GEAR_RATIO;
    inputs.leftShooterAppliedVolts = leftShooterAppliedVolts.getValueAsDouble();
    inputs.leftShooterCurrentAmps = leftShooterCurrentAmps.getValueAsDouble();
    inputs.leftShooterTempCelsius = leftShooterTempCelsius.getValueAsDouble();
  }

  //   @Override
  //   public void setVoltage(double volts) {
  //     m_middleShooter.setControl(
  //         voltageRequest.withOutput(
  //             MathUtil.clamp(
  //                 volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE)));
  //   }

  @Override
  public void setVelocity(double velocityRPM) {
    m_middleShooter.setControl(velocityRequest.withVelocity(velocityRPM / 60.0));
  }
}
