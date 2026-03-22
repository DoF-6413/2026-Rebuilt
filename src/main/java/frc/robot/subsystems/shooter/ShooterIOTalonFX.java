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
  private double m_lastKP = ShooterConstants.kP;
  private double m_lastKI = ShooterConstants.kI;
  private double m_lastKD = ShooterConstants.kD;
  private double m_lastKV = ShooterConstants.kV;

  // private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public ShooterIOTalonFX() {

    var shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
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
                    .withKI(ShooterConstants.kI)
                    .withKD(ShooterConstants.kD)
                    .withKV(ShooterConstants.kV));

    tryUntilOk(5, () -> m_middleShooter.getConfigurator().apply(config, 0.25));

    m_rightShooter.setControl(
        new Follower(
            m_middleShooter.getDeviceID(),
            MotorAlignmentValue.Opposed)); // * Mounted inverted! Keep opposite */

    m_leftShooter.setControl(
        new Follower(m_middleShooter.getDeviceID(), MotorAlignmentValue.Aligned));

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
    SmartDashboard.putNumber("Shooter/kP", ShooterConstants.kP);
    SmartDashboard.putNumber("Shooter/kI", ShooterConstants.kI);
    SmartDashboard.putNumber("Shooter/kD", ShooterConstants.kD);
    SmartDashboard.putNumber("Shooter/kV", ShooterConstants.kV);
  }

  private void updatePIDFromDashboard() {

    double kP = SmartDashboard.getNumber("Shooter/kP", m_lastKP);
    double kI = SmartDashboard.getNumber("Shooter/kI", m_lastKI);
    double kD = SmartDashboard.getNumber("Shooter/kD", m_lastKD);
    double kV = SmartDashboard.getNumber("Shooter/kV", m_lastKV);

    if (kP != m_lastKP || kI != m_lastKI || kD != m_lastKD || kV != m_lastKV) {

      var slotConfig = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withKV(kV);

      tryUntilOk(5, () -> m_middleShooter.getConfigurator().apply(slotConfig, 0.25));

      m_lastKP = kP;
      m_lastKI = kI;
      m_lastKD = kD;
      m_lastKV = kV;
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
    m_middleShooter.setControl(m_velocityRequest.withVelocity(m_targetVelocityRPS).withSlot(0));
  }
}
