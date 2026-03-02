// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Constants.ShooterConstants;

/**
 * This superstructure implementation is for Talon FXs driving motors like the Falon 500, Kraken
 * X44, or Kraken X60.
 */
public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX m_shooter = new TalonFX(ShooterConstants.MIDDLE_CAN_ID);
  private final StatusSignal<AngularVelocity> shooterVelocityRotPerSec = m_shooter.getVelocity();
  private final StatusSignal<Voltage> shooterAppliedVolts = m_shooter.getMotorVoltage();
  private final StatusSignal<Current> shooterCurrentAmps = m_shooter.getSupplyCurrent();
  private final StatusSignal<Temperature> shooterTempCelsius = m_shooter.getDeviceTemp();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public ShooterIOTalonFX() {
    var shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.Inverted =
        InvertedValue.CounterClockwise_Positive; // TODO: migrate to init
    shooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.CURRENT_LIMIT;
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    tryUntilOk(5, () -> m_shooter.getConfigurator().apply(shooterConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        RobotStateConstants.UPDATE_FREQUENCY_HZ,
        shooterVelocityRotPerSec,
        shooterAppliedVolts,
        shooterCurrentAmps,
        shooterTempCelsius);

    m_shooter.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        shooterVelocityRotPerSec, shooterAppliedVolts, shooterCurrentAmps, shooterTempCelsius);

    // Motor rotations -> feeder rotations * 60 sec/min
    inputs.shooterRPM =
        shooterVelocityRotPerSec.getValueAsDouble() * 60 / ShooterConstants.GEAR_RATIO;
    inputs.shooterAppliedVolts = shooterAppliedVolts.getValueAsDouble();
    inputs.shooterCurrentAmps = shooterCurrentAmps.getValueAsDouble();
    inputs.shooterTempCelsius = shooterTempCelsius.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    m_shooter.setControl(
        voltageRequest.withOutput(
            MathUtil.clamp(
                volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE)));
  }
}
