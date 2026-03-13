// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.column;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants.ColumnConstants;
import frc.robot.Constants.RobotStateConstants;

/**
 * This superstructure implementation is for Talon FXs driving motors like the Falon 500, Kraken
 * X44, or Kraken X60.
 */
public class ColumnIOTalonFX implements ColumnIO {
  private final TalonFX m_column = new TalonFX(ColumnConstants.CAN_ID, "Drivetrain");
  private final StatusSignal<AngularVelocity> columnVelocityRotPerSec = m_column.getVelocity();
  private final StatusSignal<Voltage> columnAppliedVolts = m_column.getMotorVoltage();
  private final StatusSignal<Current> columnCurrentAmps = m_column.getSupplyCurrent();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public ColumnIOTalonFX() {
    var columnConfig = new TalonFXConfiguration();
    columnConfig.MotorOutput.Inverted =
        ColumnConstants.IS_INVERTED
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
    columnConfig.CurrentLimits.SupplyCurrentLimitEnable = ColumnConstants.ENABLE_CURRENT_LIMIT;
    columnConfig.CurrentLimits.SupplyCurrentLimit = ColumnConstants.CURRENT_LIMIT;
    columnConfig.CurrentLimits.StatorCurrentLimitEnable = ColumnConstants.ENABLE_CURRENT_LIMIT;
    columnConfig.CurrentLimits.StatorCurrentLimit = ColumnConstants.CURRENT_LIMIT;
    columnConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    tryUntilOk(
        5,
        () ->
            m_column
                .getConfigurator()
                .apply(columnConfig, RobotStateConstants.PHX_CONFIG_TIMEOUT_SEC));

    BaseStatusSignal.setUpdateFrequencyForAll(
        RobotStateConstants.UPDATE_FREQUENCY_HZ,
        columnVelocityRotPerSec,
        columnAppliedVolts,
        columnCurrentAmps);

    m_column.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ColumnIOInputs inputs) {
    BaseStatusSignal.refreshAll(columnVelocityRotPerSec, columnAppliedVolts, columnCurrentAmps);

    // Motor rotations -> feeder rotations * 60 sec/min
    inputs.columnRPM = columnVelocityRotPerSec.getValueAsDouble() * 60 / ColumnConstants.GEAR_RATIO;
    inputs.columnAppliedVolts = columnAppliedVolts.getValueAsDouble();
    inputs.columnCurrentAmps = columnCurrentAmps.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    m_column.setControl(
        voltageRequest.withOutput(
            MathUtil.clamp(
                volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE)));
  }
}
