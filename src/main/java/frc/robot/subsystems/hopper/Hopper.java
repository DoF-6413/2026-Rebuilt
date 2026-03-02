// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotStateConstants;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private final HopperIO m_io;
  private final HopperIOInputsAutoLogged m_inputs = new HopperIOInputsAutoLogged();

  /** Creates a new Hopper. */
  public Hopper(HopperIO io) {
    System.out.println("[INIT] Hopper");

    // Initialize the IO implementation
    m_io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Hopper", m_inputs);
  }

  public void setVoltage(double volts) {
    m_io.setVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }
}
