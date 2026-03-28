package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO m_io;
  private final HoodIOInputsAutoLogged m_inputs = new HoodIOInputsAutoLogged();

  public Hood(HoodIO io) {
    this.m_io = io;
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Hood", m_inputs);
  }

  public void setPosition(double position) {
    double clamped =
        MathUtil.clamp(position, HoodConstants.K_MIN_POSITION, HoodConstants.K_MAX_POSITION);
    m_io.setPosition(clamped);
  }

  public boolean isAtTarget() {
    return Math.abs(m_inputs.targetPosition - m_inputs.currentPosition) < HoodConstants.K_TOLERANCE;
  }

  public Command positionCommand(double position) {
    return runOnce(() -> setPosition(position)).andThen(Commands.waitUntil(this::isAtTarget));
  }
}
