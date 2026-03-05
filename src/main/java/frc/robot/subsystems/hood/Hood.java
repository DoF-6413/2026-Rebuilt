package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  public void setPosition(double position) {
    double clamped =
        MathUtil.clamp(position, HoodConstants.K_MIN_POSITION, HoodConstants.K_MAX_POSITION);
    io.setPosition(clamped);
  }

  public boolean isAtTarget() {
    return Math.abs(inputs.targetPosition - inputs.currentPosition) < HoodConstants.K_TOLERANCE;
  }

  public Command positionCommand(double position) {
    return runOnce(() -> setPosition(position)).andThen(Commands.waitUntil(this::isAtTarget));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Current Position", () -> inputs.currentPosition, null);
    builder.addDoubleProperty("Target Position", () -> inputs.targetPosition, this::setPosition);
  }
}
