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
  private final InterpolatingDoubleTreeMap m_hoodAngle = new InterpolatingDoubleTreeMap();

  public Hood(HoodIO io) {
    this.m_io = io;
    // First value: distance from robot to hub, in meters | Second value: angle the hood needs to be
    // at to shoot
    // TODO: find and add more values from different distances
    m_hoodAngle.put(1.65, 0.0);
    m_hoodAngle.put(0.0, 0.0);
    m_hoodAngle.put(0.0, 0.0);
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

  public double getHoodAngle(double position) {
    return m_hoodAngle.get(position);
  }
}
