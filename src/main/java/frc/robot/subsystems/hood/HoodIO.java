package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  @AutoLog
  public static class HoodIOInputs {
    public double positionMM = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setPosition(double position) {}
}
