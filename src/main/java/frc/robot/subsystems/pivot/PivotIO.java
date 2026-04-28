package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public boolean isOK = true;
    public double appliedVoltage = 0.0;
    public double currentAmps = 0.0;
    public double relativePosRot = 0.0;
    public double velocityRadPerSec = 0.0;
    public boolean atTarget = false;
  }

  /**
   * Updates logged inputs for Pivot. Must be called periodically.
   *
   * @param inputs AutoLog inputs
   */
  public default void updateInputs(PivotIOInputs inputs) {}

  /**
   * Sets idle mode of motor
   *
   * @param enable {@code}true{@code} to enable brake mode, {@code}false{@code} for coast.
   */
  public default void enableBrakeMode(boolean enable) {}

  public default void setVoltage(double volts) {}
}
