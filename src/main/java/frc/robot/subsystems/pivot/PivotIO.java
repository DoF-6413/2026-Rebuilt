package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  public static class PivotIOInputs {
    public boolean isOK = true;
    public double appliedVoltage = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double relativePosRad = 0.0;
    public double absPositionRad = 0.0; // TODO: verify if needed, may not add encoder
    public double velocityRadPerSec = 0.0;
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

  /**
   * Sets voltage of motor
   *
   * @param volts A value between [-12, 12]
   */
  public default void setVoltage(double volts) {}

  /** Deploys intake to ground position */
  public default void deployPivot() {}
}
