package frc.robot.subsystems.column;

public interface ColumnIO {

  // @AutoLog
  public static class ColumnIOInputs {
    public double columnAppliedVolts = 0.0;
    public double columnCurrentAmps = 0.0;
    public double columnRPM = 0.0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(ColumnIOInputs inputs) {}

  /**
   * Sets idle mode of motor
   *
   * @param enable {@code}true{@code} to enable brake mode, {@code}false{@code} for coast.
   */
  public default void enableBrakeMode(boolean enable) {}

  /** Run the column at the specified voltage. */
  public default void setVoltage(double volts) {}
}
