package frc.robot.subsystems.intake;
import org.littletonrobotics.junction.AutoLog;


public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double intakeRPM = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeCurrentAmps = 0.0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /**
   * Sets idle mode of motor
   *
   * @param enable {@code}true{@code} to enable brake mode, {@code}false{@code} for coast.
   */
  public default void enableBrakeMode(boolean enable) {}

  /** Run the intake rollers at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run the intake rollers at the specified RPM. */
  public default void setRPM(double velocity) {}
}
