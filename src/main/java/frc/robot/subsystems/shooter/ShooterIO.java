package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double shooterAppliedVolts = 0.0;
    public double shooterCurrentAmps = 0.0;
    public double shooterTempCelsius = 0.0;
    public double shooterRPM = 0.0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /**
   * Sets idle mode of motor
   *
   * @param enable {@code}true{@code} to enable brake mode, {@code}false{@code} for coast.
   */
  public default void enableBrakeMode(boolean enable) {}

  /** Run the shooter at the specified voltage. */
  public default void setVoltage(double volts) {}
}
