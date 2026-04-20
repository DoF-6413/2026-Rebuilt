package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double middleShooterAppliedVolts = 0.0;
    public double middleShooterCurrentAmps = 0.0;
    public double middleShooterRPS = 0.0;
    public double rightShooterAppliedVolts = 0.0;
    public double rightShooterCurrentAmps = 0.0;
    public double rightShooterRPS = 0.0;
    public double leftShooterAppliedVolts = 0.0;
    public double leftShooterCurrentAmps = 0.0;
    public double leftShooterRPS = 0.0;
    public double velocityErrorRPS = 0.0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /**
   * Sets idle mode of motor
   *
   * @param enable {@code}true{@code} to enable brake mode, {@code}false{@code} for coast.
   */
  public default void enableBrakeMode(boolean enable) {}

  /** Run the shooters at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run the shooters at the specified speed. */
  public default void setVelocity(double velocityRPM) {}
}
