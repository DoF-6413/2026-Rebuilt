package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {

  @AutoLog
  public static class HopperIOInputs {
    public double hopperRPM = 0.0;
    public double hopperAppliedVolts = 0.0;
    public double hopperCurrentAmps = 0.0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(HopperIOInputs inputs) {}

  /** Run the hopper motor at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run the hopper motor at the specified RPM. */
  public default void setRPM(double velocity) {}
}
