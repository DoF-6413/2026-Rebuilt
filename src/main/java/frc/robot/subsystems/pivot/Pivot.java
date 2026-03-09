package frc.robot.subsystems.pivot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIO m_io;
  private final PivotIOInputsAutoLogged m_inputs = new PivotIOInputsAutoLogged();
  private double m_targetPosition = PivotConstants.HOMED_ANGLE_ROT;

  /**
   * Constructs a new {@link Pivot} instance.
   *
   * <p>This creates a new Climber {@link SubsystemBase} object with the given IO implementation
   * which determines whether the methods and inputs are initialized with the real, sim, or replay
   * code.
   *
   * @param io {@link PivotIO} implementation of the current robot mode.
   */
  public Pivot(PivotIO io) {
    System.out.println("[INIT] Pivot");

    // Initialize the IO implementation
    m_io = io;
  }

  @Override
  public void periodic() {
    // Update and log inputs
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Pivot", m_inputs);
  }

  public boolean isAtTarget() {
    return m_inputs.relativePosRot - m_targetPosition
        < Units.degreesToRotations(PivotConstants.TOLERANCE_DEG);
  }

  public double getAngle() {
    return m_inputs.relativePosRot;
  }

  /**
   * Sets idle mode of motor
   *
   * @param enable {@code}true{@code} to enable brake mode, {@code}false{@code} for coast.
   */
  public void enableBrakeMode(boolean enable) {
    m_io.enableBrakeMode(enable);
  }

  /**
   * Sets the pivot to the desired angle
   *
   * @param angle angle for the pivot to go to
   */
  public void setPosition(double angle) {
    m_io.setPosition(angle);
  }

  public void setVoltage(double volts) {
    m_io.setVoltage(volts);
  }
}
