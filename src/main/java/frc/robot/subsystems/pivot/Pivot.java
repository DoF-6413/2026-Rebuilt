package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
  private final PivotIO m_io;
  private final PivotIOInputsAutoLogged m_inputs = new PivotIOInputsAutoLogged();

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
   * Sets voltage of motor
   *
   * @param volts A value between [-12, 12]
   */
  public void setVoltage(double volts) {
    m_io.setVoltage(volts);
  }

  public void deployPivot() {
    m_io.deployPivot();
  }
}
