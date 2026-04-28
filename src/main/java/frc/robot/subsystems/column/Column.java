package frc.robot.subsystems.column;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Column extends SubsystemBase {
  private final ColumnIO m_io;
  private final ColumnIOInputsAutoLogged m_inputs = new ColumnIOInputsAutoLogged();

  public Column(ColumnIO io) {
    System.out.println("[INIT] Column");
    m_io = io;
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Column", m_inputs);
  }

  public void setVoltage(double volts) {
    m_io.setVoltage(volts);
  }

  public void enableBrakeMode(boolean enable) {
    m_io.enableBrakeMode(enable);
  }

  /** Returns RPM of column motor */
  public double getRPM() {
    return m_inputs.columnRPM;
  }
}
