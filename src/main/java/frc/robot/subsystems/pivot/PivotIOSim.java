package frc.robot.subsystems.pivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RobotStateConstants;

public class PivotIOSim implements PivotIO {
  private final SingleJointedArmSim m_armSim;

  public PivotIOSim() {
    System.out.println("[INIT] Creating PivotIOSim");

    m_armSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getKrakenX60(1), PivotConstants.MOI_KG_M2, PivotConstants.GEAR_RATIO),
            null,
            PivotConstants.GEAR_RATIO,
            PivotConstants.LENGTH_M,
            PivotConstants.MIN_ANGLE_RAD,
            PivotConstants.MAX_ANGLE_RAD,
            false,
            PivotConstants.STOW_ANGLE_RAD,
            null);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    m_armSim.update(RobotStateConstants.PERIODIC_LOOP_SEC);
  }
}
