package frc.robot.subsystems.pivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
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
            DCMotor.getKrakenX60(1),
            PivotConstants.GEAR_RATIO,
            PivotConstants.LENGTH_M,
            PivotConstants.HOMED_ANGLE_ROT,
            PivotConstants.DEPLOYED_ANGLE_ROT,
            false,
            PivotConstants.HOMED_ANGLE_ROT);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    m_armSim.update(RobotStateConstants.PERIODIC_LOOP_SEC);

    inputs.relativePosRot = Units.radiansToRotations(m_armSim.getAngleRads());
    inputs.velocityRadPerSec = m_armSim.getVelocityRadPerSec();
    inputs.currentAmps = m_armSim.getCurrentDrawAmps();
    inputs.isOK = true;
  }
}
