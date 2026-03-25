package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RobotStateConstants;

public class PivotIOTalonFX implements PivotIO {

  // Motor, controller, configurator
  private final TalonFX m_pivotTalonFX = new TalonFX(PivotConstants.CAN_ID, "Drivetrain");
  private final TalonFXConfiguration m_motorConfig =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
          .withVoltage(
              new VoltageConfigs().withPeakReverseVoltage(-RobotStateConstants.MAX_VOLTAGE))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(PivotConstants.CURRENT_LIMIT * 5) // 10
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(PivotConstants.CURRENT_LIMIT)
                  .withSupplyCurrentLimitEnable(true))
          .withSlot0(
              new Slot0Configs()
                  .withKP(PivotConstants.kP)
                  .withKI(PivotConstants.kI)
                  .withKD(PivotConstants.kD)
                  .withKV(PivotConstants.kV));

  // Status signals
  private final StatusSignal<Angle> m_position;
  private final StatusSignal<AngularVelocity> m_velocity;
  private final StatusSignal<Voltage> m_appliedVolts;
  private final StatusSignal<Current> m_currentAmps;

  // Constructor
  public PivotIOTalonFX() {
    System.out.println("[INIT] PivotIOTalonFX");

    m_pivotTalonFX.setPosition(0.0);
    m_pivotTalonFX.optimizeBusUtilization();
    m_pivotTalonFX.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);

    m_pivotTalonFX.getConfigurator().apply(m_motorConfig);

    // Update IOs
    m_position = m_pivotTalonFX.getPosition();
    m_velocity = m_pivotTalonFX.getVelocity();
    m_appliedVolts = m_pivotTalonFX.getMotorVoltage();
    m_currentAmps = m_pivotTalonFX.getStatorCurrent();

    m_position.setUpdateFrequency(RobotStateConstants.UPDATE_FREQUENCY_HZ);
    m_velocity.setUpdateFrequency(RobotStateConstants.UPDATE_FREQUENCY_HZ);
    m_appliedVolts.setUpdateFrequency(RobotStateConstants.UPDATE_FREQUENCY_HZ);
    m_currentAmps.setUpdateFrequency(RobotStateConstants.UPDATE_FREQUENCY_HZ);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.isOK =
        BaseStatusSignal.refreshAll(m_position, m_velocity, m_appliedVolts, m_currentAmps).isOK();

    inputs.relativePosRot = m_position.getValueAsDouble();
    inputs.velocityRadPerSec =
        edu.wpi.first.math.util.Units.rotationsToRadians(m_velocity.getValueAsDouble());
    inputs.appliedVoltage = m_appliedVolts.getValueAsDouble();
    inputs.currentAmps = m_currentAmps.getValueAsDouble();
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    m_pivotTalonFX.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setVoltage(double volts) {
    m_pivotTalonFX.setVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }
}
