package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RobotStateConstants;

public class PivotIOTalonFX implements PivotIO {

  // Motor, controller, configurator
  private final TalonFX m_pivotTalonFX = new TalonFX(PivotConstants.CAN_ID, "Drivetrain");
  private final PositionVoltage m_positionVoltageRequest = new PositionVoltage(0.0);
  private final TalonFXConfiguration m_motorConfig =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
          .withVoltage(new VoltageConfigs().withPeakReverseVoltage(-2))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(PivotConstants.CURRENT_LIMIT * 5) // 10
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(PivotConstants.CURRENT_LIMIT)
                  .withSupplyCurrentLimitEnable(true))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(100)
                  .withMotionMagicAcceleration(100))
          .withSlot0(
              new Slot0Configs()
                  .withKP(PivotConstants.kP)
                  .withKI(PivotConstants.kI)
                  .withKD(PivotConstants.kD)
                  .withKV(PivotConstants.kV));

  // Used for setting the position of the pivot motor, taken from WCP
  private final MotionMagicVoltage m_pivotMotionMagicRequest =
      new MotionMagicVoltage(0).withSlot(0);

  // Status signals
  private StatusSignal<Voltage> m_appliedVolts;
  private StatusSignal<Current> m_currentAmps;
  private StatusSignal<Temperature> m_tempCelsius;
  private StatusSignal<Angle> m_positionRot;
  private StatusSignal<AngularVelocity> m_velocityRotPerSec;

  // Constructor
  public PivotIOTalonFX() {
    System.out.println("[INIT] PivotIOTalonFX");

    m_pivotTalonFX.setPosition(0.0);
    m_pivotTalonFX.optimizeBusUtilization();
    m_pivotTalonFX.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);

    m_pivotTalonFX.getConfigurator().apply(m_motorConfig);

    // Update IOs
    m_positionRot = m_pivotTalonFX.getPosition();
    m_velocityRotPerSec = m_pivotTalonFX.getVelocity();
    m_appliedVolts = m_pivotTalonFX.getMotorVoltage();
    m_currentAmps = m_pivotTalonFX.getStatorCurrent();
    m_tempCelsius = m_pivotTalonFX.getDeviceTemp();

    m_positionRot.setUpdateFrequency(RobotStateConstants.UPDATE_FREQUENCY_HZ);
    m_velocityRotPerSec.setUpdateFrequency(RobotStateConstants.UPDATE_FREQUENCY_HZ);
    m_appliedVolts.setUpdateFrequency(RobotStateConstants.UPDATE_FREQUENCY_HZ);
    m_currentAmps.setUpdateFrequency(RobotStateConstants.UPDATE_FREQUENCY_HZ);
    m_tempCelsius.setUpdateFrequency(RobotStateConstants.UPDATE_FREQUENCY_HZ);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.isOK =
        BaseStatusSignal.refreshAll(
                m_positionRot, m_velocityRotPerSec, m_appliedVolts, m_currentAmps, m_tempCelsius)
            .isOK();

    inputs.appliedVoltage = m_appliedVolts.getValueAsDouble();
    inputs.currentAmps = m_currentAmps.getValueAsDouble();
    inputs.tempCelsius = m_tempCelsius.getValueAsDouble();
    inputs.relativePosRot = m_positionRot.getValueAsDouble();
    inputs.velocityRadPerSec = m_velocityRotPerSec.getValueAsDouble();
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    m_pivotTalonFX.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setPosition(double angle) {
    m_pivotTalonFX.setControl(m_positionVoltageRequest.withPosition(angle));
  }

  @Override
  public void setVoltage(double volts) {
    m_pivotTalonFX.setVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }
}
