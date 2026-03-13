package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.RobotStateConstants;

public class HopperIOTalonFX implements HopperIO {
  // Motor, controller, configurator
  private final TalonFX m_hopper = new TalonFX(HopperConstants.CAN_ID, "Drivetrain");
  private final TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();

  // Status signals
  private final StatusSignal<AngularVelocity> hopperVelocityRotPerSec = m_hopper.getVelocity();
  private final StatusSignal<Voltage> hopperAppliedVolts = m_hopper.getMotorVoltage();
  private final StatusSignal<Current> hopperCurrentAmps = m_hopper.getSupplyCurrent();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  // Constructor
  public HopperIOTalonFX() {
    System.out.println("[INIT] HopperIOTalonFX");

    m_motorConfig
        .MotorOutput
        .withInverted(
            HopperConstants.IS_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(
            HopperConstants.IS_BRAKE_MODE_ENABLED ? NeutralModeValue.Brake : NeutralModeValue.Coast)
        .withControlTimesyncFreqHz(RobotStateConstants.UPDATE_FREQUENCY_HZ);
    m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = HopperConstants.ENABLE_CURRENT_LIMIT;
    m_motorConfig.CurrentLimits.SupplyCurrentLimit = HopperConstants.CURRENT_LIMIT;
    m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = HopperConstants.ENABLE_CURRENT_LIMIT;
    m_motorConfig.CurrentLimits.StatorCurrentLimit = HopperConstants.CURRENT_LIMIT;

    m_hopper.getConfigurator().apply(m_motorConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        RobotStateConstants.UPDATE_FREQUENCY_HZ,
        hopperAppliedVolts,
        hopperCurrentAmps,
        hopperVelocityRotPerSec);

    m_hopper.setPosition(0.0);
    m_hopper.optimizeBusUtilization();
    m_hopper.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    BaseStatusSignal.refreshAll(hopperVelocityRotPerSec, hopperAppliedVolts, hopperCurrentAmps);

    inputs.hopperRPM = hopperVelocityRotPerSec.getValueAsDouble() * 60 / HopperConstants.GEAR_RATIO;
    inputs.hopperAppliedVolts = hopperAppliedVolts.getValueAsDouble();
    inputs.hopperCurrentAmps = hopperCurrentAmps.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    m_hopper.setControl(voltageRequest.withOutput(volts));
  }
}
