package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotStateConstants;

/**
 * This superstructure implementation is for Talon FXs driving motors like the Falon 500, Kraken
 * X44, or Kraken X60.
 */
public class IntakeIOTalonFX implements IntakeIO {
  // Motor, controller, configurator
  private final TalonFX m_intake = new TalonFX(IntakeConstants.CAN_ID, "Drivetrain");

  // Status signals
  private final StatusSignal<AngularVelocity> intakeVelocityRotPerSec = m_intake.getVelocity();
  private final StatusSignal<Voltage> intakeAppliedVolts = m_intake.getMotorVoltage();
  private final StatusSignal<Current> intakeCurrentAmps = m_intake.getSupplyCurrent();

  private VoltageOut voltageRequest = new VoltageOut(0.0);

  // Constructor
  public IntakeIOTalonFX() {
    System.out.println("[INIT] IntakeIOTalonFX");

    var motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.CURRENT_LIMIT;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = IntakeConstants.ENABLE_CURRENT_LIMIT;
    motorConfig.MotorOutput.NeutralMode =
        IntakeConstants.IS_BRAKE_MODE_ENABLED ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    tryUntilOk(5, () -> m_intake.getConfigurator().apply(motorConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        RobotStateConstants.UPDATE_FREQUENCY_HZ,
        intakeVelocityRotPerSec,
        intakeAppliedVolts,
        intakeCurrentAmps);

    m_intake.setPosition(0.0);
    m_intake.optimizeBusUtilization();
    m_intake.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(intakeVelocityRotPerSec, intakeAppliedVolts, intakeCurrentAmps);

    inputs.intakeRPM = intakeVelocityRotPerSec.getValueAsDouble() * 60 / IntakeConstants.GEAR_RATIO;
    inputs.intakeAppliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.intakeCurrentAmps = intakeCurrentAmps.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    m_intake.setControl(voltageRequest.withOutput(volts));
  }
}
