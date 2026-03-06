package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.subsystems.pivot.PivotIOTalonFX.Position;

public class PivotIOTalonFX implements PivotIO {

  public enum Position {
    HOMED(110),
    STOWED(100),
    INTAKE(-4),
    AGITATE(20);

    private final double degrees;

    private Position(double degrees) {
      this.degrees = degrees;
    }

    public Angle angle() {
      return Degrees.of(degrees);
    }
  }

  // Motor, controller, configurator
  private final TalonFX m_pivotTalonFX = new TalonFX(PivotConstants.CAN_ID, "Drivetrain");
  private final TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();

  // Used for setting the position of the pivot motor, taken from WCP
  private final MotionMagicVoltage m_pivotMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0); 

  // Status signals
  private StatusSignal<Voltage> m_appliedVolts;
  private StatusSignal<Current> m_currentAmps;
  private StatusSignal<Temperature> m_tempCelsius;
  private StatusSignal<Angle> m_positionRot;
  private StatusSignal<AngularVelocity> m_velocityRotPerSec;

  // Constructor
  public PivotIOTalonFX() {
    System.out.println("[INIT] PivotIOTalonFX");

    m_motorConfig
        .MotorOutput
        .withInverted(
            PivotConstants.IS_INVERTED
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive)
        .withNeutralMode(
            PivotConstants.IS_BRAKE_MODE_ENABLED ? NeutralModeValue.Brake : NeutralModeValue.Coast)
        .withControlTimesyncFreqHz(RobotStateConstants.UPDATE_FREQUENCY_HZ);

    m_pivotTalonFX.setPosition(0.0);
    m_pivotTalonFX.optimizeBusUtilization();
    m_pivotTalonFX.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);

    m_motorConfig
        .CurrentLimits
        .withStatorCurrentLimit(PivotConstants.CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(PivotConstants.ENABLE_CURRENT_LIMIT);

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
    inputs.relativePosRad = m_positionRot.getValueAsDouble();
    inputs.absPositionRad = m_positionRot.getValueAsDouble();
    inputs.velocityRadPerSec = m_velocityRotPerSec.getValueAsDouble();
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    m_pivotTalonFX.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  private boolean isPositionWithinTolerance() {
    final Angle currentPosition = m_pivotTalonFX.getPosition().getValue();
    final Angle targetPosition = m_pivotMotionMagicRequest.getPositionMeasure();
    return currentPosition.isNear(targetPosition, Degrees.of(5));
  }

  public void set(Position position) {
    m_pivotTalonFX.setControl(
      m_pivotMotionMagicRequest
        .withPosition(position.angle())
    );
  }

  @Override
  public void deployPivot() {
    m_pivotTalonFX.setPosition(Units.radiansToRotations(PivotConstants.MIN_ANGLE_RAD));
  }


  public Command agitateCommand() {
        return runOnce(() -> set(Speed.INTAKE))
            .andThen(
                Commands.sequence(
                    runOnce(() -> set(Position.AGITATE)),
                    Commands.waitUntil(this::isPositionWithinTolerance),
                    runOnce(() -> set(Position.INTAKE)),
                    Commands.waitUntil(this::isPositionWithinTolerance)
                )
                .repeatedly()
            )
            .handleInterrupt(() -> {
                set(Position.INTAKE);
                set(Speed.STOP);
            });
    }
}
