// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.Constants.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.commands.Agitate;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Eject;
import frc.robot.commands.IntakeRetract;
import frc.robot.commands.Launch;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ShooterSpinUp;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.column.Column;
import frc.robot.subsystems.column.ColumnIO;
import frc.robot.subsystems.column.ColumnIOSim;
import frc.robot.subsystems.column.ColumnIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOServo;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.hopper.HopperIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Column m_column;
  private final Hopper m_hopper;
  private final Intake m_intake;
  private final Pivot m_pivot;
  private final Shooter m_shooter;
  private final Hood m_hood;
  private final Vision m_vision;

  // Controller
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVE_CONTROLLER);
  private final CommandXboxController auxController =
      new CommandXboxController(OperatorConstants.AUX_CONTROLLER);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (RobotStateConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        m_column = new Column(new ColumnIOTalonFX());
        m_hopper = new Hopper(new HopperIOTalonFX());
        m_intake = new Intake(new IntakeIOTalonFX());
        m_pivot = new Pivot(new PivotIOTalonFX());
        m_shooter = new Shooter(new ShooterIOTalonFX());
        m_hood =
            new Hood(new HoodIOServo(HoodConstants.leftServoPort, HoodConstants.rightServoPort));

        m_vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera1Name, robotToCamera1),
                new VisionIOPhotonVision(camera2Name, robotToCamera2));

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        m_column = new Column(new ColumnIOSim());
        m_hopper = new Hopper(new HopperIOSim());
        m_intake = new Intake(new IntakeIOSim());
        m_pivot = new Pivot(new PivotIOSim());
        m_shooter = new Shooter(new ShooterIOSim());
        m_hood = new Hood(new HoodIOSim(HoodConstants.leftServoPort, HoodConstants.rightServoPort));

        m_vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose),
                new VisionIOPhotonVisionSim(camera2Name, robotToCamera2, drive::getPose));

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        m_column = new Column(new ColumnIO() {});
        m_hopper = new Hopper(new HopperIO() {});
        m_intake = new Intake(new IntakeIO() {});
        m_pivot = new Pivot(new PivotIO() {});
        m_shooter = new Shooter(new ShooterIO() {});
        m_hood = new Hood(new HoodIO() {});
        m_vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Register Named Commands
    NamedCommands.registerCommand("Intake", new RunIntake(m_intake, m_pivot));
    NamedCommands.registerCommand(
        "SpinUpHub", new ShooterSpinUp(m_shooter, m_hood, () -> drive.getPose(), "hub"));
    NamedCommands.registerCommand(
        "LaunchHub",
        new Launch(m_shooter, m_hopper, m_column, m_hood, () -> drive.getPose(), "hub")
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    NamedCommands.registerCommand(
        "LaunchTrench",
        new Launch(m_shooter, m_hopper, m_column, m_hood, () -> drive.getPose(), "trench")
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    NamedCommands.registerCommand(
        "LaunchTower",
        new Launch(m_shooter, m_hopper, m_column, m_hood, () -> drive.getPose(), "tower")
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    NamedCommands.registerCommand(
        "LaunchAnywhere",
        new Launch(m_shooter, m_hopper, m_column, m_hood, () -> drive.getPose(), "none")
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    NamedCommands.registerCommand("Agitate", new Agitate(m_intake, m_pivot));
    NamedCommands.registerCommand(
        "AutoAim",
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> 0,
            () -> 0,
            () -> {
              Translation2d target =
                  DriverStation.getAlliance()
                      .filter(a -> a == Alliance.Red)
                      .map(a -> FieldConstants.RED_HUB_POSITION)
                      .orElse(FieldConstants.BLUE_HUB_POSITION);

              Pose2d robot = drive.getPose();
              return new Rotation2d(target.getX() - robot.getX(), target.getY() - robot.getY());
            }));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    driverControllerBindings();
    auxControllerBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void driverControllerBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
                drive,
                () -> -1 * driverController.getLeftY(),
                () -> -1 * driverController.getLeftX(),
                () -> -0.65 * driverController.getRightX())
            .withName("JoystickDrive"));

    // Reset gyro to 0° when A button is pressed
    // driverController
    //     .a()
    //     .onTrue(
    //         new InstantCommand(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), Rotation2d.k180deg)),
    //                 drive)
    //             .ignoringDisable(true)
    //             .withName("ResetPose"));

    // Lock wheels in x
    driverController.a().whileTrue(Commands.run(drive::stopWithX, drive).withName("XLock"));
  }

  private void auxControllerBindings() {
    // D-pad Up: Deploy intake pivot and run intake rollers
    auxController.povUp().whileTrue(new RunIntake(m_intake, m_pivot).withName("AuxIntake"));
    // D-pad Down: Retract the intake back up
    auxController
        .povDown()
        .whileTrue(new IntakeRetract(m_intake, m_pivot).withName("IntakeRetract"));
    // Right Trigger: Start spinning up the shooter and adjusting the hood; this should keep
    // adjusting those parameters based on the distance to the hub until the command ends
    auxController
        .rightTrigger()
        .whileTrue(new ShooterSpinUp(m_shooter, m_hood, () -> drive.getPose(), "none"));
    // Right Bumper: Starts up the shooter and sets the hood 0%. This is meant for shooting from
    // right in front of the hub
    auxController
        .rightBumper()
        .whileTrue(
            new Launch(m_shooter, m_hopper, m_column, m_hood, () -> drive.getPose(), "hub")
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                // .alongWith(new Agitate(m_intake, m_pivot))
                .withName("Launch Hub"));
    // Left Trigger: Shoots from anywhere by adjusting the shooter RPM and hood angle
    auxController
        .leftTrigger()
        .whileTrue(
            new Launch(m_shooter, m_hopper, m_column, m_hood, () -> drive.getPose(), "none")
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                // .alongWith(new Agitate(m_intake, m_pivot)) TODO: uncomment this once done tuning
                .withName("Launching from anywhere"));
    // Left Bumper: auto aims the robot towards the hub
    auxController
        .leftBumper()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -driverController.getLeftY(), // forward
                    () -> -driverController.getLeftX(), // strafe
                    () -> {
                      Translation2d target =
                          DriverStation.getAlliance()
                              .filter(a -> a == Alliance.Red)
                              .map(a -> FieldConstants.RED_HUB_POSITION)
                              .orElse(FieldConstants.BLUE_HUB_POSITION);

                      Pose2d robot = drive.getPose();
                      return new Rotation2d(
                          target.getX() - robot.getX(), target.getY() - robot.getY());
                    })
                .withName("Auto aiming"));
    auxController
        .button(7)
        .whileTrue(
            new Launch(m_shooter, m_hopper, m_column, m_hood, () -> drive.getPose(), "tower")
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .alongWith(new Agitate(m_intake, m_pivot))
                .withName("Launching from tower"));
    auxController
        .button(8)
        .whileTrue(
            new Launch(m_shooter, m_hopper, m_column, m_hood, () -> drive.getPose(), "trench")
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .alongWith(new Agitate(m_intake, m_pivot))
                .withName("Launching from trench"));

    // Controlling hood
    // Button Y: sets the hood to the maximum position
    auxController
        .y()
        .onTrue(
            new InstantCommand(() -> m_hood.setPosition(HoodConstants.K_MAX_POSITION))
                .withName("HoodMax"));
    // Button A: sets the hood to the minimum position
    auxController
        .a()
        .onTrue(
            new InstantCommand(() -> m_hood.setPosition(HoodConstants.K_MIN_POSITION))
                .withName("HoodMin"));

    // Button X: switch to x pattern
    auxController.x().whileTrue(Commands.run(drive::stopWithX, drive).withName("XLock"));
    // Button B: Eject balls through the intake
    auxController.b().whileTrue(new Eject(m_column, m_hopper, m_intake).withName("Out-taking"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Command hoodDown() {
    return new InstantCommand(() -> m_hood.setPosition(HoodConstants.K_MIN_POSITION));
  }
}
