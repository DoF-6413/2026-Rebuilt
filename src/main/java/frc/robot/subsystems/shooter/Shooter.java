// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  // Update IOs
  private final ShooterIO m_io;
  private final ShooterIOInputsAutoLogged m_inputs = new ShooterIOInputsAutoLogged();

  // Controllers
  // private final PIDController m_PIDController;
  // private final SimpleMotorFeedforward m_FFController;
  private double m_setpoint = ShooterConstants.SETPOINT_2_RPM;
  // private boolean m_enablePID = true;
  // private boolean m_enableTesting = true;

  public Shooter(ShooterIO io) {
    System.out.println("[INIT] Shooter");
    m_io = io;

    // m_PIDController =
    //     new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
    // m_PIDController.setSetpoint(m_setpoint);
    // m_PIDController.setTolerance(ShooterConstants.TOLERANCE_RPM);
    // m_FFController =
    //     new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV,
    // ShooterConstants.kA);

    // Puts adjustable PID and FF values onto the SmartDashboard for testing mode
    SmartDashboard.putBoolean("Tuning/Shooter/EnableTuning", false);
    SmartDashboard.putNumber("Tuning/Shooter/Setpoint", ShooterConstants.SETPOINT_2_RPM);
    // SmartDashboard.putNumber("Tuning/Shooter/PID/kP", ShooterConstants.kP);
    // SmartDashboard.putNumber("Tuning/Shooter/PID/kI", ShooterConstants.kI);
    // SmartDashboard.putNumber("Tuning/Shooter/PID/kD", ShooterConstants.kD);
    // SmartDashboard.putNumber("Tuning/Shooter/FF/kS", ShooterConstants.kS);
    // SmartDashboard.putNumber("Tuning/Shooter/FF/kV", ShooterConstants.kV);
    // SmartDashboard.putNumber("Tuning/Shooter/FF/kA", ShooterConstants.kA);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Shooter", m_inputs);

    // Sets the voltage of the Shooter Motors using PID
    // if (m_enablePID) {
    // setVoltage(
    //     m_PIDController.calculate(m_inputs.middleShooterRPM)
    //         + (m_FFController.calculate(m_inputs.middleShooterRPM)
    //             / RobotStateConstants.MAX_VOLTAGE));
    // }

    // Enables test values along with printing other useful measurements for testing
    // if (m_enableTesting) {
    //   updatePID();
    //   updateFF();

    // SmartDashboard.putNumber("Tuning/Shooter/Error RPM", m_PIDController.getPositionError());
    // SmartDashboard.putBoolean("Tuning/Shooter/AtSetpoint", atSetpoint());
    // }

    SmartDashboard.putNumber("Shooter/RPM???", m_setpoint);
  }

  // public void setVoltage(double volts) {
  //   m_io.setVoltage(volts);
  // }

  public void setVelocity(double velocityRPM) {
    m_io.setVelocity(velocityRPM);
  }

  public void enableBrakeMode(boolean enable) {
    m_io.enableBrakeMode(enable);
  }

  public double getVelocity() {
    return m_inputs.middleShooterRPS;
  }

  //   /**
  //    * Sets the gains for the PID controller.
  //    *
  //    * @param kP Proportional gain value.
  //    * @param kI Integral gain value.
  //    * @param kD Derivative gain value.
  //    */
  //   public void setPID(double kP, double kI, double kD) {
  //     // m_PIDController.setPID(kP, kI, kD);
  //   }

  //   /**
  //    * Sets the gains for the PID controller.
  //    *
  //    * @param kP Proportional gain value.
  //    * @param kI Integral gain value.
  //    * @param kD Derivative gain value.
  //    */
  //   public void setFF(double kS, double kV, double kA) {
  //     // m_FFController.setKs(kS);
  //     // m_FFController.setKv(kV);
  //     // m_FFController.setKa(kA);
  //   }

  //   /** Update PID gains for the Shooter motor from SmartDashboard inputs. */
  //   private void updatePID() {
  //     // If any value on SmartDashboard changes, update the gains
  //     if (ShooterConstants.SETPOINT_RPM
  //             != SmartDashboard.getNumber("Tuning/Shooter/Setpoint",
  // ShooterConstants.SETPOINT_RPM)
  //         || ShooterConstants.kP
  //             != SmartDashboard.getNumber("Tuning/Shooter/PID/kP", ShooterConstants.kP)
  //         || ShooterConstants.kI
  //             != SmartDashboard.getNumber("Tuning/Shooter/PID/kI", ShooterConstants.kI)
  //         || ShooterConstants.kD
  //             != SmartDashboard.getNumber("Tuning/Shooter/PID/kD", ShooterConstants.kD)) {
  //       ShooterConstants.SETPOINT_RPM =
  //           SmartDashboard.getNumber("Tuning/Shooter/Setpoint", ShooterConstants.SETPOINT_RPM);
  //       ShooterConstants.kP = SmartDashboard.getNumber("Tuning/Shooter/PID/kP",
  // ShooterConstants.kP);
  //       ShooterConstants.kI = SmartDashboard.getNumber("Tuning/Shooter/PID/kI",
  // ShooterConstants.kI);
  //       ShooterConstants.kD = SmartDashboard.getNumber("Tuning/Shooter/PID/kD",
  // ShooterConstants.kD);
  //       // Sets the new gains
  //       // m_PIDController.setSetpoint(m_setpoint);
  //       setPID(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
  //     }
  //   }

  //   /** Update FF gains for the Shooter motor from SmartDashboard inputs. */
  //   private void updateFF() {
  //     // If any value on SmartDashboard changes, update the gains
  //     if (ShooterConstants.kS != SmartDashboard.getNumber("Tuning/Shooter/FF/kS",
  // ShooterConstants.kS)
  //         || ShooterConstants.kV
  //             != SmartDashboard.getNumber("Tuning/Shooter/FF/kV", ShooterConstants.kV)
  //         || ShooterConstants.kA
  //             != SmartDashboard.getNumber("Tuning/Shooter/FF/kA", ShooterConstants.kA)) {
  //       ShooterConstants.kS = SmartDashboard.getNumber("Tuning/Shooter/FF/kS",
  // ShooterConstants.kS);
  //       ShooterConstants.kV = SmartDashboard.getNumber("Tuning/Shooter/FF/kV",
  // ShooterConstants.kV);
  //       ShooterConstants.kA = SmartDashboard.getNumber("Tuning/Shooter/FF/kA",
  // ShooterConstants.kA);
  //       // Sets the new gains
  //       setFF(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);
  //     }
  //   }

  //   /**
  //    * Enable closed loop PID control for the ALGAE Pivot.
  //    *
  //    * @param enable {@code true} to enable PID control, {@code false} to disable.
  //    */
  //   public void enablePID(boolean enable) {
  //     m_enablePID = enable;
  //   }

  //   /** Returns if Shooter RPM is at setpoint */
  //   public boolean atSetpoint() {
  //     // return m_PIDController.atSetpoint();
  //     return true;
  //   }
}
