package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj.Servo;

public class HoodIOSim implements HoodIO {
  private final Servo leftServo;
  private final Servo rightServo;
  private double targetPosition = 0.5;

  public HoodIOSim(int leftPort, int rightPort) {
    leftServo = new Servo(leftPort);
    rightServo = new Servo(rightPort);

    // Applying your specific bounds
    leftServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    rightServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Servos don't have feedback, so we assume they are where we told them to be
    inputs.targetPosition = this.targetPosition;
    inputs.currentPosition = this.targetPosition;
  }

  @Override
  public void setPosition(double position) {
    targetPosition = position;
    leftServo.set(position);
    rightServo.set(position);
  }
}
