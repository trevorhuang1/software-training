package frc.robot.subsystems.swerve.sim;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;

public class GyroSim implements GyroIO {
  public GyroData gyroData = new GyroData();

  public void updateData(GyroData data) {
    gyroData = data;
  }

  public void updateGyroYaw() {
    double angleDiffRad = Robot.swerve.getChassisSpeeds().omegaRadiansPerSecond * 0.02;
    Rotation2d currentRotationDiff = new Rotation2d(angleDiffRad);

    gyroData.yawDeg = (gyroData.yawDeg + currentRotationDiff.getDegrees()) % 360;
  }

  @Override
  public void resetGyro() {
    gyroData = new GyroData();
  }
}
