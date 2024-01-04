package frc.robot.subsystems.swerve.sim;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroSim implements GyroIO {
  public GyroData gyroData = new GyroData();

  public void updateData(GyroData data) {
    gyroData = data;
  }

  public void updateGyroYaw(Rotation2d currentRotationDiff) {
    gyroData.yawDeg = (gyroData.yawDeg + currentRotationDiff.getDegrees()) % 360;
  }

  @Override
  public void resetGyro() {
    gyroData = new GyroData();
  }
}
