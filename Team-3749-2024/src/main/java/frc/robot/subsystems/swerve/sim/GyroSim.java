package frc.robot.subsystems.swerve.sim;

public class GyroSim implements GyroIO {
  public static GyroData gyroData = new GyroData();

  public void updateData(GyroData data) {
    // gyroData = data;

    gyroData.pitch = data.pitch;
    gyroData.roll = data.roll;
    gyroData.yawDeg = data.yawDeg % 360;
  }

  @Override
  public void resetGyro() {
    gyroData = new GyroData();
  }
}
