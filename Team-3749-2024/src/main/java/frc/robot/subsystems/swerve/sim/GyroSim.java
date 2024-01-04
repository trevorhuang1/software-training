package frc.robot.subsystems.swerve.sim;

public class GyroSim implements GyroIO {
  public static GyroData gyroData = new GyroData();

  public void updateData(GyroData data) {
    gyroData = data;
  }

  @Override
  public void resetGyro() {
    gyroData = new GyroData();
  }
}
