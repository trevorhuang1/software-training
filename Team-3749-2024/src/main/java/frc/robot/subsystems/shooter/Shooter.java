package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShooterIO.ShooterData;
import frc.robot.utils.ShuffleData;

public class Shooter extends SubsystemBase {

  private ShooterIO shooterIO;
  private ShooterData data = new ShooterData();

  private PIDController bottomFeedback = new PIDController(
      ShooterConstants.shooterBottomPID.kP,
      ShooterConstants.shooterBottomPID.kI,
      ShooterConstants.shooterBottomPID.kD);

  private PIDController topFeedback = new PIDController(
      ShooterConstants.shooterTopPID.kP,
      ShooterConstants.shooterTopPID.kI,
      ShooterConstants.shooterTopPID.kD);

  private SimpleMotorFeedforward topShooterFF = new SimpleMotorFeedforward(
      0,
      ShooterConstants.topkV,
      0);

  private SimpleMotorFeedforward bottomShooterFF = new SimpleMotorFeedforward(
      0,
      ShooterConstants.bottomkV,
      0);

  private ShuffleData<Double> topShooterVelocityLog = new ShuffleData<Double>(this.getName(), "top shooter velocity",
      0.0);
  private ShuffleData<Double> bottomShooterVelocityLog = new ShuffleData<Double>(this.getName(),
      "bottom shooter velocity", 0.0);
  private ShuffleData<Double> topShootervoltageLog = new ShuffleData<Double>(this.getName(), "top shooter voltage",
      0.0);
  private ShuffleData<Double> bottomShootervoltageLog = new ShuffleData<Double>(this.getName(),
      "bottom shooter voltage", 0.0);
  private ShuffleData<Double> topShootercurrentLog = new ShuffleData<Double>(this.getName(), "top shooter current",
      0.0);
  private ShuffleData<Double> bottomShootercurrentLog = new ShuffleData<Double>(this.getName(),
      "bottom shooter current", 0.0);

  public Shooter() {
    shooterIO = new ShooterSparkMax();
    if (Robot.isSimulation()) {
      shooterIO = new ShooterSim();
    }
  }

  // ShuffleData<Double> kVData = new ShuffleData<Double>(this.getName(), "kVData", 0.0);
  // ShuffleData<Double> kPData = new ShuffleData<Double>(this.getName(), "kPData", 0.0);
  // ShuffleData<Double> velData = new ShuffleData<Double>(this.getName(), "velData", 0.0);

  public void setShooterVelocity(double velocityRadPerSec) {

    double topVoltage = topFeedback.calculate(
        data.topShooterVelocityRadPerSec,
        velocityRadPerSec) +
        topShooterFF.calculate(velocityRadPerSec);

    double bottomVoltage = bottomFeedback.calculate(
        data.bottomShooterVelocityRadPerSec,
        velocityRadPerSec) +
        bottomShooterFF.calculate(velocityRadPerSec);

    // double topVoltage = kVData.get() *velocityRadPerSec;
    // double bottomVoltage = kVData.get()
    setVoltage(topVoltage, bottomVoltage);
  }

  public void setVoltage(double topVolts, double bottomVolts) {
    shooterIO.setVoltage(topVolts, bottomVolts);
  }

  public void stop() {
    shooterIO.setVoltage(0, 0);

  }

  @Override
  public void periodic() {

    shooterIO.updateData(data);

    topShooterVelocityLog.set(data.topShooterVelocityRadPerSec);
    bottomShooterVelocityLog.set(data.bottomShooterVelocityRadPerSec);

    topShootervoltageLog.set(data.topShooterVolts);
    bottomShootervoltageLog.set(data.bottomShooterVolts);

    topShootercurrentLog.set(data.topShooterCurrentAmps);
    bottomShootercurrentLog.set(data.bottomShooterCurrentAmps);
  }

}
