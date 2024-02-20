package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.*;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShooterIO.ShooterData;
import frc.robot.utils.Constants;

public class Shooter extends SubsystemBase {

  private ShooterIO shooterIO;
  private ShooterData data = new ShooterData();
  private PIDController shooterController = new PIDController(
    Constants.ShintakeConstants.shooterPID.kP,
    Constants.ShintakeConstants.shooterPID.kI,
    Constants.ShintakeConstants.shooterPID.kD
  );
  private SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(0, 1);
  private double shooterVelocity = 0;
  private double armAngle = 0;
  private double antiShooterRubbing = 0; //i dont actually know what angle the arm is at when it's resting

  //and the wheels are touching the body of the robot but thats what this represents

  public Shooter() {
    shooterIO = new ShooterSparkMax();
    if (Robot.isSimulation()) {
      shooterIO = new ShooterSim();
    }
  }

  public void setShooterVelocity(double velocity) {
    this.shooterVelocity = velocity;
  }

  public void moveShooter() {
    if (armAngle == antiShooterRubbing) {
      return; //the wheels might still have inertia from spinning but it shouldn't be a problem?
    }
    double voltage =
      shooterController.calculate(
        data.leftShooterVelocityRadPerSec,
        shooterVelocity
      ) +
      shooterFF.calculate(shooterVelocity);

    setVoltage(voltage);
  }

  public void setVoltage(double volts) {
    shooterIO.setVoltage(volts, volts);
  }

  @Override
  public void periodic() {
    shooterIO.updateData(data);
    SmartDashboard.putNumber("shooterVolts", data.leftShooterVolts);
    SmartDashboard.putNumber("shooterVelocityRadPerSec", data.leftShooterVelocityRadPerSec);
    SmartDashboard.putNumber("shooterTempCelsius", data.leftShooterTempCelcius);

    leftShooterAbsPos += data.leftShooterVelocityRadPerSec * 0.02;
    rightShooterAbsPos += data.rightShooterVelocityRadPerSec * 0.02;
  }

  private final MutableMeasure<Voltage> identificationTurnVoltageMeasure = mutable(
    Volts.of(0)
  );
  private final MutableMeasure<Angle> identificationTurnDistanceMeasure = mutable(
    Radians.of(0)
  );
  private final MutableMeasure<Velocity<Angle>> identificaitonTurnVelocityMeasure = mutable(
    RadiansPerSecond.of(0)
  );
  private static double leftShooterAbsPos = 0;
  private static double rightShooterAbsPos = 0;

  private SysIdRoutine shooterRoutine = new SysIdRoutine(
    // new SysIdRoutine.Config(),
    new SysIdRoutine.Config(
      Volts.per(Seconds).of(1),
      Volts.of(12),
      Seconds.of(10)
    ),
    new SysIdRoutine.Mechanism(
      (Measure<Voltage> volts) -> {
        setVoltage(volts.magnitude());
      },
      log -> {
        log
          .motor("shooter-left-motor")
          .voltage(
            identificationTurnVoltageMeasure.mut_replace(
              data.leftShooterVolts,
              Volts
            )
          )
          .angularPosition(
            identificationTurnDistanceMeasure.mut_replace(
              leftShooterAbsPos,
              Radians
            )
          )
          .angularVelocity(
            identificaitonTurnVelocityMeasure.mut_replace(
              data.leftShooterVelocityRadPerSec,
              RadiansPerSecond
            )
          );

        log
          .motor("shooter-right-motor")
          .voltage(
            identificationTurnVoltageMeasure.mut_replace(
              data.rightShooterVolts,
              Volts
            )
          )
          .angularPosition(
            identificationTurnDistanceMeasure.mut_replace(
              rightShooterAbsPos,
              Radians
            )
          )
          .angularVelocity(
            identificaitonTurnVelocityMeasure.mut_replace(
              data.rightShooterVelocityRadPerSec,
              RadiansPerSecond
            )
          );
      },
      this
    )
  );

  public Command getShooterSysIDQuasistaticForwardTest() {
    return shooterRoutine.quasistatic(Direction.kForward);
  }

  public Command getShooterSysIDQuasistaticReverseTest() {
    return shooterRoutine.quasistatic(Direction.kReverse);
  }

  public Command getShooterSysIDDynamicForwardTest() {
    return shooterRoutine.dynamic(Direction.kForward);
  }

  public Command getShooterSysIDDynamicReverseTest() {
    return shooterRoutine.dynamic(Direction.kReverse);
  }
}
