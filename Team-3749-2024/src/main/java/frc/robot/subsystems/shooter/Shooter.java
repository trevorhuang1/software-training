package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ArmFeedforward;
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
  //private SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(0, 1);
  private ArmFeedforward topShooterFF = new ArmFeedforward(Constants.ShintakeConstants.topKs, Constants.ShintakeConstants.topKg, 
  Constants.ShintakeConstants.topKv,Constants.ShintakeConstants.topKa); //ks kg kv ka
  private ArmFeedforward bottomShooterFF = new ArmFeedforward(Constants.ShintakeConstants.bottomKs, Constants.ShintakeConstants.bottomKg, 
  Constants.ShintakeConstants.bottomKv,Constants.ShintakeConstants.bottomKa); //ks kg kv ka
  private double shooterVelocity = 0;
  private static double bottomShooterAbsPos = 0;
  private static double topShooterAbsPos = 0;

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
    double topVoltage =
      shooterController.calculate(
        data.topShooterVelocityRadPerSec,
        shooterVelocity
      ) +
      topShooterFF.calculate(shooterController.getSetpoint(), shooterVelocity);

    double bottomVoltage =
      shooterController.calculate(
        data.bottomShooterVelocityRadPerSec,
        shooterVelocity
      ) +
      bottomShooterFF.calculate(shooterController.getSetpoint(), shooterVelocity);

    setVoltage(topVoltage,bottomVoltage);
  }

  public void setVoltage(double topVolts, double bottomVolts) {
    shooterIO.setVoltage(topVolts, bottomVolts);
  }

  @Override
  public void periodic() {
    
    shooterIO.updateData(data);

    SmartDashboard.putNumber("bottomShooterVolts", data.bottomShooterVolts);
    SmartDashboard.putNumber("bottomShooterVelocityRadPerSec", data.bottomShooterVelocityRadPerSec);
    SmartDashboard.putNumber("bottomShooterTempCelsius", data.bottomShooterTempCelcius);

    SmartDashboard.putNumber("topShooterVolts", data.topShooterVolts);
    SmartDashboard.putNumber("topShooterVelocityRadPerSec", data.topShooterVelocityRadPerSec);
    SmartDashboard.putNumber("topShooterTempCelsius", data.topShooterTempCelcius);

    bottomShooterAbsPos += data.bottomShooterVelocityRadPerSec * 0.02;
    topShooterAbsPos += data.topShooterVelocityRadPerSec * 0.02;
  }

  /*

  private final MutableMeasure<Voltage> identificationTurnVoltageMeasure = mutable(
    Volts.of(0)
  );
  private final MutableMeasure<Angle> identificationTurnDistanceMeasure = mutable(
    Radians.of(0)
  );
  private final MutableMeasure<Velocity<Angle>> identificaitonTurnVelocityMeasure = mutable(
    RadiansPerSecond.of(0)
  );

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
          .motor("shooter-bottom-motor")
          .voltage(
            identificationTurnVoltageMeasure.mut_replace(
              data.bottomShooterVolts,
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
          .motor("shooter-top-motor")
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
  */
}
