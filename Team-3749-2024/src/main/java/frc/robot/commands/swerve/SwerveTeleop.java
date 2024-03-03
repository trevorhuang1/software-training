package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Constants.*;
import java.util.function.Supplier;

/***
 * @author Noah Simon
 * @author Raadwan Masum
 * @author Rohin Sood
 *         Default command to control the SwervedriveSubsystem with joysticks
 */

public class SwerveTeleop extends Command {

  private final Swerve swerve;
  private final Supplier<Double> xSpdFunction, ySpdFunction, xTurningSpdFunction;
  private final SlewRateLimiter driveLimiter, turningLimiter;

  public SwerveTeleop(
      Supplier<Double> xSpdFunction,
      Supplier<Double> ySpdFunction,
      Supplier<Double> xTurningSpdFunction) {
    this.swerve = Robot.swerve;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.xTurningSpdFunction = xTurningSpdFunction;

    // This should be max Acceleration! I think.
    this.driveLimiter = new SlewRateLimiter(DriveConstants.maxAccelerationMetersPerSecondSquared);
    this.turningLimiter = new SlewRateLimiter(
        DriveConstants.maxAngularAccelerationRadiansPerSecondSquared);
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double xMagnitude = xSpdFunction.get();
    double yMagnitude = ySpdFunction.get();
    double turningSpeed = xTurningSpdFunction.get();


    SmartDashboard.putNumberArray(
        "inputs",
        new Double[] { xMagnitude, yMagnitude, turningSpeed });


  
    double linearMagnitude = Math.hypot(xMagnitude, yMagnitude);
    Rotation2d linearDirection = new Rotation2d(xMagnitude, yMagnitude);

    // deadbands
    linearMagnitude = MathUtil.applyDeadband(linearMagnitude, ControllerConstants.deadband);
    turningSpeed = Math.abs(turningSpeed) > ControllerConstants.deadband
        ? turningSpeed
        : 0.0;

    // squaring the inputs for smoother driving at low speeds
    linearMagnitude = Math.copySign(Math.pow(Math.abs(linearMagnitude), 2.5), linearMagnitude);
    turningSpeed = Math.copySign(Math.pow(Math.abs(turningSpeed), 2.5), turningSpeed);

    double driveSpeedMPS = linearMagnitude * DriveConstants.maxSpeedMetersPerSecond;

    turningSpeed =  
        turningSpeed * DriveConstants.maxAngularSpeedRadiansPerSecond;

    SmartDashboard.putNumber("drive speed", driveSpeedMPS);
    SmartDashboard.putNumber("turn speed", turningSpeed);

    // Calcaulate new linear components
    double xSpeed = driveSpeedMPS * Math.cos(linearDirection.getRadians());
    double ySpeed = driveSpeedMPS * Math.sin(linearDirection.getRadians());

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        ySpeed,
        xSpeed,
        turningSpeed,
        swerve.getRotation2d());

    // set chassis speeds
    swerve.setChassisSpeeds(chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
