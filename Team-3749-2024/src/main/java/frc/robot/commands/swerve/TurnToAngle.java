package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.DriveConstants;

public class TurnToAngle extends Command {
  private final Swerve swerve = Robot.swerve;

  private Rotation2d desiredRotation = swerve.getRotation2d();
  private boolean preserveVelocity = false;

  int index;

  private final PIDController pid_turnController = new PIDController(1, 0, 0);
  private final SlewRateLimiter turningLimiter = new SlewRateLimiter(
      DriveConstants.maxAngularSpeedMetersPerSecond);

  /***
   * @param Rotation2d desiredRotation
   */
  public TurnToAngle(Rotation2d desiredRotation) {
    this.desiredRotation = desiredRotation;
  }

  /***
   * @param Rotation2d desiredRotation
   * @param boolean    preserveVelocity
   */
  public TurnToAngle(Rotation2d desiredRotation, boolean preserveVelocity) {
    this.desiredRotation = desiredRotation;
    this.preserveVelocity = preserveVelocity;
  }

  @Override
  public void initialize() {
    pid_turnController.setSetpoint(desiredRotation.getRadians());
    pid_turnController.setTolerance(Constants.DriveConstants.toleranceRad_TurnToAngle);
    pid_turnController.enableContinuousInput(0, 2 * Math.PI);
  }

  @Override
  public void execute() {
    // // calculate the turning speed and clamp the output to the maximum speed that
    // // the motor can turn
    // double turningSpeed = MathUtil.clamp(
    // pid_turnController.calculate(swerve.getRotation2d().getRadians(),
    // desiredRotation.getRadians()),
    // -(Constants.DriveConstants.maxAngularSpeedMetersPerSecond),
    // Constants.DriveConstants.maxAngularSpeedMetersPerSecond);

    // // slowly ramp up speed
    // turningSpeed = turningLimiter.calculate(turningSpeed)
    // * DriveConstants.maxAngularSpeedMetersPerSecond;

    // // create new ChassisSpeeds with the current x/y speeds to prevent
    // interfering
    // // with other things that are happening;
    // ChassisSpeeds newChassisSpeeds;
    // if (preserveVelocity) {
    // ChassisSpeeds currentChassisSpeeds = swerve.getChassisSpeeds();
    // newChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
    // currentChassisSpeeds.vxMetersPerSecond,
    // currentChassisSpeeds.vyMetersPerSecond, turningSpeed,
    // swerve.getRotation2d());
    // } else {
    // newChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, turningSpeed,
    // swerve.getRotation2d());
    // }

    // SmartDashboard.putNumber("puter", turningSpeed);
    // swerve.setChassisSpeeds(newChassisSpeeds);

    index++;
    swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0.5,
        swerve.getRotation2d()));
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("yello", "bye1");
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    // return pid_turnController.atSetpoint();
    return index > 500;
  }
}