// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveModuleIO.ModuleData;
import frc.robot.subsystems.swerve.sim.GyroIO;
import frc.robot.subsystems.swerve.sim.GyroIO.GyroData;
import frc.robot.subsystems.swerve.sim.GyroSim;
import frc.robot.utils.Constants;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.Constants.DriveConstants;

/***
 * @author Noah Simon
 * @author Rohin Sood
 * @author Raadwan Masum
 * @author Harkirat
 * 
 *         Subsystem class for swerve drive, used to manage four swerve modules
 *         and set their states. Also includes a pose estimator, gyro, and
 *         logging information
 */
public class Swerve extends SubsystemBase {
  private SwerveModule[] modules = new SwerveModule[4];

  private GyroIO gyro;
  private GyroData gyroData;
  // equivilant to a odometer, but also intakes vision
  private SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  private ShuffleData<Double[]> odometryLog = new ShuffleData<Double[]>("swerve", "odometry",
      new Double[] { 0.0, 0.0, 0.0, 0.0 });
  private ShuffleData<Double[]> realStatesLog = new ShuffleData<Double[]>("swerve", "real states",
      new Double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
  private ShuffleData<Double[]> desiredStatesLog = new ShuffleData<Double[]>("swerve", "desired states",
      new Double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
  private ShuffleData<Double> yawLog = new ShuffleData<Double>("swerve", "yaw", 0.0);
  private ShuffleData<Double> pitchLog = new ShuffleData<Double>("swerve", "pitch", 0.0);
  private ShuffleData<Double> rollLog = new ShuffleData<Double>("swerve", "roll", 0.0);

  public Swerve() {
    for (int i = 0; i < 4; i++) {
      modules[i] = new SwerveModule(i);
    }

    if (!Robot.isReal()) {
      gyro = new GyroSim();

      gyroData = new GyroData();
    } else {
      // real swerve module instatiation here

    }

    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.DriveConstants.driveKinematics,
        new Rotation2d(0),
        new SwerveModulePosition[] { modules[0].getPosition(), modules[1].getPosition(),
            modules[2].getPosition(), modules[3].getPosition() },
        new Pose2d(new Translation2d(0, 0), new Rotation2d(0, 0)));

  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.driveKinematics.toSwerveModuleStates(chassisSpeeds);
    // take shortest path to destination
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.DriveConstants.maxSpeedMetersPerSecond);

    // if robot is sim, then use chassisSpeed velocity to calculate rotation
    // otherwise, use real gyro
    if (!Robot.isReal()) {
      double rotationDiffRad = chassisSpeeds.omegaRadiansPerSecond * 0.02;
      gyroData.yawDeg = (gyroData.yawDeg + Units.radiansToDegrees(rotationDiffRad)) % (360);
    } else {

    }

    // 6. Output each module states to wheels
    setModuleStates(moduleStates);
  }

  public ChassisSpeeds getChassisSpeeds() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return DriveConstants.driveKinematics.toChassisSpeeds(states);
  }

  public void updateGyro() {
    gyro.updateData(gyroData);
  }

  public void resetGyro() {
    gyro.resetGyro();
    System.out.println("RESET");
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(gyroData.yawDeg / 180 * Math.PI);
  }

  public Pose2d getPose() {

    Pose2d estimatedPose = swerveDrivePoseEstimator.getEstimatedPosition();

    return new Pose2d(estimatedPose.getTranslation(), getRotation2d());
  }

  public SwerveDrivePoseEstimator getPoseEstimator() {
    return swerveDrivePoseEstimator;
  }

  // Not sure that this works properly
  public void resetOdometry(Pose2d pose) {

    swerveDrivePoseEstimator.resetPosition(getRotation2d(),
        new SwerveModulePosition[] { modules[0].getPosition(), modules[1].getPosition(),
            modules[2].getPosition(), modules[3].getPosition() },
        pose);

  }

  public void updateOdometry() {
    updateGyro();

    swerveDrivePoseEstimator.update(getRotation2d(),
        new SwerveModulePosition[] { modules[0].getPosition(), modules[1].getPosition(),
            modules[2].getPosition(), modules[3].getPosition() });

  }

  public void stopModules() {
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.maxSpeedMetersPerSecond);
    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(desiredStates[i]);
    }
  }

  public double getVerticalTilt() {
    return gyroData.pitch;
  }

  @Override
  public void periodic() {
    updateOdometry();

    for (int i = 0; i < 4; i++) {
      modules[i].periodic();
    }

    Double[] realStates = {
        modules[0].getState().angle.getDegrees(),
        modules[0].getState().speedMetersPerSecond,
        modules[1].getState().angle.getDegrees(),
        modules[1].getState().speedMetersPerSecond,
        modules[2].getState().angle.getDegrees(),
        modules[2].getState().speedMetersPerSecond,
        modules[3].getState().angle.getDegrees(),
        modules[3].getState().speedMetersPerSecond
    };

    Double[] desriedStates = {
        modules[0].getDesiredState().angle.getDegrees(),
        modules[0].getDesiredState().speedMetersPerSecond,
        modules[1].getDesiredState().angle.getDegrees(),
        modules[1].getDesiredState().speedMetersPerSecond,
        modules[2].getDesiredState().angle.getDegrees(),
        modules[2].getDesiredState().speedMetersPerSecond,
        modules[3].getDesiredState().angle.getDegrees(),
        modules[3].getDesiredState().speedMetersPerSecond
    };

    realStatesLog.set(realStates);
    desiredStatesLog.set(desriedStates);
    odometryLog.set(
        new Double[] { getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees() });

    yawLog.set(gyroData.yawDeg);
    pitchLog.set(gyroData.pitch);
    rollLog.set(gyroData.roll);
  }
}