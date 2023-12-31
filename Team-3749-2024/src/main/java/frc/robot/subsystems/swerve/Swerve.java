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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.GyroIO.GyroData;
import frc.robot.subsystems.swerve.SwerveModuleIO.ModuleData;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.RobotType;

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
    private SwerveModuleIO[] modules = new SwerveModuleIO[4];
    private ModuleData[] moduleData = new ModuleData[4];

    private GyroIO gyro;
    private GyroData gyroData;
    // equivilant to a odometer, but also intakes vision
    private SwerveDrivePoseEstimator swerveDrivePoseEstimator;


    public Swerve() {

        if (Constants.ROBOT_TYPE == RobotType.SIM) {
            gyro = new GyroIO() {
            };

            gyroData = new GyroData();
            for (int i = 0; i < 4; i++) {
                modules[i] = new SwerveModuleSim();
                moduleData[i] = new ModuleData();
            }
        } else {
            // real swerve module instatiation here

        }

        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.DriveConstants.kDriveKinematics,
                new Rotation2d(0),
                new SwerveModulePosition[] { moduleData[0].position, moduleData[1].position, moduleData[2].position,
                        moduleData[3].position },
                new Pose2d(new Translation2d(0, 0), new Rotation2d(0, 0)));


    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
               // 5. Convert chassis speeds to individual module states
               SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
       
               // 6. Output each module states to wheels
               setModuleStates(moduleStates);
    }

    public ChassisSpeeds getChassisSpeeds(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i<4; i++){
            states[i] = new SwerveModuleState(moduleData[i].driveVelocityMPerSec, new Rotation2d( moduleData[i].turnAbsolutePositionRad));
        }
        return DriveConstants.kDriveKinematics.toChassisSpeeds(states);
    }

    public void resetGyro() {
        gyro.resetGyro();
        System.out.println("RESET");
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(gyroData.yaw / 180 * Math.PI);
    }

    public Pose2d getPose() {

        Pose2d estimatedPose = swerveDrivePoseEstimator.getEstimatedPosition();

        return new Pose2d(estimatedPose.getTranslation(), getRotation2d());
    }

    public SwerveDrivePoseEstimator getPoseEstimator(){
        return swerveDrivePoseEstimator;
    }
 
    public void resetOdometry(Pose2d pose) {


        swerveDrivePoseEstimator.resetPosition(getRotation2d(),
                new SwerveModulePosition[] { moduleData[0].position, moduleData[1].position, moduleData[2].position,
                        moduleData[3].position },
                pose);
    }

    public void updateOdometry() {

        swerveDrivePoseEstimator.update(getRotation2d(),
                new SwerveModulePosition[] { moduleData[0].position, moduleData[1].position, moduleData[2].position,
                        moduleData[3].position });

    }


    public void stopModules() {
        for (SwerveModuleIO module : modules) {
            module.stop();
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.realMaxSpeedMetersPerSecond);
        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(desiredStates[i], moduleData[i]);
        }
    }




    public double getVerticalTilt() {
        return gyroData.pitch;
    }


    @Override
    public void periodic() {
        updateOdometry();

        for (int i = 0; i < 4; i++) {
            modules[i].updateData(moduleData[i]);
        }

        SmartDashboard.putNumberArray("Odometry",
                new double[] { getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees() });
        
        double[] realStates = {
                moduleData[0].turnAbsolutePositionRad / Math.PI * 180,
                moduleData[0].driveVelocityMPerSec,
                moduleData[1].turnAbsolutePositionRad / Math.PI * 180,
                moduleData[1].driveVelocityMPerSec,
                moduleData[2].turnAbsolutePositionRad / Math.PI * 180,
                moduleData[2].driveVelocityMPerSec,
                moduleData[3].turnAbsolutePositionRad / Math.PI * 180,
                moduleData[3].driveVelocityMPerSec };


        double[] theoreticalStates = {
                moduleData[0].theoreticalState.angle.getDegrees(),
                moduleData[0].theoreticalState.speedMetersPerSecond,
                moduleData[1].theoreticalState.angle.getDegrees(),
                moduleData[1].theoreticalState.speedMetersPerSecond,
                moduleData[2].theoreticalState.angle.getDegrees(),
                moduleData[2].theoreticalState.speedMetersPerSecond,
                moduleData[3].theoreticalState.angle.getDegrees(),
                moduleData[3].theoreticalState.speedMetersPerSecond,
        };

        SmartDashboard.putNumberArray("Theoretical States", theoreticalStates);
        SmartDashboard.putNumberArray("Real Staets", realStates);
        SmartDashboard.putNumber("Yaw", getRotation2d().getDegrees());
        SmartDashboard.putNumber("Pitch", getVerticalTilt());
        SmartDashboard.putNumber("Robot Pose X", getPose().getX());
        SmartDashboard.putNumber("Robot Pose Y", getPose().getY());
    }
}