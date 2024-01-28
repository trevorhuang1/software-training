package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class Constants {

  public static final boolean ROBOT_IS_REAL = Robot.isReal();

  public static final class Sim {
    public static final double loopPeriodSec = 0.02;

    public static final class PIDValues {
      // will eventally be easier to change values from here than poke around through
      // files
      public static double kP_teleopTurn = 1.3;
      public static double kD_teleopTurn = 0.0;

      public static double kP_MiscDrive = 0.42;
      public static double kD_MiscDrive = 0.02;
      public static double kP_MiscTurn = 0.15;
      public static double kD_MiscTurn = 0.003;

      public static double kP_TurnToAngle = 0.15;
      public static double kD_TurnToAngle = 0.008;
    };

  }

  public static final class ModuleConstants {
    public static final double wheelDiameterMeters = Units.inchesToMeters(4);
    public static final double driveMotorGearRatio = 6.75;
    public static final double turnMotorGearRatio = 12.8;
    public static final double kPTurningReal = 2.25;
    public static final double kPDrivingReal = 0.0;
    public static final double kVDrivingReal = 2.5;
    public static final double kSDrivingReal = 0.0;

    public static final double kPTurningSim = 4;
    public static final double kVDrivingSim = 3.19;
    public static final double kSDrivingSim = 0.0;
    public static final double kPDrivingSim = 0.0;

  }

  public static final class DriveConstants {

    // Distance between right and left wheels
    public static final double trackWidth = Units.inchesToMeters(17.5);
    // Distance between front and back wheels
    public static final double wheelBase = Units.inchesToMeters(17.5);
    public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2, trackWidth / 2), // front left
        new Translation2d(wheelBase / 2, -trackWidth / 2), // front right
        new Translation2d(-wheelBase / 2, trackWidth / 2), // back left
        new Translation2d(-wheelBase / 2, -trackWidth / 2)); // back right

    public static final int[] driveMotorPorts = { 1, 3, 7, 5 }; // FL, FR, BL, BR
    public static final int[] turningMotorPorts = { 2, 4, 8, 6 }; // FL, FR, BL, BR

    public static final boolean[] driveEncoderReversed = { true, false, true, false };
    public static final boolean[] turningEncoderReversed = { false, false, false, false };

    public static final int[] absoluteEncoderPorts = { 9, 10, 11, 12 };

    public static final boolean[] driveAbsoluteEncoderReversed = { false, false, false, false };

    public static final double[] driveAbsoluteEncoderOffsetDeg = { 130.34, 107.75, 61.70, 168.75 };

    private static final double realMaxSpeedMetersPerSecond = 3.707;
    private static final double realMaxAngularSpeedRadiansPerSecond = 11.795;
    private static final double realMaxAccelerationMetersPerSecondSquared = 7.800;
    private static final double realMaxAngularAccelerationRadiansPerSecondSquared = 30.273;

    public static final int driveMotorStallLimit = 30;
    public static final int driveMotorFreeLimit = 60;
    public static final int turnMotorStallLimit = 30;
    public static final int turnMotorFreeLimit = 60;

    private static final double simMaxSpeedMetersPerSecond = 3.707;
    private static final double simMaxAngularSpeedRadiansPerSecond = 11.795;
    private static final double simMaxAccelerationMetersPerSecondSquared = 2.5;
    private static final double simMaxAngularAccelerationRadiansPerSecondSquared = 30.273;

    // private static final double simMaxSpeedMetersPerSecond = 2.65;
    // private static final double simMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
    // private static final double simMaxAccelerationMetersPerSecondSquared = 2.5;
    // private static final double simMaxAngularAccelerationRadiansPerSecondSquared
    // = 2 * 2 * Math.PI;

    private static final double simMaxMotorVoltage = 12.0;
    private static final double realMaxMotorVoltage = 12.0;

    public static final double maxMotorVolts = Robot.isReal() ? DriveConstants.realMaxMotorVoltage
        : DriveConstants.simMaxMotorVoltage;

    public static final double maxSpeedMetersPerSecond = Robot.isReal()
        ? DriveConstants.realMaxSpeedMetersPerSecond
        : DriveConstants.simMaxSpeedMetersPerSecond;

    public static final double maxAngularSpeedRadiansPerSecond = Robot.isReal()
        ? DriveConstants.realMaxAngularSpeedRadiansPerSecond
        : DriveConstants.simMaxAngularSpeedRadiansPerSecond;

    public static final double maxAccelerationMetersPerSecondSquared = Robot.isReal()
        ? DriveConstants.realMaxAccelerationMetersPerSecondSquared
        : DriveConstants.simMaxAccelerationMetersPerSecondSquared;

    public static final double maxAngularAccelerationRadiansPerSecondSquared = Robot.isReal()
        ? DriveConstants.realMaxAngularAccelerationRadiansPerSecondSquared
        : DriveConstants.simMaxAngularAccelerationRadiansPerSecondSquared;

    public static final double toleranceM_Misc = 0.02;
    public static final double toleranceRad_Misc = Math.PI / 750;

    public static final Pose2d fieldStartingPose = new Pose2d(1, 1, Rotation2d.fromDegrees(0));
  }

  public static final class AutoConstants {
    public static double kP_PathPlannerDrive = 9.8;
    public static double kD_PathPlannerDrive = 0;

    public static double kP_PathPlannerTurn = 0.9;
    public static double kD_PathPlannerTurn = 0;

    public static PIDConstants driveConstants = new PIDConstants(Constants.AutoConstants.kP_PathPlannerDrive, 0,
        Constants.AutoConstants.kD_PathPlannerDrive);
    public static PIDConstants turnConstants = new PIDConstants(Constants.AutoConstants.kP_PathPlannerTurn, 0,
        Constants.AutoConstants.kD_PathPlannerTurn);

    public static HolonomicPathFollowerConfig cfgHolonomicFollower = new HolonomicPathFollowerConfig(
        // in your Constants class
        driveConstants,
        turnConstants,
        Constants.DriveConstants.maxSpeedMetersPerSecond, // Max module speed, in m/s
        Math.sqrt(2 * (DriveConstants.trackWidth * DriveConstants.trackWidth)), // Drivetrain radius
        new ReplanningConfig() // Default path replanning config. See the API for the
    // options here
    );

    private static PathConstraints sim = new PathConstraints(
        1.5,
        Constants.DriveConstants.maxAccelerationMetersPerSecondSquared,
        Constants.DriveConstants.maxAngularSpeedRadiansPerSecond,
        Constants.DriveConstants.maxAngularAccelerationRadiansPerSecondSquared);

    private static PathConstraints real = new PathConstraints(
        Constants.DriveConstants.maxSpeedMetersPerSecond,
        Constants.DriveConstants.maxAccelerationMetersPerSecondSquared,
        Constants.DriveConstants.maxAngularSpeedRadiansPerSecond,
        Constants.DriveConstants.maxAngularAccelerationRadiansPerSecondSquared);

    public static PathConstraints defaultPathConstraints = Robot.isReal() ? real : sim;
  }

  public static final class ControllerConstants {
    public static final double deadband = 0.1;
  }
}