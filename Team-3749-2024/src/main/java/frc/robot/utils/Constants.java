package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.util.PIDConstants;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class Constants {

  public static final class Sim {
    public static final double loopPeriodSec = 0.02;;

  }

  public static enum RobotState {
    STOW(0, 0),
    SPEAKER(30, 0),
    AMP(80, 0);

    public double armPosRad;
    public double shintakePosRad;

    private RobotState(double armPosRad, double shintakePosRad) {
      this.armPosRad = armPosRad;
      this.shintakePosRad = shintakePosRad;
    }
  }

  public static final class ArmConstants {

    public static final int leftID = 15;
    public static final int rightID = 16;
    // inverse gear ratio * 1min/60sec * 2PI to get rad/sec
    public static final double gearRatio = 200.0;
    public static final int encoderID = 7;
    public static final double encoderOffsetRad = 0;

    // Control - PID, FF, and Trapezoidal Constraints
    private static final PIDConstants simPID = new PIDConstants(0, 0, 0); // 2.2,0,0
    private static final PIDConstants realPID = new PIDConstants(1.25, 0, 0);

    public static final PIDConstants PID = Robot.isReal() ? realPID : simPID;


    private static final double simkS = 0.0;
    private static final double simkG = 0.203;// stick arm at 0 degrees, tune till it doesnt move

    private static final double simkV = 6.616; // max volts - kG / max velocity
    private static final double simkA = 0; // (max volts - kG - vel@maxacceleration*kV )/max acceleration


    private static final double realkS = 0.1085;
    private static final double realkG = 0.2435; 
    private static final double realkV = 3.95;
    private static final double realkA = 0.23;

    public static final double kS = Robot.isReal() ? realkS : simkS;
    public static final double kG = Robot.isReal() ? realkG : simkG;
    public static final double kV = Robot.isReal() ? realkV : simkV;
    public static final double kA = Robot.isReal() ? realkA : simkA;

    // private static final Constraints simConstraints = new Constraints(2.36,
    // 71.58);
    private static final Constraints simConstraints = new Constraints(1.783, 89.175);

    private static final Constraints realConstraints = new Constraints(2.662,
        9);
    public static final Constraints constraints = Robot.isReal() ? realConstraints : simConstraints;

    // Field Parameters
    public static final double armHeightOffset = 0.2159; 
    public static final double armLengthOffset = 0.2286;
    public static final double armLength = 0.93;
    public static final double shooterVelocity = 10.0; // NOTE: likely will vary, might need to pass as parameter

    // Field Parameters
    public static final double speakerHeight = 2.05; // likely thing you'll need to tune
    public static final double minDistance = 0.9; // / NOTE: Not percise, please check

    // Calcuation stuff
    // TODO: adjust stage margin stuff and distance margin/max angle (ask Bailey)
    public static final double distMargin = 0.25; // Half a meter is kind of a lot, don't you think?
    public static final double maxAngle = 42.109;
    public static final double maxAngleRad = Math.toRadians(maxAngle);

    // Important Field Coordinates (everything converted from inches to meters)
    //NOTE: may need to adjust slightly to make sure code works properly (some inpercision in measurements)
    public static final Translation2d redSpeakerPosition = new Translation2d(Units.inchesToMeters(653.2), Units.inchesToMeters(218.64)); // rounded need to change
    public static final Translation2d blueSpeakerPosition = new Translation2d(0, Units.inchesToMeters(218.64)); // rounded need to change

    // public static final Translation2d[] redStagePoints = {new Translation2d(13.8986, 4.1056), new Translation2d(10.4648, 2.1752),new Translation2d(10.4648, 6.0361)};
    // public static final Translation2d[] blueStagePoints = {new Translation2d(16.5928-redStagePoints[0].getX(), redStagePoints[0].getY()),new Translation2d(16.5928-redStagePoints[1].getX(), redStagePoints[1].getY()),new Translation2d(16.5928-redStagePoints[2].getX(), redStagePoints[2].getY())};

    public static final double stageMargin = 20; // margin in inches
    public static final Translation2d[] redStagePoints = {new Translation2d(Units.inchesToMeters(125.01 + stageMargin),Units.inchesToMeters(161.64)), new Translation2d(Units.inchesToMeters(231.2 + stageMargin), Units.inchesToMeters(104.64 + stageMargin)), new Translation2d(Units.inchesToMeters(231.2 + stageMargin), Units.inchesToMeters(218.64 + stageMargin))};
    public static final Translation2d[] blueStagePoints = {new Translation2d(653.2-redStagePoints[0].getX(), redStagePoints[0].getY()), new Translation2d(653.2-redStagePoints[1].getX(), redStagePoints[1].getY()), new Translation2d(653.2-redStagePoints[2].getX(), redStagePoints[2].getY())};
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
    }

    // Distance between right and left wheels
    public static final double trackWidth = Units.inchesToMeters(17.5);
    // Distance between front and back wheels
    public static final double wheelBase = Units.inchesToMeters(17.5);
    public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2, trackWidth / 2), // front left
        new Translation2d(wheelBase / 2, -trackWidth / 2), // front right
        new Translation2d(-wheelBase / 2, trackWidth / 2), // back left
        new Translation2d(-wheelBase / 2, -trackWidth / 2)); // back right

    public static final int[] driveMotorPorts = { 3, 5, 7, 9 }; // FL, FR, BL, BR
    public static final int[] turningMotorPorts = { 4, 6, 8, 10 }; // FL, FR, BL, BR

    public static final boolean[] driveEncoderReversed = { true, false, true, false };
    public static final boolean[] turningEncoderReversed = { false, false, false, false };

    public static final int[] absoluteEncoderPorts = { 11, 12, 13, 14 };

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
    public static double kP_PathPlannerDrive = 8.8;
    public static double kD_PathPlannerDrive = 0;

    public static double kP_PathPlannerTurn = 0.9;
    public static double kD_PathPlannerTurn = 0;

    public static PIDConstants drivePIDConstants = new PIDConstants(Constants.AutoConstants.kP_PathPlannerDrive, 0,
        Constants.AutoConstants.kD_PathPlannerDrive);
    public static PIDConstants turnPIDConstants = new PIDConstants(Constants.AutoConstants.kP_PathPlannerTurn, 0,
        Constants.AutoConstants.kD_PathPlannerTurn);

    public static HolonomicPathFollowerConfig cfgHolonomicFollower = new HolonomicPathFollowerConfig(
        // in your Constants class
        drivePIDConstants,
        turnPIDConstants,
        Constants.DriveConstants.maxSpeedMetersPerSecond, // Max module speed, in m/s
        Math.sqrt(2 * (DriveConstants.trackWidth * DriveConstants.trackWidth)), // Drivetrain radius
        new ReplanningConfig() // Default path replanning config. See the API for the
    // options here
    );

    public static PathConstraints defaultPathConstraints = new PathConstraints(
        Constants.DriveConstants.maxSpeedMetersPerSecond,
        Constants.DriveConstants.maxAccelerationMetersPerSecondSquared,
        Constants.DriveConstants.maxAngularSpeedRadiansPerSecond,
        Constants.DriveConstants.maxAngularAccelerationRadiansPerSecondSquared);
  }

  public static final class ControllerConstants {
    public static final double deadband = 0.1;
  }
}