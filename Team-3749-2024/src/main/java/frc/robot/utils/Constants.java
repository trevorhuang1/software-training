package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class Constants {

  public static enum RobotType {
    REAL,
    SIM
  }

  public static final RobotType ROBOT_TYPE = Robot.isReal() ? RobotType.REAL : RobotType.SIM;

  public static final class Sim {
    public static final double loopPeriodSec = 0.02;
  }



  public static final class ControllerConstants {
    public static final double deadband = 0.1;
  }

  public static final class WristConstants {
    public static final int wristId = 17;
    public static final double gearRatio = 1;

    private static final PIDConstants simPID = new PIDConstants(35, 0, 1);
    private static final PIDConstants realPID = new PIDConstants(1, 0, 0);

    private static final Constraints simConstraint = new Constraints(Math.PI, 2 * Math.PI); // we stealing from arm with
                                                                                            // this one
    private static final Constraints realConstraint = new Constraints(Math.PI, Math.PI);

    public static final Constraints trapezoidConstraint = Robot.isReal() ? realConstraint : simConstraint;
    public static final PIDConstants PID = Robot.isReal() ? realPID : simPID;

    // thanks arm (robbery)
    public static final double simkS = 0.0;
    public static final double simkG = .33775;// stick arm at 0 degrees, tune till it doesnt move (ok it moves but it
                                              // should be fine it'll take like 3 years to move 1 degree)
    public static final double simkV = 6.616; // max volts - kG / max velocity
    public static final double simkA = 0; // (max volts - kG - vel@maxacceleration*kV )/max acceleration

    public static final double kYIntercept = 0.9808;
    public static final double kBar = 0.0289;
    public static final double kArm = -0.026;
    public static final double kBarSquared = 0.0;
    public static final double kArmSquared = 0.0;

    public static final double groundGoal = Math.toRadians(-40);
    public static final double stowGoal = 0;
    public static final double wristOffsetRad = Units.degreesToRadians(247.95);
  }

  public static final class ShooterConstants {
    public static final int shooterBottomId = 19;
    public static final int shooterTopId = 18;

    public static final double shooterVelocityRadPerSec = 100;

    private static final PIDConstants simShooterTopPID = new PIDConstants(0, 0, 0);
    private static final PIDConstants realShooterTopPID = new PIDConstants(0.0, 0, 0);
    public static final PIDConstants shooterTopPID = Robot.isReal() ? realShooterTopPID : simShooterTopPID;

    private static final PIDConstants simShooterBottomPID = new PIDConstants(0, 0, 0);
    private static final PIDConstants realShooterBottomPID = new PIDConstants(0, 0, 0);
    public static final PIDConstants shooterBottomPID = Robot.isReal() ? realShooterBottomPID : simShooterBottomPID;

    public static final double topkV = 0.020; // volts/radPerSec

    public static final double bottomkV = 0.;

  }

  public static final class IntakeConstants {

    public static final int intakeId = 20;

    public static final double gearRatio = 5;
    public static final double intakeVelocity = 75;
    public static final double outtakeVelocity = -30;
    private static final PIDConstants simIntakePID = new PIDConstants(0, 0, 0);
    private static final PIDConstants realIntakePID = new PIDConstants(0.01, 0, 0);
    public static final PIDConstants intakePID = Robot.isReal() ? realIntakePID : simIntakePID;
    public static final double kV = 0.101;
  }

  public static class VisionConstants {
    // See
    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    // page 208
    public static final double targetWidth = Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters

    // See
    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    // page 197
    public static final double targetHeight = Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters

    public static final Transform3d cam_to_robot = new Transform3d(
        new Translation3d(0, 0, -Units.inchesToMeters(15.25)), new Rotation3d());

    public static final Transform3d robot_to_cam = cam_to_robot.inverse();

    public static final int reflective_tape_pipeline_index = 0;
    public static final int apriltag_pipeline_index = 1;

    public static final double camera_height = Units.inchesToMeters(20); // meters
    public static final double camera_yaw = 0;
    public static final double camera_pitch = 0;
    public static final double camera_roll = 0;

    // msg from Noah: I forget what these do
    public static final double retro_cam_offset = 0.56;
    public static final double apriltag_cam_offset = 3.1;

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
    private static final PIDConstants simPID = new PIDConstants(2.2, 0, 0); // 2.2,0,0
    private static final PIDConstants realPID = new PIDConstants(0, 0, 0);

    public static final PIDConstants PID = Robot.isReal() ? realPID : simPID;

    public static final int leftID = 0;
    public static final int rightID = 1;
    // inverse gear ratio * 1min/60sec * 2PI to get rad/sec
    public static final double relativeEncoderVelocityConversionFactor = 1 / 150 * 1 / 60 * Math.PI * 2;
    public static final int encoderID = 2;
    public static final double encoderOffsetRad = 0;

    // Control - PID, FF, and Trapezoidal Constraints

    private static final double simkS = 0.0;
    private static final double simkG = 0.203;// stick arm at 0 degrees, tune till it doesnt move

    private static final double simkV = 6.616; // max volts - kG / max velocity
    private static final double simkA = 0; // (max volts - kG - vel@maxacceleration*kV )/max acceleration

    private static final double realkS = 0;
    private static final double realkG = 0;
    private static final double realkV = 0;
    private static final double realkA = 0;

    public static final double kS = Robot.isReal() ? realkS : simkS;
    public static final double kG = Robot.isReal() ? realkG : simkG;
    public static final double kV = Robot.isReal() ? realkV : simkV;
    public static final double kA = Robot.isReal() ? realkA : simkA;

    // private static final Constraints simConstraints = new Constraints(2.36,
    // 71.58);
    private static final Constraints simConstraints = new Constraints(1.783, 89.175);

    private static final Constraints realConstraints = new Constraints(Math.PI, 2 * Math.PI);
    public static final Constraints constraints = Robot.isReal() ? realConstraints : simConstraints;

    // Field Parameters
    public static final double armHeightOffset = 0.0; // how high up the arm is, NOTE: need to find
    public static final double armLength = 0.93;
    public static final double shooterVelocity = 10.0; // NOTE: likely will vary, might need to pass as
                                                       // parameter

    // NOTE: Not percise
    // Field Parameters
    public static final double speakerHeight = 2.05;
    public static final double minDistance = 0.9;

    // Calcuation stuff
    public static final double distMargin = 0.5;
    public static final double maxAngle = 42.109;
    public static final double maxAngleRad = Math.toRadians(maxAngle);
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
    public static final double trackWidth = Units.inchesToMeters(19.5);
    // Distance between front and back wheels
    public static final double wheelBase = Units.inchesToMeters(19.5);
    public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2, trackWidth / 2), // front left
        new Translation2d(wheelBase / 2, -trackWidth / 2), // front right
        new Translation2d(-wheelBase / 2, trackWidth / 2), // back left
        new Translation2d(-wheelBase / 2, -trackWidth / 2)); // back right

    public static final int[] driveMotorPorts = { 3, 5, 7, 9 }; // FL, FR, BL, BR
    public static final int[] turningMotorPorts = { 4, 6, 8, 10 }; // FL, FR, BL, BR
    public static final int[] absoluteEncoderPorts = { 11, 12, 13, 14 };

    public static final boolean[] driveMotorReversed = { true, false, false, true };
    public static final boolean[] turningMotorReversed = { false, false, false, false };
    public static final boolean[] driveAbsoluteEncoderReversed = { false, false, false, false };
    public static final double[] absoluteEncoderOffsetDeg = { -18.896-0.088, -292.730+44, -11.891-216.361, -68 +20.275};

    // public static final double[] absoluteEncoderOffsetDeg = { -275, -48, 0, 263
    // };

    public static final int driveMotorStallLimit = 20;
    public static final int driveMotorFreeLimit = 40;
    public static final int turnMotorStallLimit = 20;
    public static final int turnMotorFreeLimit = 40;

    private static final double realMaxSpeedMetersPerSecond = 3.707;
    private static final double realMaxAngularSpeedRadiansPerSecond = 11.795;
    private static final double realMaxAccelerationMetersPerSecondSquared = 7.800;
    private static final double realMaxAngularAccelerationRadiansPerSecondSquared = 30.273;

    private static final double simMaxSpeedMetersPerSecond = 3.707;
    private static final double simMaxAngularSpeedRadiansPerSecond = 11.795;
    private static final double simMaxAccelerationMetersPerSecondSquared = 2.5;
    private static final double simMaxAngularAccelerationRadiansPerSecondSquared = 30.273;

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

    public static final Pose2d fieldStartingPose = new Pose2d(1.37, 4.51, Rotation2d.fromDegrees(-22.62));
    // will eventally be easier to change values from here than poke around through
    // files
    public static double kP_teleopTurn = 1.3;
    public static double kD_teleopTurn = 0.0;

    public static double kP_TurnToAngle = 0.15;
    public static double kD_TurnToAngle = 0.008;

    public static double kP_MiscDrive = 0.42;
    public static double kD_MiscDrive = 0.02;

    public static double kP_MiscTurn = 0.15;
    public static double kD_MiscTurn = 0.003;
  }

  public static final class AutoConstants {
    public static double kP_PathPlannerDrive = 13.854869643983458;
    public static double kD_PathPlannerDrive = 0;

    public static double kP_PathPlannerTurn = 2;
    public static double kD_PathPlannerTurn = 0.4;

    public static PIDConstants drivePIDConstants = new PIDConstants(
        Constants.AutoConstants.kP_PathPlannerDrive,
        0.09090823761513889,
        Constants.AutoConstants.kD_PathPlannerDrive);
    public static PIDConstants turnPIDConstants = new PIDConstants(
        Constants.AutoConstants.kP_PathPlannerTurn, 0,
        Constants.AutoConstants.kD_PathPlannerTurn);

    public static HolonomicPathFollowerConfig cfgHolonomicFollower = new HolonomicPathFollowerConfig(
        // in your Constants class
        drivePIDConstants,
        turnPIDConstants,
        Constants.DriveConstants.maxSpeedMetersPerSecond, // Max module speed, in m/s
        Math.sqrt(2 * (DriveConstants.trackWidth * DriveConstants.trackWidth)), // Drivetrain
                                                                                // radius
        new ReplanningConfig() // Default path replanning config. See the API for the
    // options here
    );

    public static PathConstraints defaultPathConstraints = new PathConstraints(
        Constants.DriveConstants.maxSpeedMetersPerSecond,
        Constants.DriveConstants.maxAccelerationMetersPerSecondSquared,
        Constants.DriveConstants.maxAngularSpeedRadiansPerSecond,
        Constants.DriveConstants.maxAngularAccelerationRadiansPerSecondSquared);
  }

}