package frc.robot.utils;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
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
import java.util.HashMap;
import java.util.Map;

public class Constants {

    public static enum RobotType {
        REAL,
        SIM
    }

    public static final RobotType ROBOT_TYPE = Robot.isReal()
            ? RobotType.REAL
            : RobotType.SIM;

    public static final class Sim {

        public static final double loopPeriodSec = 0.02;
    }

    public static final class ControllerConstants {

        public static final double deadband = 0.125;
    }

    public static final class WristConstants {

        public static final int wristId = 17;
        public static final double gearRatio = 1;

        private static final PIDConstants simPID = new PIDConstants(35, 0, 1);

        private static final PIDConstants realPID = new PIDConstants(0.35, 0, 0); // 1

        public static final PIDConstants PID = Robot.isReal() ? realPID : simPID;

        private static final Constraints simConstraint = new Constraints(
                Math.PI,
                2 * Math.PI); // we stealing from arm with
        // this one
        private static final Constraints realConstraint = new Constraints(
                Math.PI,
                Math.PI);

        public static final Constraints trapezoidConstraint = Robot.isReal()
                ? realConstraint
                : simConstraint;

        // thanks arm (robbery)
        public static final double simkS = 0.0;
        public static final double simkG = .33775;
        public static final double simkV = 6.616;
        public static final double simkA = 0;

        public static final double realkS = 0.1;
        public static final double realkVForward = 1.27; // radians
        public static final double realkVBackward = 1.075; // radians

        public static final double kYIntercept = 0.2500;
        public static final double kBar = 1.2051170442485861;
        public static final double kBarSquared = -1.8766465126496676;
        public static final double kBarCubed = 0.48841621426762893;
        public static final double kArm = -0.08453955731495624;
        public static final double kArmSquared = -0.020323277876598187;
        public static final double kBarArm = 3.7070918562599866;
        public static final double kBarSquaredArm = -2.6560157465636793;
        public static final double kBarCubedArm = 0.47800105908391155;
        public static final double kBarArmSquared = -2.132368908319232;
        public static final double kBarSquaredArmSquared = 2.148987748059586;
        public static final double kBarCubedArmSquared = -0.496866520545201;

        public static final double groundGoalRad = Units.degreesToRadians(155);
        public static final double stowGoalRad = 0;
        public static final double wristOffsetRad = Units.degreesToRadians(252.8);
    }

    public static final class ShooterConstants {

        public static final int shooterBottomId = 19;
        public static final int shooterTopId = 18;

        public static final double shooterVelocityRadPerSec = 100;

        private static final PIDConstants simShooterTopPID = new PIDConstants(
                0,

                0,
                0);
        private static final PIDConstants realShooterTopPID = new PIDConstants(
                0.004,
                0,
                0);
        public static final PIDConstants shooterTopPID = Robot.isReal()
                ? realShooterTopPID
                : simShooterTopPID;

        private static final PIDConstants simShooterBottomPID = new PIDConstants(
                0,
                0,
                0);
        private static final PIDConstants realShooterBottomPID = new PIDConstants(
                0.004,
                0,
                0);
        public static final PIDConstants shooterBottomPID = Robot.isReal()
                ? realShooterBottomPID
                : simShooterBottomPID;

        public static final double topkV = 0.0201; // volts/radPerSec

        public static final double bottomkV = 0.0201;
    }

    public static final class IntakeConstants {

        public static final int intakeId = 20;

        public static final double gearRatio = 5;
        public static final double intakeVelocity = 75;
        public static final double outtakeVelocity = -30;
        private static final PIDConstants simIntakePID = new PIDConstants(0, 0, 0);
        private static final PIDConstants realIntakePID = new PIDConstants(
                0.01,
                0,
                0);
        public static final PIDConstants intakePID = Robot.isReal()
                ? realIntakePID
                : simIntakePID;
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
                new Translation3d(0, 0, -Units.inchesToMeters(15.25)),
                new Rotation3d());

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

        public static final int leftID = 15;
        public static final int rightID = 16;
        // inverse gear ratio * 1min/60sec * 2PI to get rad/sec
        public static final double gearRatio = 200.0;

        public static final double sprocketRatio = 64.0 / 24.0;
        public static final int encoderID = 7;
        public static final double encoderOffsetRad = 0;

        // Control - PID, FF, and Trapezoidal Constraints
        private static final PIDConstants simPID = new PIDConstants(0, 0, 0); // 2.2,0,0
        private static final PIDConstants realPID = new PIDConstants(1.25, 0, 0);

        public static final PIDConstants PID = Robot.isReal() ? realPID : simPID;

        private static final double simkS = 0.0;
        private static final double simkG = 0.203; // stick arm at 0 degrees, tune till it doesnt move

        private static final double simkV = 6.616; // max volts - kG / max velocity
        private static final double simkA = 0; // (max volts - kG - vel@maxacceleration*kV )/max acceleration

        private static final double realkS = 0.12;
        private static final double realkG = 0.247;
        private static final double realkV = 3.94;
        private static final double realkA = 0.23;

        public static final double kS = Robot.isReal() ? realkS : simkS;
        public static final double kG = Robot.isReal() ? realkG : simkG;
        public static final double kV = Robot.isReal() ? realkV : simkV;
        public static final double kA = Robot.isReal() ? realkA : simkA;
        public static final double deployedKG = 0.57;
        public static final double deployedKP = 1.32;

        // private static final Constraints simConstraints = new Constraints(2.36,
        // 71.58);
        private static final Constraints simConstraints = new Constraints(
                1.783,
                89.175);

        private static final Constraints realConstraints = new Constraints(
                2.662,
                9);
        public static final Constraints constraints = Robot.isReal()
                ? realConstraints
                : simConstraints;

        // Field Parameters
        public static final double armHeightOffset = 0.2159;
        public static final double armLengthOffset = 0.2286;
        public static final double armLength = 0.93;
        public static final double shooterVelocity = 20.0; // NOTE: likely will vary, might need to pass as parameter

        // Field Parameters
        public static final double speakerHeight = 2.05; // likely thing you'll need to tune
        public static final double minDistance = 0.9; // / NOTE: Not percise, please check

        // Calcuation stuff
        // Max angle??? (ask Bailey)
        public static final double distMargin = 0.25; // Half a meter is kind of a lot, don't you think?
        public static final double maxAngle = 42.109;
        public static final double maxAngleRad = Math.toRadians(maxAngle);

        // Important Field Coordinates (everything converted from inches to meters)
        // NOTE: may need to adjust slightly to make sure code works properly (some
        // inpercision in measurements)
        public static final Translation2d redSpeakerPosition = new Translation2d(
                Units.inchesToMeters(653.2),
                Units.inchesToMeters(218.64)); // rounded need to change
        public static final Translation2d blueSpeakerPosition = new Translation2d(
                0,
                Units.inchesToMeters(218.64)); // rounded need to change

        public static final double stageMargin = 20; // margin in inches
        public static final Translation2d[] redStagePoints = {
                new Translation2d(
                        Units.inchesToMeters(125.01 + stageMargin),
                        Units.inchesToMeters(161.64)),
                new Translation2d(
                        Units.inchesToMeters(231.2 + stageMargin),
                        Units.inchesToMeters(104.64 + stageMargin)),
                new Translation2d(
                        Units.inchesToMeters(231.2 + stageMargin),
                        Units.inchesToMeters(218.64 + stageMargin))
        };
        public static final Translation2d[] blueStagePoints = {
                new Translation2d(
                        653.2 - redStagePoints[0].getX(),
                        redStagePoints[0].getY()),
                new Translation2d(
                        653.2 - redStagePoints[1].getX(),
                        redStagePoints[1].getY()),
                new Translation2d(
                        653.2 - redStagePoints[2].getX(),
                        redStagePoints[2].getY())
        };
    }

    public static final class ModuleConstants {

        public static final double wheelDiameterMeters = Units.inchesToMeters(4); 
        // 2.04
        // 2.006
        // I think 2 is good lol 

        public static final double driveMotorGearRatio = 6.75;
        public static final double turnMotorGearRatio = 12.8;

        private static final double kPTurningReal = 3.75;
        private static final double kDTurningReal = 0;

        private static final double kPDrivingReal = 0.27;
        private static final double kSDrivingReal = 0.26;
        private static final double kVDrivingReal = 2.765; 
        private static final double kADrivingReal = 0.0;

        private static final double kPTurningSim = 4;
        private static final double kPDrivingSim = 0.0;
        private static final double kVDrivingSim = 3.19;
        private static final double kSDrivingSim = 0.0;

        public static double kPturning = Robot.isReal()
                ? kPTurningReal
                : kPTurningSim;
        public static double kDTurning = Robot.isReal()
                ? kDTurningReal
                : 0;
        public static double kPDriving = Robot.isReal()
                ? kPDrivingReal
                : kPDrivingSim;
        public static double kSDriving = Robot.isReal()
                ? kSDrivingReal
                : kSDrivingSim;
        public static double kVDriving = Robot.isReal()
                ? kVDrivingReal
                : kVDrivingSim;
        public static double kADriving = Robot.isReal()
                ? kADrivingReal
                : 0;

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

        public static final boolean[] driveMotorReversed = {
                false,
                true,
                false,
                true
        };
        public static final boolean[] turningMotorReversed = {
                false,
                false,
                false,
                false
        };
        public static final boolean[] driveAbsoluteEncoderReversed = {
                false,
                false,
                false,
                false
        };
        public static final double[] absoluteEncoderOffsetDeg = {
                517.148,
                291.973,
                843.662,
                667.617
        };

        // public static final double[] absoluteEncoderOffsetDeg = { -275, -48, 0, 263
        // };

        public static final int driveMotorStallLimit = 25;
        public static final int driveMotorFreeLimit = 40;
        public static final int turnMotorStallLimit = 25;
        public static final int turnMotorFreeLimit = 40;

        private static final double realMaxSpeedMetersPerSecond = 3.48; // 3.48
        private static final double realMaxAccelerationMetersPerSecondSquared = 1.92; // 1.92
        private static final double realTeleopMaxAngularSpeedRadiansPerSecond = 6.5;
        private static final double realMaxAngularSpeedRadiansPerSecond = 9.847;

        // these should be different from the teleop ones 
        private static final double realMaxAngularAccelerationRadiansPerSecondSquared = 9.02;

        private static final double simMaxSpeedMetersPerSecond = 3.707;
        private static final double simMaxAccelerationMetersPerSecondSquared = 2.5;
        private static final double simMaxAngularSpeedRadiansPerSecond = 9.94;
        private static final double simMaxAngularAccelerationRadiansPerSecondSquared = 9;

        private static final double simMaxMotorVoltage = 12.0;
        private static final double realMaxMotorVoltage = 12.0;

        public static final double maxMotorVolts = Robot.isReal()
                ? DriveConstants.realMaxMotorVoltage
                : DriveConstants.simMaxMotorVoltage;

        public static final double maxSpeedMetersPerSecond = Robot.isReal()
                ? DriveConstants.realMaxSpeedMetersPerSecond
                : DriveConstants.simMaxSpeedMetersPerSecond;

        public static final double maxAngularSpeedRadiansPerSecond = Robot.isReal()
                ? DriveConstants.realTeleopMaxAngularSpeedRadiansPerSecond
                : DriveConstants.simMaxAngularSpeedRadiansPerSecond;

        public static final double maxAccelerationMetersPerSecondSquared = Robot.isReal()
                ? DriveConstants.realMaxAccelerationMetersPerSecondSquared
                : DriveConstants.simMaxAccelerationMetersPerSecondSquared;

        public static final double maxAngularAccelerationRadiansPerSecondSquared = Robot.isReal()
                ? DriveConstants.realMaxAngularAccelerationRadiansPerSecondSquared
                : DriveConstants.simMaxAngularAccelerationRadiansPerSecondSquared;

        public static final double toleranceM_Misc = 0.02;
        public static final double toleranceRad_Misc = Math.PI / 750;

        public static final Pose2d fieldStartingPose = new Pose2d(
                1.37,
                4.51,
                Rotation2d.fromDegrees(-22.62));
        // will eventally be easier to change values from here than poke around through
        // files

    }

    public static final class AutoConstants {

        public static double kP_PathPlannerDrive = 7  ;
        public static double kD_PathPlannerDrive = 0.25;

        public static double kP_PathPlannerTurn = 6.25; 
        public static double kD_PathPlannerTurn = 0.1; 

        public static PIDConstants drivePIDConstants = new PIDConstants(
                Constants.AutoConstants.kP_PathPlannerDrive,
                0.0,
                Constants.AutoConstants.kD_PathPlannerDrive);
        public static PIDConstants turnPIDConstants = new PIDConstants(
                Constants.AutoConstants.kP_PathPlannerTurn,
                0,
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
