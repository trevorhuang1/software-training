package frc.robot.subsystems.arm;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;

public final class ArmConstants {

    public static final int leftID = 15;
    public static final int rightID = 16;
    // inverse gear ratio * 1min/60sec * 2PI to get rad/sec
    public static final double gearRatio = 200.0;

    public static final double sprocketRatio = 64.0 / 24.0;
    public static final int encoderID = 7;
    public static final double encoderOffsetRad = Units.degreesToRadians(86.15);

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
    public static final double shooterVelocity = 20.0; // NOTE: likely will vary, might need to pass as
                                                       // parameter

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
