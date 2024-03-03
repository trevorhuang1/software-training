package frc.robot.subsystems.wrist;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;

public final class WristConstants {

    public static final int wristId = 17;
    public static final double gearRatio = 1;

    public static final double groundGoalRad = Units.degreesToRadians(141);
    public static final double stowGoalRad = Units.degreesToRadians(3);

    public static final double wristOffsetRad = Units.degreesToRadians(257.4);

    private static final PIDConstants simPID = new PIDConstants(35, 0, 1);

    private static final PIDConstants realPID = new PIDConstants(0.4, 0, 0); // 0.35

    public static final PIDConstants PID = Robot.isReal() ? realPID : simPID;

    private static final Constraints simConstraint = new Constraints(
            Math.PI,
            2 * Math.PI); // we stealing from arm with
    // this one
    private static final Constraints realConstraint = new Constraints(
            2.5*Math.PI,
            2.5*Math.PI);

    public static final Constraints trapezoidConstraint = Robot.isReal()
            ? realConstraint
            : simConstraint;

    // thanks arm (robbery)
    public static final double simkS = 0.0;
    public static final double simkG = .33775;
    public static final double simkV = 6.616;
    public static final double simkA = 0;

    public static final double realkS = 0.1;
    public static final double realkVForward = 1.575;// 1.6 // radians
    public static final double realkVBackward = 1.35; // radians

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

}
