package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utils.MiscConstants;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.UtilityFunctions;

public class GroundIntake extends Command {

    private boolean set4bar = false;
    private boolean setArmGround = false;

    public GroundIntake() {
        addRequirements(Robot.arm, Robot.wrist, Robot.intake);
    }

    @Override
    public void initialize() {
        set4bar = false;
        Robot.arm.setGoal(Units.degreesToRadians(14));

    }

    @Override
    public void execute() {
        double armPosRad = Robot.arm.getPositionRad();
        double wristPosRad = Robot.wrist.getPositionRad();

        // if within ~4 degrees of the goal but also bassically stopped in terms of
        // velocity  (the velocity is more important, the 4 degrees is to make sure its on the correct side)
        if (!set4bar

                && UtilityFunctions.withinMargin(0.5, armPosRad, Robot.arm.getGoal())
                && UtilityFunctions.withinMargin(0.05, 0, Robot.arm.getVelocityRadPerSec())) {
            set4bar = true;
            Robot.wrist.setGoalGround();
        }
        // if within ~8 degrees of the goal but also bassically stopped in terms of
        // velocity (the 4bar positions can get a little drifty)
        if (!setArmGround && set4bar
                && UtilityFunctions.withinMargin(0.1, wristPosRad, Robot.wrist.getWristGoal().position)
                && UtilityFunctions.withinMargin(0.05, 0, Robot.wrist.getVelocityRadPerSec())) {
            Robot.arm.setGoal(Units.degreesToRadians(8.5));
            setArmGround = true;
        }
        if (set4bar && setArmGround) {
            Robot.intake.setIntakeVelocity(60);

        }

    }

    @Override
    public void end(boolean interupted) {
        set4bar = false;
        setArmGround = false;
        // add this back in later, but with a delay so that the 4bar moves
        // Robot.arm.setGoal(0);
        Robot.wrist.setGoalStow();
        Robot.intake.setVoltage(0);

    }

    @Override
    public boolean isFinished() {
        return false;

    }
}
