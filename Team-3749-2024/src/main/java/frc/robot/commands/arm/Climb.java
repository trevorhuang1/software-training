package frc.robot.commands.arm;

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
import frc.robot.utils.Constants;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.Constants.ArmConstants;

public class Climb extends Command {

    private double voltage = 0;

    public Climb() {
        addRequirements(Robot.arm);
    }

    @Override
    public void initialize() {

    }

    // private ShuffleData<Double> dKG = new
    // ShuffleData<Double>(Robot.arm.getName(), "DKG", 0.0);

    @Override
    public void execute() {
        // Can only climb if arm starts straight up
        if (Robot.arm.getGoal() != Math.PI/2){
            return;
        }
        // Stop/slow down at 2 degrees
        if (Units.radiansToDegrees(Robot.arm.getPositionRad()) > 2) {

            Robot.arm.setVoltage(-3);
        }
        else {
            Robot.arm.setVoltage(-1.5);
        }

    }

    @Override
    public void end(boolean interupted) {
        voltage = 0;
        Robot.arm.setVoltage(voltage);

    }

    @Override
    public boolean isFinished() {
        return false;

    }
}
