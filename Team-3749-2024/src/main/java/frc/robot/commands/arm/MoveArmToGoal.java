package frc.robot.commands.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.wrist.WristConstants.WristStates;

public class MoveArmToGoal extends Command  {

    public MoveArmToGoal() {
        addRequirements(Robot.arm);
    }

    @Override
    public void initialize() {

    }

    // private ShuffleData<Double> dKG = new
    // ShuffleData<Double>(Robot.arm.getName(), "DKG", 0.0);

    @Override
    public void execute() {
        Robot.arm.moveToGoal();

    }

    @Override
    public void end(boolean interupted) {

    }

    @Override
    public boolean isFinished() {
        return false;

    }
}
