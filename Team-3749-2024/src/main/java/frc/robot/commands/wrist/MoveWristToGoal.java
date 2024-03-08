package frc.robot.commands.wrist;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.wrist.WristConstants.WristStates;

public class MoveWristToGoal extends Command {

    public MoveWristToGoal() {
        addRequirements(Robot.wrist);
    }

    @Override
    public void initialize() {

    }

    // private ShuffleData<Double> dKG = new
    // ShuffleData<Double>(Robot.arm.getName(), "DKG", 0.0);

    @Override
    public void execute() {
        // the below may no longer be true

        // if the wrist is not at stow and the arm is not at stow and the arm is moving,
        // then set a solid 1
        // volt to maintain its locked position in full deploy. Will cause problems if
        // in-transit to stow while the arm moves somewhere else, so consider having two
        // different in-transits
        if ((Robot.wrist.getState() == WristStates.FULL_DEPLOYED)
                && Robot.arm.getState() != ArmStates.STOW
                && (Math.abs(Robot.arm.getVelocityRadPerSec()) > 0.05)) {

            Robot.wrist.setVoltage(1);

        } else {
            Robot.wrist.moveWristToGoal();

        }

    }

    @Override
    public void end(boolean interupted) {

    }

    @Override
    public boolean isFinished() {
        return false;

    }
}
