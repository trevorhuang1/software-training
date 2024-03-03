package frc.robot.commands.wrist;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.wrist.WristConstants.WristStates;

public class MoveWristToGoal extends Command  {

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
        // keep the wrist down when moving the arm off of stow
        if ((Robot.wrist.getState() == WristStates.FULL_DEPLOYED || Robot.wrist.getState() == WristStates.GROUND_INTAKE) && Robot.arm.getGoal()!= ArmConstants.stowPositionRad){
            System.out.println("" + Robot.arm.getGoal() + " : " + ArmConstants.stowPositionRad);
            Robot.wrist.setVoltage(0.5);
        }
        else{
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
