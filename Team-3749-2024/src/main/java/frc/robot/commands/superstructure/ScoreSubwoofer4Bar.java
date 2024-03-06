package frc.robot.commands.superstructure;

import frc.robot.Robot;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.wrist.WristConstants.WristStates;

public class ScoreSubwoofer4Bar implements SuperStructureCommandInterface {
    

    public ScoreSubwoofer4Bar(){}

    @Override
    public void execute(){
        Robot.wrist.setGoal(WristStates.SUBWOOFER);
        Robot.wrist.moveWristToGoal();
    }

    public void reset(){

    }
}
