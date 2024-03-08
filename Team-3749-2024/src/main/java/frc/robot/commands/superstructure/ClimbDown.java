package frc.robot.commands.superstructure;

import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;

public class ClimbDown implements SuperStructureCommandInterface {

    public ClimbDown() {

    }

    @Override
    public void start() {
        
        Robot.arm.setVoltage(-4.5);

    }

    @Override
    public void execute() {
                Robot.arm.setVoltage(-4.5);

    }
}