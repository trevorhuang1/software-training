package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import frc.robot.utils.SuperStructureStates;

public class SuperStructureCommands {

    private GroundIntake groundIntake = new GroundIntake();
    private Stow stow = new Stow();
    private ScoreAmp scoreAmp= new ScoreAmp();

    private SuperStructureCommandInterface currentCommand = stow;

    public SuperStructureCommands() {
    }

    private void switchCommands(SuperStructureCommandInterface command) {
        // System.out.println("ground intake");
        if (currentCommand != command) {
            currentCommand.reset();
        }
        currentCommand = command;
    }

    public void execute() {
        SmartDashboard.putString("state", Robot.state.name());
        SmartDashboard.putString("current command", currentCommand.getClass().getName());
        SuperStructureStates state = Robot.state;
       
        switch (Robot.state) {
            case STOW:
                // System.out.println("stow");
                switchCommands(stow);
                break;
            case GROUND_INTAKE:
                // System.out.println("ground intake");
                switchCommands(groundIntake);
                break;
            case AMP:
                switchCommands(scoreAmp);
                break;
        }
        currentCommand.execute();
    }

}
