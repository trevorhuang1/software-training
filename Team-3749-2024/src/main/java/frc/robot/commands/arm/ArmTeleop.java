package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ArmTeleop extends Command {

    public ArmTeleop(){

    }
    
    @Override
    public void initialize(){
        addRequirements(Robot.arm);
    }
    @Override 
    public void execute(){

    }
    @Override
    public void end(){
        
    }

    @Override 
    public boolean isFinished(){

    }
}
