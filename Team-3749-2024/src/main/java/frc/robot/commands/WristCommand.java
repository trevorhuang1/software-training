package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.utils.Constants;

public class WristCommand extends Command{
    private HashMap<Boolean, Double> setpointToggle = new HashMap<Boolean,Double>();
    private boolean isGroundIntake = false;
    public WristCommand()
    {
        setpointToggle.put(true,Constants.WristConstants.groundGoal);
        setpointToggle.put(false,Constants.WristConstants.stowGoal);
        addRequirements(Robot.wrist);
    }


    @Override
    public void initialize()
    { 
        
    //    this.isGroundIntake = !this.isGroundIntake;
    //    Robot.wrist.setWristGoal(setpointToggle.get(isGroundIntake));
    }

    @Override
    public void execute()
    { 
        Robot.wrist.moveWristToAngle();

    }

    @Override
    public boolean isFinished() {
        return true;
      }
}
