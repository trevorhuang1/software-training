package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.utils.Constants;

public class WristCommand extends Command{
    private boolean isGroundSetpoint = false;
    private HashMap<Boolean, Double> setpointToggle = new HashMap<Boolean,Double>();
    public WristCommand()
    {
        setpointToggle.put(true,Constants.WristConstants.groundSetpoint);
        setpointToggle.put(false,Constants.WristConstants.stowSetpoint);
        addRequirements(Robot.wristSpark);
    }

    @Override
    public void execute()
    {
        this.isGroundSetpoint = !isGroundSetpoint;
        Robot.wristSpark.setWristAngle(setpointToggle.get(this.isGroundSetpoint));
    }
    @Override
    public boolean isFinished() {
        return true;
      }
}
