package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.utils.Constants;

public class WristCommand extends Command{
    private double currentSetpoint = Constants.WristConstants.stowSetpoint;
    private boolean isGroundSetpoint = false;

    public WristCommand()
    {
        this.isGroundSetpoint = !isGroundSetpoint;
        addRequirements(Robot.wristSpark);
    }

    @Override 
    public void execute()
    {

    }  
    //wristMotor.setVoltage(wristFF.calculate(currentSetpoint) + wristController.calculate(wristEncoder.getPosition()+wristOffset,currentSetpoint));
}
