package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class WristCommand extends Command{

    public WristCommand()
    {
        addRequirements(Robot.wrist);
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
