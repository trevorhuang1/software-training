package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ShintakeCommand extends Command{

    public ShintakeCommand()
    {
        addRequirements(Robot.shintake);
    }

    @Override
    public void execute()
    { 
        Robot.shintake.moveShintake();
    }

    @Override
    public boolean isFinished() {
        return false;
      }
}
