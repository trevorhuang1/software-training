package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class IntakeCommand extends Command{

    public IntakeCommand()
    {
        addRequirements(Robot.intake);
    }

    @Override
    public void execute()
    { 
        Robot.intake.moveIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
      }
}
