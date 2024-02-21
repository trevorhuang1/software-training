package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ShooterCommand extends Command{

    public ShooterCommand()
    {
        addRequirements(Robot.shooter);
    }

    @Override
    public void execute()
    { 
        Robot.shooter.moveShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
