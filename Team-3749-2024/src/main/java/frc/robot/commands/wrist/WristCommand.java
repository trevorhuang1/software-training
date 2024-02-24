package frc.robot.commands.wrist;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class WristCommand extends Command{

    private double previousVelocity = 0;

    public WristCommand()
    {
        addRequirements(Robot.wrist);
    }

    @Override
    public void execute()
    { 
        State setpoint = Robot.wrist.getWristSetpoint();
        double acceleration = (setpoint.velocity - previousVelocity) / 0.02;
        previousVelocity = setpoint.velocity;
        System.out.println(setpoint.position);
        Robot.wrist.moveWristToAngle(setpoint.position,setpoint.velocity,acceleration);
    }

    @Override
    public boolean isFinished() {
        return false;
      }
}
