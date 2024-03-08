package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.utils.MiscConstants;

public class GetConstraints extends Command {

    private double prevSetpointVelocity = 0;

    public GetConstraints() {
        addRequirements(Robot.arm); 

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double pos = Robot.arm.getPositionRad();
        if (pos>=Units.degreesToRadians(80)){
            Robot.arm.setVoltage(0);
            return;
        }
        Robot.arm.setVoltage(12);

    }

    @Override
    public void end(boolean interupted) {

    }

    @Override
    public boolean isFinished() {
        return false;

    }
}
