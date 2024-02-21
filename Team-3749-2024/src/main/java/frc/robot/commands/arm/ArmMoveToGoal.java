package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.utils.Constants;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.Constants.ArmConstants;

public class ArmMoveToGoal extends Command {

    private double prevSetpointVelocity = 0;
    BooleanSupplier isFourBarDeployedSupplier;

    public ArmMoveToGoal(BooleanSupplier isFourBarDeployedSupplier) {
        addRequirements(Robot.arm);
        this.isFourBarDeployedSupplier = isFourBarDeployedSupplier;
    }

    @Override
    public void initialize() {
    }

    // private ShuffleData<Double> dKG = new
    // ShuffleData<Double>(Robot.arm.getName(), "DKG", 0.0);

    @Override
    public void execute() {
        State setpoint = Robot.arm.getSetpoint();
        double accelerationSetpoint = (setpoint.velocity - prevSetpointVelocity) / 0.02;
        prevSetpointVelocity = setpoint.velocity;

        // if resting on the hard stop, don't waste voltage on kG
        // if (setpoint.position == 0 && Robot.arm.getRotation2d().getDegrees() < 2) {
        // Robot.arm.setVoltage(0);
        // return;
        // }
        if (setpoint.velocity == 0 && isFourBarDeployedSupplier.getAsBoolean()) {
            // kg and P
            double voltage = ArmConstants.deployedKG * Math.cos(Robot.arm.getRotation2d().getRadians())
                    + ArmConstants.deployedKP * (setpoint.position - Robot.arm.getRotation2d().getRadians());
            Robot.arm.setVoltage(voltage);
            return;
        }

        Robot.arm.setState(setpoint.position, setpoint.velocity, accelerationSetpoint);

    }

    @Override
    public void end(boolean interupted) {

    }

    @Override
    public boolean isFinished() {
        return false;

    }
}
