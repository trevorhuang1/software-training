package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.utils.Constants;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.Constants.ArmConstants;

public class ArmMoveToGoal extends Command {

    private double prevSetpointVelocity = 0;

    public ArmMoveToGoal() {
        addRequirements(Robot.arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        State setpoint = Robot.arm.getSetpoint();
        double accelerationSetpoint = (setpoint.velocity - prevSetpointVelocity) / 0.02;
        prevSetpointVelocity = setpoint.velocity;
        double feedback = Robot.arm.calculatePID(Robot.arm.getRotation2d().getRadians());

        // if resting on the hard stop, don't waste voltage on kG
        if (setpoint.position == 0 && Robot.arm.getRotation2d().getDegrees() < 2) {
            Robot.arm.setVoltage(0);

            return;
        }
        // if 4bar is deployed, switch kG
        if (setpoint.velocity == 0 && Robot.wrist.getIsDeployed()) {
            // ks, kg, and P
            double error = (setpoint.position - Robot.arm.getRotation2d().getRadians());
            double voltage = Math.signum(error) * ArmConstants.kS
                    + ArmConstants.deployedKG * Math.cos(Robot.arm.getRotation2d().getRadians())
                    + ArmConstants.deployedKP * error;
            Robot.arm.setVoltage(voltage);
            return;
        }

        double feedforward;
        if (setpoint.velocity != 0) {
            feedforward = Robot.arm.calculateFF(Robot.arm.getRotation2d().getRadians(),setpoint.velocity, accelerationSetpoint);
        } else {
            // have the kS help the PID when stationary
            feedforward = Math.signum(feedback) * ArmConstants.kS;
        }
        Robot.arm.setVoltage(feedforward + feedback);
    }

    @Override
    public void end(boolean interupted) {

    }

    @Override
    public boolean isFinished() {
        return false;

    }
}
