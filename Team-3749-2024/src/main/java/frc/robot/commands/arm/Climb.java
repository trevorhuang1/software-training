package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.utils.Constants;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.Constants.ArmConstants;

public class Climb extends Command {

    private double goal = 0;
    private State currentSetpoint = new State();
    private ProfiledPIDController profiledFeedbackController = new ProfiledPIDController(ArmConstants.PID.kP,
            ArmConstants.PID.kI,
            ArmConstants.PID.kD,
            new Constraints(0.2, 0.2));

    private ArmFeedforward feedForwardController = new ArmFeedforward(ArmConstants.kS,
            ArmConstants.kG,
            ArmConstants.kV);

    private double prevSetpointVelocity = 0;

    public Climb() {
        addRequirements(Robot.arm);
    }

    @Override
    public void initialize() {
        profiledFeedbackController.setGoal(0);

    }

    // private ShuffleData<Double> dKG = new
    // ShuffleData<Double>(Robot.arm.getName(), "DKG", 0.0);

    @Override
    public void execute() {

        currentSetpoint = profiledFeedbackController.getSetpoint();
        double accelerationSetpoint = (currentSetpoint.velocity - prevSetpointVelocity) / 0.02;
        prevSetpointVelocity = currentSetpoint.velocity;

        // update for logging
        double feedback;
        double feedforward;
        feedback = profiledFeedbackController.calculate(Robot.arm.getRotation2d().getRadians());
        feedforward = feedForwardController.calculate(Robot.arm.getRotation2d().getRadians(), currentSetpoint.velocity,
                accelerationSetpoint);

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
