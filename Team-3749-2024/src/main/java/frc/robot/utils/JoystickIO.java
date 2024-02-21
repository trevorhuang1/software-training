package frc.robot.utils;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmMoveToGoal;
import frc.robot.commands.arm.GetConstraints;
// import frc.robot.commands.swerve.MoveToPose;
// import frc.robot.commands.swerve.Teleop;
// import frc.robot.commands.swerve.TeleopJoystickRelative;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Util class for button bindings
 * 
 * @author Rohin Sood
 */
public class JoystickIO {
    private static String[] lastJoystickNames = new String[] { "", "", "", "", "", "" };

    private Xbox pilot;
    private Xbox operator;


    public JoystickIO(Xbox pilot, Xbox operator) {
        this.pilot = pilot;
        this.operator = operator;
    }

    /**
     * Calls binding methods according to the joysticks connected
     */
    public void getButtonBindings() {
        System.out.println(DriverStation.isJoystickConnected(0));

        if (DriverStation.isJoystickConnected(1)) {
            // if both xbox controllers are connected
            pilotAndOperatorBindings();

        } else if (DriverStation.isJoystickConnected(0)) {
            // if only one xbox controller is connected
            pilotBindings();

        } else if (Robot.isSimulation()) {
            // will show not connected if on sim
            simBindings();

        } else {
            // if no joysticks are connected (ShuffleBoard buttons)

        }
        setDefaultCommands();
    }

    /**
     * If both controllers are plugged in (pi and op)
     */
    public void pilotAndOperatorBindings() {

    }

    /**
     * If only one controller is plugged in (pi)
     */

    private ShuffleData<Double> armAngle = new ShuffleData<Double>("arm", "setpoint angle", 0.0);

    public void pilotBindings() {

        // pilot.a().whileTrue(Commands.run(()-> Robot.arm.setVoltage(kG.get())));
        // pilot.a().onFalse(Commands.runOnce(() -> Robot.arm.setVoltage(0)));
        // pilot.b().whileTrue(Commands.run(()-> Robot.arm.setVoltage(-4)));
        // pilot.b().onFalse(Commands.runOnce(() -> Robot.arm.setVoltage(0)));

        pilot.a().whileTrue(Commands.run(()->Robot.arm.setGoal(Units.degreesToRadians(armAngle.get()))));
        pilot.y().whileTrue(Commands.run(()->Robot.arm.setGoal(0)));

        // pilot.a().onFalse(Commands.runOnce(() -> Robot.arm.setVoltage(0)));
        pilot.b().whileTrue(new GetConstraints());

    }

    public void simBindings() {
        // pilot.aWhileHeld(new MoveToPose(new Pose2d(5, 5, new Rotation2d())));
    }

    /**
     * Sets the default commands
     */
    public void setDefaultCommands() {
        Robot.arm.setDefaultCommand(new ArmMoveToGoal());
    }

}
