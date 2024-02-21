package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;

import java.util.Map;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;
// import frc.robot.commands.arm.ArmMoveToGoal;
import frc.robot.commands.swerve.Teleop;

/**
 * Util class for button bindings
 *
 * @author Rohin Sood
 */
public class JoystickIO {

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
        pilotBindings();

        //op bindings
    }

    /**
     * If only one controller is plugged in (pi)
     * 
     */

    public void pilotBindings() {

        // pilot.aWhileHeld(
        // Commands.run(() -> Robot.wrist.setVoltage(12)),
        // Commands.run(() ->
        // Robot.wrist.setVoltage(0))
        // );
        // pilot.bWhileHeld(
        // Commands.run(() -> Robot.wrist.setVoltage(-12)),
        // Commands.run(() -> Robot.wrist.
        // setVoltage(0))
        // );

        pilot.xWhileHeld(
                Commands.run(() -> Robot.intake.setIntakeVelocity(60)),
                Commands.run(() -> Robot.intake.setVoltage(0)));
        pilot.yWhileHeld(
                Commands.run(() -> Robot.intake.setIntakeVelocity(-60)),
                Commands.run(() -> Robot.intake.setVoltage(0)));
        pilot.aWhileHeld(
                Commands.run(() -> Robot.intake.setVoltage(12)),
                Commands.run(() -> Robot.intake.setVoltage( 0)));
        // pilot.rightTriggerWhileHeld(
        // Commands.run(() -> Robot.intake.setVoltage(-7)),
        // Commands.run(() -> Robot.intake.setVoltage(0))
        // );
        // pilot.leftTriggerWhileHeld(
        // Commands.run(() -> Robot.intake.setVoltage(7)),
        // Commands.run(() -> Robot.intake.setVoltage(0))
        // );
    }

    /**
     * If NO joysticks are plugged in (Buttons for commands are runnable in the
     * "Controls" tab in ShuffleBoard)
     */
    public void noJoystickBindings() {
        ShuffleboardTab controlsTab = Shuffleboard.getTab("Controls");

        // Example
        ShuffleboardLayout armCommands = controlsTab
                .getLayout("Arm", BuiltInLayouts.kList)
                .withSize(2, 2)
                .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands
        //sysid config
        pilot.aWhileHeld(Robot.swerve.getTurnSysIdQuasistaticForwardTest());
        pilot.bWhileHeld(Robot.swerve.getTurnSysIdQuasistaticReverseTest());
        pilot.yWhileHeld(Robot.swerve.getTurnSysIdDynamicForwardTest());
        pilot.xWhileHeld(Robot.swerve.getTurnSysIdDynamicReverseTest());

        //pilot commandse
    }

    public void simBindings() {
    }

    /**
     * Sets the default commands
     */
    public void setDefaultCommands() {

        Robot.swerve.setDefaultCommand(
                new Teleop(() -> -pilot.getLeftX(), () -> -pilot.getLeftY(), () -> -pilot.getRightX()));
    }
}