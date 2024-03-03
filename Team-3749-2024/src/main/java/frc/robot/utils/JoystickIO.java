package frc.robot.utils;

import org.photonvision.estimation.RotTrlTransform3d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.swerve.TeleopJoystickRelative;
import frc.robot.commands.GroundIntake;
import frc.robot.commands.arm.Climb;
import frc.robot.commands.arm.GetConstraints;
// import frc.robot.commands.arm.ArmMoveToGoal;
import frc.robot.commands.swerve.SwerveTeleop;
import frc.robot.commands.wrist.getRegressionData;
import frc.robot.subsystems.arm.ShootKinematics;
// import frc.robot.commands.swerve.MoveToPose;
// import frc.robot.commands.swerve.Teleop;
// import frc.robot.commands.swerve.TeleopJoystickRelative;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.Robot;

/**
 * Util class for button bindings
 *
 * @author Rohin Sood
 */
public class JoystickIO {

    public JoystickIO() {

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
        // op bindings
    }

    public void pilotBindings() {

        // intake
        Robot.pilot.leftTrigger().whileTrue(Commands.run(() -> Robot.intake.setIntakeVelocity(60),
                Robot.intake));
        Robot.pilot.leftTrigger().onFalse(Commands.runOnce(() -> Robot.intake.setVoltage(0),
                Robot.intake));

        Robot.pilot.leftBumper().whileTrue(new GroundIntake());

        Robot.pilot.x().onTrue(Commands.runOnce(() -> Robot.arm.setGoal(Units.degreesToRadians(0))));
        // Robot.pilot.b().onTrue(Commands.runOnce(() ->
        // Robot.arm.setGoal(Units.degreesToRadians(6))));

        Robot.pilot.y().onTrue(Commands.runOnce(() -> Robot.arm.setGoal(Units.degreesToRadians(40))));
        Robot.pilot.back().whileTrue(new Climb());

        // gyro
        Robot.pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));

        // // 4bar

        Robot.pilot.rightBumper().onTrue(Commands.runOnce(() -> Robot.wrist.setGoalGround()));
        Robot.pilot.leftBumper().onTrue(Commands.runOnce(() -> Robot.wrist.setGoalStow()));
        // operator.a().whileTrue(Commands.run(() -> Robot.swerve.setChassisSpeeds(
        // ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(1, 0, 0),
        // Robot.swerve.getRotation2d())),
        // Robot.swerve));

        Robot.pilot.povDown().onTrue(Commands.runOnce(
                () -> Robot.swerve.resetOdometry(new Pose2d(new Translation2d(13.95,5.55), Robot.swerve.getRotation2d()))));
    }

    public void simBindings() {
        // Robot.pilot.aWhileHeld(new MoveToPose(new Pose2d(5, 5, new Rotation2d())));
    }

    /**
     * Sets the default commands
     */
    public void setDefaultCommands() {

        Robot.swerve.setDefaultCommand(
                new SwerveTeleop(
                        () -> -Robot.pilot.getLeftX(),
                        () -> -Robot.pilot.getLeftY(),
                        () -> -Robot.pilot.getRightX()));
    }

}
