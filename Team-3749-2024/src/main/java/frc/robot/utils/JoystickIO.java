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
import frc.robot.commands.arm.Climb;
import frc.robot.commands.arm.GetConstraints;
import frc.robot.commands.arm.MoveArmToGoal;
import frc.robot.commands.superstructure.GroundIntake;
// import frc.robot.commands.arm.ArmMoveToGoal;
import frc.robot.commands.swerve.SwerveTeleop;
import frc.robot.commands.wrist.MoveWristToGoal;
import frc.robot.commands.wrist.getRegressionData;
import frc.robot.subsystems.arm.ArmSim;
import frc.robot.subsystems.arm.ShootKinematics;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
// import frc.robot.commands.swerve.MoveToPose;
// import frc.robot.commands.swerve.Teleop;
// import frc.robot.commands.swerve.TeleopJoystickRelative;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

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

        Robot.operator.b()
                .whileTrue(Commands.run(() -> Robot.intake.setVoltage(12)))
                .onFalse(Commands.runOnce(() -> Robot.intake.stop()));

        Robot.operator.rightTrigger().whileTrue(Commands
                .run(() -> Robot.shooter.setShooterVelocity(ShooterConstants.shooterVelocityRadPerSec), Robot.shooter))
                .onFalse(Commands.runOnce(() -> {
                    Robot.shooter.stop();
                    Robot.intake.stop();
                }, Robot.shooter));

        Robot.operator.leftBumper()
                .onTrue(Commands.run(() -> Robot.intake.setVoltage(-1)));

        Robot.operator.x().onTrue(Commands.runOnce(() -> Robot.state = SuperStructureStates.AMP, Robot.wrist,
                Robot.arm, Robot.intake, Robot.shooter))
                .onFalse(
                        Commands.runOnce(() -> Robot.state = SuperStructureStates.STOW, Robot.wrist, Robot.arm,
                                Robot.intake));

        Robot.pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));
        Robot.pilot.leftTrigger()
                .onTrue(Commands.runOnce(() -> Robot.state = SuperStructureStates.GROUND_INTAKE, Robot.wrist, Robot.arm,
                        Robot.intake))
                .onFalse(Commands.runOnce(() -> Robot.state = SuperStructureStates.STOW, Robot.wrist, Robot.arm));

        // op bindings
    }

    public void pilotBindings() {
        /// ground intake (some is redundant)
        Robot.pilot.leftTrigger().onTrue(Commands.runOnce(() -> Robot.state = SuperStructureStates.GROUND_INTAKE))
                .onFalse(Commands.runOnce(() -> {
                    Robot.state = SuperStructureStates.STOW;
                }, Robot.wrist, Robot.intake));
        // .whileTrue(Commands.run(() ->
        // Robot.intake.setIntakeVelocity(IntakeConstants.intakeVelocityRadPerSec),
        // Robot.intake));
        // score subwoofer
        Robot.pilot.a().onTrue(Commands.runOnce(() -> Robot.state = SuperStructureStates.SUBWOOFER))
                .onFalse(Commands.runOnce(() -> {
                    Robot.state = SuperStructureStates.STOW;
                }, Robot.wrist));

        Robot.pilot.rightTrigger().whileTrue(
                Commands.run(() -> Robot.shooter.setShooterVelocity(ShooterConstants.shooterVelocityRadPerSec)))
                .onFalse(Commands.runOnce(() -> Robot.shooter.stop(), Robot.shooter));

        Robot.pilot.b().whileTrue(Commands.run(() -> Robot.intake.setVoltage(12)))
                .onFalse(Commands.runOnce(() -> Robot.intake.stop(), Robot.intake));
        // gyro
        Robot.pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));

        Robot.pilot.leftBumper().whileTrue(Commands.run(() -> {
            Robot.intake.setVoltage(2);
            Robot.shooter.setVoltage(-0.2, -0.2);
        })).onFalse(Commands.runOnce(() -> {
            Robot.intake.setVoltage(0);
            Robot.shooter.setVoltage(0, 0);

        }));
                Robot.pilot.rightBumper().whileTrue(Commands.run(() -> {
            Robot.intake.setVoltage(-2);
            Robot.shooter.setVoltage(-2, -2);
        })).onFalse(Commands.runOnce(() -> {
            Robot.intake.setVoltage(0);
            Robot.shooter.setVoltage(0, 0);

        }));
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
