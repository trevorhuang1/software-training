package frc.robot.utils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmMoveToGoal;
import frc.robot.commands.arm.Climb;
import frc.robot.commands.arm.GetConstraints;
// import frc.robot.commands.arm.ArmMoveToGoal;
import frc.robot.commands.swerve.Teleop;
import frc.robot.commands.wrist.WristCommand;
import frc.robot.commands.wrist.getRegressionData;
import frc.robot.subsystems.arm.ShootKinematics;
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

  private Xbox pilot;
  private Xbox operator;

  public JoystickIO() {
    pilot = Robot.pilot;
    operator = Robot.operator;
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
    // pilot.leftTrigger().whileTrue(Commands.run(() ->
    // Robot.intake.setIntakeVelocity(60),
    // Robot.intake));
    // pilot.leftTrigger().onFalse(Commands.runOnce(() ->
    // Robot.intake.setVoltage(0),
    // Robot.intake));
    // pilot.rightTrigger().whileTrue(Commands.run(() ->
    // Robot.intake.setIntakeVelocity(-60),
    // Robot.intake));
    // pilot.rightTrigger().onFalse(Commands.runOnce(() ->
    // Robot.intake.setVoltage(0),
    // Robot.intake));

    // pilot.a().onTrue(Commands.runOnce(() ->
    // Robot.arm.setGoal(Units.degreesToRadians(6.5))));
    // pilot.y().onTrue(Commands.runOnce(() ->
    // Robot.arm.setGoal(Units.degreesToRadians(90))));

    // pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));
    // pilot.back().whileTrue(new Climb());

    // pilot.y()
    //         .whileTrue(
    //             Commands.run(() ->
    //                 Robot.wrist.runFF(2)
    //             )).whileFalse(Commands.run(() ->
    //                 Robot.wrist.runFF(0)));

    // pilot.y().onFalse(Commands.runOnce(() -> Robot.wrist.setVoltage(0), Robot.wrist));

    // pilot.a().whileTrue(
    //     new getRegressionData(true));
    // pilot.a().onFalse(Commands.runOnce(() -> Robot.wrist.setVoltage(0), Robot.wrist));

    // pilot.b().whileTrue(
    //     new getRegressionData(false));
    // pilot.b().onFalse(Commands.runOnce(() -> Robot.wrist.setVoltage(0), Robot.wrist));

    // pilot.rightBumper().onTrue(Commands.runOnce(() -> Robot.wrist.setGoalGround()));
    // pilot.leftBumper().onTrue(Commands.runOnce(() -> Robot.wrist.setGoalStow()));

    pilot.aWhileHeld(Robot.swerve.getTurnSysIdDynamicForwardTest());
    pilot.bWhileHeld(Robot.swerve.getTurnSysIdDynamicReverseTest());
    pilot.yWhileHeld(Robot.swerve.getTurnSysIdQuasistaticForwardTest());
    pilot.xWhileHeld(Robot.swerve.getTurnSysIdQuasistaticReverseTest());
  }

  public void simBindings() {
    // pilot.aWhileHeld(new MoveToPose(new Pose2d(5, 5, new Rotation2d())));
  }

  /**
   * Sets the default commands
   */
  public void setDefaultCommands() {
    // Robot.arm.setDefaultCommand(new ArmMoveToGoal(() -> pilot.b().getAsBoolean()));

    // Robot.arm.setDefaultCommand(new ArmMoveToGoal(() ->
    // Robot.wrist.getIsGroundIntake()));
    Robot.wrist.setDefaultCommand(new WristCommand());

    Robot.swerve.setDefaultCommand(
      new Teleop(
        () -> -pilot.getLeftX(),
        () -> -pilot.getLeftY(),
        () -> -pilot.getRightX()
      )
    );
  }
}
