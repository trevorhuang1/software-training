// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.swerve.PathPlannerUtils;
import frc.robot.commands.swerve.MoveToPose;
import frc.robot.commands.swerve.TurnToAngle;
import frc.robot.utils.JoystickIO;
import frc.robot.utils.Xbox;

public class RobotContainer {

  private Xbox pilot = new Xbox(0);
  private Xbox operator = new Xbox(1);
  private final JoystickIO joystickIO = new JoystickIO(pilot, operator);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    DriverStation.removeRefreshedDataEventHandle(44000);

    NamedCommands.registerCommand("PrintCMD-hello", new PrintCommand("hello"));
    PathPlannerUtils.init_PathPlannerUtils();

    configureBindings();

    RobotController.setBrownoutVoltage(7.0);
    // Robot.swerve.resetOdometry(new Pose2d(1, 1, new Rotation2d(Math.PI * 11/6)));
  }

  private void configureBindings() {
    joystickIO.getButtonBindings();

  }

  public Command getAutonomousCommand() {

    // Command command = new MoveToPose(new Pose2d(5, 7, new Rotation2d(Math.PI /
    // 2)));
    // Command command = new TurnToAngle(new Rotation2d(Math.PI / 2));
    // Command command = PathPlannerUtils.followPathSequential(new String[] { "CirclePath", "SquigglyPath" });
    // Command command = PathPlannerUtils.getAutoPath();
    Command command = PathPlannerUtils.getPathFindCommand(
      new Pose2d(15, 6, new Rotation2d(0)),
      3.0,
      4.0,
      540,
      720
      );

    return command;
  }
}
