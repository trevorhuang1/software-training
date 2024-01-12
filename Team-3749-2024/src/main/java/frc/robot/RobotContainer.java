// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;
import java.util.HashMap;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.swerve.PPUtils;
import frc.robot.commands.swerve.MoveToPose;
import frc.robot.commands.swerve.PPPaths;
import frc.robot.commands.swerve.TurnToAngle;
import frc.robot.utils.JoystickIO;
import frc.robot.utils.Xbox;
import frc.robot.utils.Constants.DriveConstants;

public class RobotContainer {

  private Xbox pilot = new Xbox(0);
  private Xbox operator = new Xbox(1);
  private final JoystickIO joystickIO = new JoystickIO(pilot, operator);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    DriverStation.removeRefreshedDataEventHandle(44000);

    HashMap<String, Command> commandList = new HashMap<String, Command>();

    commandList.put("PrintCMD-hello", new PrintCommand("hello"));

    PPPaths.init_pathCommands(commandList);
    PPUtils.initPPUtils();

    configureBindings();

    RobotController.setBrownoutVoltage(7.0);
    // Robot.swerve.resetOdometry(new Pose2d(1, 1, new Rotation2d(Math.PI * 11/6)));
  }

  private void configureBindings() {
    joystickIO.getButtonBindings();

  }

  public Command getAutonomousCommand() {
    Command command;
    // Command command = new MoveToPose(new Pose2d(5, 7, new Rotation2d(Math.PI /
    // 2)));
    // Command command = new TurnToAngle(new Rotation2d(Math.PI / 2));
    // Command command = PPUtils.followPathSequential(new String[] {
    // "CirclePath", "SquigglyPath" });
    // Command command = PPUtils.getAutoPath();
    // command = PPUtils.getPathFindToPreplannedCommand(
    //     "CirclePath",
    //     DriveConstants.pathFinderConstraints);

    command = PPUtils.getPathFindToPoseCommand(new Pose2d(null, null, null))

    return command;
  }
}
