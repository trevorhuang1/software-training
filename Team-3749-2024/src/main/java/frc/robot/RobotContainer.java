// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;
import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
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
import frc.robot.commands.swerve.TurnToAngle;
import frc.robot.utils.Constants;
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

    initAuto();
    configureBindings();

    RobotController.setBrownoutVoltage(7.0);
    Robot.swerve.resetOdometry(DriveConstants.fieldStartingPose);
    Robot.swerve.logDesiredOdometry(DriveConstants.fieldStartingPose);
  }

  private void configureBindings() {
    joystickIO.getButtonBindings();

  }

  public void initAuto() {
    HashMap<String, Command> commandList = new HashMap<String, Command>();

    commandList.put("PrintCMD-hello", new PrintCommand("hello"));

    PPUtils.initPathCommands(commandList);
    PPUtils.initPPUtils();
  }

  public Command getAutonomousCommand() {
    Command command = new PrintCommand("Ran Autonomous");

    PathPlannerPath path = PathPlannerPath.fromPathFile("Straight line");
    Robot.swerve.resetOdometry(path.getPreviewStartingHolonomicPose());
    return AutoBuilder.followPath(path);
  }

}
