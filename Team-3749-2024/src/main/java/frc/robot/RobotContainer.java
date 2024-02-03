// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.FileNotFoundException;
import java.io.IOException;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.nio.file.Path;
import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.swerve.AutoUtils;
import frc.robot.commands.swerve.MoveToPose;
import frc.robot.commands.swerve.Teleop;
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

    configureBindings();
    AutoUtils.initPPUtils();

    RobotController.setBrownoutVoltage(7.0);

    Robot.swerve.resetOdometry(DriveConstants.fieldStartingPose);
  }

  private void configureBindings() {
    joystickIO.getButtonBindings();

  }

  public Command getAutonomousCommand() {
    // return Commands.run(() -> Robot.arm.setVoltage(8-0.973));
    return AutoUtils.getAutoPath("Choreo-BottomSpeaker-3xWing_Speaker-Center-Speaker");
    // return Commands.run(() -> Robot.arm.setGoal(Units.degreesToRadians(90)));
  }


}
