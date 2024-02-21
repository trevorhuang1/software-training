// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import java.util.HashMap;

import frc.robot.commands.swerve.AutoUtils;
import frc.robot.subsystems.arm.ShootKinematics;
import java.nio.file.Path;
import java.sql.Driver;
import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.PrintCommand;
// import frc.robot.commands.swerve.MoveToPose;
import frc.robot.commands.swerve.Teleop;
// import frc.robot.commands.swerve.TurnToAngle;
import frc.robot.utils.Constants;
import frc.robot.utils.JoystickIO;
import frc.robot.utils.Xbox;
import frc.robot.utils.Constants.DriveConstants;

public class RobotContainer {
  public static Xbox pilot = new Xbox(0);
  public static Xbox operator = new Xbox(1);
  private final JoystickIO joystickIO = new JoystickIO(pilot, operator);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    DriverStation.removeRefreshedDataEventHandle(44000);

    configureBindings();
  
    // DataLogManager.start(".wpilog");
    // DataLogManager.logNetworkTables(true);
    // DriverStation.startDataLog(DataLogManager.getLog(), true);

    initAuto();

    RobotController.setBrownoutVoltage(7.0);

    Robot.swerve.resetOdometry(DriveConstants.fieldStartingPose);
    // Robot.swerve.setDefaultCommand(new Teleop(pilot::getLeftX, pilot::getLeftY, pilot::getRightX, pilot::getRightY));
  }


  private void configureBindings() {
    joystickIO.getButtonBindings();
    joystickIO.pilotBindings();
  }

  public void initAuto() {
    HashMap<String, Command> commandList = new HashMap<String, Command>();

    commandList.put("PrintCMD-hello", Commands.print("hewlow"));
    commandList.put("shoot", Commands.print("shot a thing"));
    commandList.put("shoot-amp", Commands.print("shot a thing"));

    AutoUtils.initAuto(commandList);
  }

  public Command getAutonomousCommand() {

    // return
    // AutoUtils.timeCommand(AutoUtils.getAutoPath("bottom-s_3w-speaker+c-speaker"));
    return Robot.swerve.getSysIdDynamic(Direction.kForward);
  }
}
