// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import java.util.HashMap;

import frc.robot.commands.swerve.AutoUtils;
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
    initAuto();

    RobotController.setBrownoutVoltage(7.0);

    Robot.swerve.resetOdometry(DriveConstants.fieldStartingPose);
  }

  private void configureBindings() {
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
    
    return AutoUtils.timeCommand(AutoUtils.getAutoPath());
  }
}
