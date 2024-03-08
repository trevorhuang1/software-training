// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.HashMap;

import frc.robot.commands.swerve.AutoUtils;
import frc.robot.commands.swerve.Autos;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.utils.JoystickIO;
import frc.robot.utils.SuperStructureStates;

public class RobotContainer {

  private final JoystickIO joystickIO = new JoystickIO();

  public RobotContainer() {
    if (Robot.isSimulation()) {
      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      inst.stopServer();
      // Change the IP address in the below function to the IP address you use to
      // connect to the PhotonVision UI.
      inst.setServer("127.0.0.1");
      inst.startClient4("Robot Simulation");
    }
    DriverStation.silenceJoystickConnectionWarning(true);
    DriverStation.removeRefreshedDataEventHandle(44000);

    configureBindings();

    // DataLogManager.start(".wpilog");
    // DataLogManager.logNetworkTables(true);
    // DriverStation.startDataLog(DataLogManager.getLog(), true);

    initAuto();

    RobotController.setBrownoutVoltage(7.0);

    // Robot.swerve.resetOdometry(DriveConstants.fieldStartingPose);
    // Robot.swerve.setDefaultCommand(new Teleop(pilot::getLeftX, pilot::getLeftY,
    // pilot::getRightX, pilot::getRightY));
  }

  private void configureBindings() {
    // joystickIO.pilotBindings();
    joystickIO.pilotAndOperatorBindings();
    joystickIO.setDefaultCommands();
  }

  public void initAuto() {
    HashMap<String, Command> commandList = new HashMap<String, Command>();

    commandList.put("cycle",
       new SequentialCommandGroup(Commands.runOnce(() -> Robot.state = SuperStructureStates.SUBWOOFER),
            new WaitCommand(3), Commands.runOnce(() -> Robot.intake.setState(IntakeStates.FEED)),
            new WaitCommand(0.25), Commands.runOnce(() -> Robot.state = SuperStructureStates.GROUND_INTAKE)));

    AutoUtils.initAuto(commandList);
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(1);
    // return new PrintCommand("no auto");
    // return Commands.run(() -> {
    //   Robot.intake.setIntakeVelocity(100);
    //   Robot.shooter.setShooterVelocity(150);
    // });
    // return Autos.get4Piece();
    // return Robot.swerve.getSysIdDynamic(Direction.kForward);
  }
}
