package frc.robot.commands.swerve;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Constants;

public class PPUtils {
  private static Swerve swerve = Robot.swerve;
  public static Consumer<Pose2d> pathTargetPose = pose -> swerve.logDesiredOdometry(pose);

  static SendableChooser<Command> autoChooser;
  static SendableChooser<Alliance> allianceChooser;

  static boolean isFirstPath = true;

  public static void initPPUtils() {
    PathPlannerLogging.setLogTargetPoseCallback(pathTargetPose);

    allianceChooser = new SendableChooser<>();
    allianceChooser.addOption("Blue Alliance", Alliance.Blue);
    allianceChooser.addOption("Red Alliance", Alliance.Red);
    allianceChooser.setDefaultOption("Red Alliance", Alliance.Red);

    AutoBuilder.configureHolonomic(
        swerve::getPose,
        swerve::resetOdometry,
        swerve::getChassisSpeeds,
        swerve::setChassisSpeeds,
        Constants.PathPlannerConstants.cfgHolonomicFollower,
        () -> {
          Alliance robotAlliance = allianceChooser.getSelected();
          robotAlliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : robotAlliance;
          System.out.println();
          if (robotAlliance == Alliance.Red) {
            return true;
          } else {
            return false;
          }
        },
        swerve);

    autoChooser = AutoBuilder.buildAutoChooser("TestAuto");

    SmartDashboard.putData("Choose Auto", autoChooser);
    SmartDashboard.putData("Choose Alliance", allianceChooser);
  }

  public static void initPathCommands(HashMap<String, Command> commandList) {
    commandList.forEach((String cmdName, Command cmd) -> {
      NamedCommands.registerCommand(cmdName, cmd);
    });
  }

  public static void checkIsFirstPath(PathPlannerPath path) {
    if (isFirstPath) {
      swerve.resetOdometry(path.getPreviewStartingHolonomicPose());
      isFirstPath = !isFirstPath;
    }
  };

  public static Command getAutoPath() {
    return autoChooser.getSelected();
  }

  public static Command getAutoPath(String autoPathName) {

    return AutoBuilder.buildAuto(autoPathName);
  }

  public static Command getPathFindToPoseCommand(Pose2d pose, PathConstraints constraints) {
    return AutoBuilder.pathfindToPose(pose, constraints);
  }

  public static Command getPathFindToPoseCommand(Pose2d targetPose, double endingVelocity) {

    return AutoBuilder.pathfindToPose(targetPose, Constants.PathPlannerConstants.defaultPathConstraints,
        endingVelocity);
  }

  public static Command getPathFindToPoseCommand(Pose2d targetPose, PathConstraints constraints,
      double endingVelocity) {

    return AutoBuilder.pathfindToPose(targetPose, constraints, endingVelocity);
  }

  public static Command getPathFindThenFollowPathCommand(String pathName, PathConstraints constraints) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
        path,
        constraints, 0);
    return pathfindingCommand;
  }


  //?????????????????????????????????????????????????????????????????????????????????????????????????????
  //  What is the point of follow subsequent paths when we can make an auto and have PP do it for us  ???
  //?????????????????????????????????????????????????????????????????????????????????????????????????????

  // public static Command followPathCommand(PathPlannerPath path) {
  //   return new FollowPathHolonomic(path, swerve::getPose, swerve::getChassisSpeeds, swerve::setChassisSpeeds,
  //       Constants.PathPlannerConstants.cfgHolonomicFollower, () -> {
  //         Alliance robotAlliance = allianceChooser.getSelected();
  //         robotAlliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get()
  //             : robotAlliance;

  //         if (robotAlliance == Alliance.Red) {
  //           return true;
  //         } else {
  //           return false;
  //         }
  //       }, swerve);
  // }

  // public static Command followMultiplePathsCommand(String... pathNames) {
  //   Command commandToFollow = new SequentialCommandGroup();

  //   for (String name : pathNames) {
  //     PathPlannerPath path = PathPlannerPath.fromPathFile(name);

  //     path.getAllPathPoints().get(-1);
  //     //do something to edit the desired velocity maybe

  //     commandToFollow = commandToFollow.andThen(followPathCommand(path));
  //   }

  //   return commandToFollow;
  // }
}