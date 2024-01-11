package frc.robot.commands.swerve;

import java.util.HashMap;
import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.Sim.PIDValues;

public class PathPlannerUtils {
  private static Swerve swerve = Robot.swerve;
  static SendableChooser<Command> autoChooser;

  static boolean isFirstPath = true;
  public static Consumer<Pose2d> pathTargetPose = pose -> swerve.logDesiredOdometry(pose);

  public static void initPathPlannerUtils() {
    PathPlannerLogging.setLogTargetPoseCallback(pathTargetPose);
    AutoBuilder.configureHolonomic(swerve::getPose, swerve::resetOdometry, swerve::getChassisSpeeds,
        swerve::setChassisSpeeds,
        Constants.PathPlannerConstants.cfgHolonomicFollower, swerve);

    autoChooser = AutoBuilder.buildAutoChooser("TestAuto");

    SmartDashboard.putData("Choose Auto", autoChooser);
  }

  public static Command followPath(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if (isFirstPath) {
            swerve.resetOdometry(path.getPreviewStartingHolonomicPose());
            isFirstPath = !isFirstPath;
          }
        }),
        new FollowPathWithEvents(getHolonomicPathCommand(path), path, swerve::getPose));
  }

  public static Command followPath(String[] pathNames) {
    // SequentialCommandGroup sequentialCommand = new SequentialCommandGroup(new
    // InstantCommand(() -> {
    // // Reset odometry for the first path you run during auto
    // if (isFirstPath) {
    // PathPlannerPath path = PathPlannerPath.fromPathFile(pathNames[0]);
    // swerve.resetOdometry(path.getPreviewStartingHolonomicPose());
    // isFirstPath = !isFirstPath;
    // }
    // }));

    // for (int i = 0; i < pathNames.length; i++) {
    // PathPlannerPath path = PathPlannerPath.fromPathFile(pathNames[i]);
    // PathPlannerLogging.setLogTargetPoseCallback(pathTargetPose);

    // sequentialCommand = sequentialCommand
    // .andThen(new FollowPathWithEvents(getHolonomicPath(path), path,
    // swerve::getPose));
    // }

    return new PrintCommand("ran the tung");
  }

  public static Command getAutoPath() {
    return autoChooser.getSelected();
  }

  private static Command getHolonomicPathCommand(PathPlannerPath path) {
    return new FollowPathHolonomic(
        path,
        swerve::getPose, // Robot pose supplier
        swerve::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        swerve::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        Constants.PathPlannerConstants.cfgHolonomicFollower,
        swerve // Reference to this subsystem to set requirements
    );
  }
  public static Command getPathFindCommand(Pose2d targetPose, PathConstraints constraints) {

    Command pathfindingCommand = AutoBuilder.pathfindToPose(
      targetPose,
      constraints,
      0
    );
    return pathfindingCommand;
  }
  public static Command getPathFindToPreplannedCommand(String pathName, PathConstraints constraints) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
      path,
      constraints,
      3.0
    );
    return pathfindingCommand;
  }
}

/*
  public static Command followPath(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if (isFirstPath) {
            swerve.resetOdometry(path.getPreviewStartingHolonomicPose());
            isFirstPath = !isFirstPath;
          }
        }),
        new FollowPathWithEvents(getHolonomicPathCommand(path), path, swerve::getPose));

        */