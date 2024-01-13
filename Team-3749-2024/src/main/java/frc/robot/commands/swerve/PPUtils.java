package frc.robot.commands.swerve;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
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
  static SendableChooser<Command> autoChooser;

  static boolean isFirstPath = true;
  public static Consumer<Pose2d> pathTargetPose = pose -> swerve.logDesiredOdometry(pose);

  public static void initPPUtils() {
    PathPlannerLogging.setLogTargetPoseCallback(pathTargetPose);
    AutoBuilder.configureHolonomic(swerve::getPose, swerve::resetOdometry, swerve::getChassisSpeeds,
        swerve::setChassisSpeeds,
        Constants.PathPlannerConstants.cfgHolonomicFollower, swerve);

    autoChooser = AutoBuilder.buildAutoChooser("TestAuto");

    SmartDashboard.putData("Choose Auto", autoChooser);
  }

  public static void initPathCommands(HashMap<String, Command> commandList) {
    commandList.forEach((String cmdName, Command cmd) -> {
      NamedCommands.registerCommand(cmdName, cmd);
    });
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
        swerve// Reference to this subsystem to set requirements
    );
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

  /**
   * PathFinds to path then follows path
   */
  public static Command getPathFindThenFollowPathCommand(String pathName, PathConstraints constraints) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
        path,
        constraints, 0);
    return pathfindingCommand;
  }

  class Paths {
    public static Command getPathFindToPosesCommand(List<Pose2d> desiredPoses, PathConstraints constraints,
        GoalEndState endState) {
      Command command = new SequentialCommandGroup();

      PathPlannerPath path = new PathPlannerPath(PathPlannerPath.bezierFromPoses(desiredPoses), constraints, endState);

      getHolonomicPathCommand(path);

      return command;
    }
  }
}

/*
 * public static Command followPath(String pathName) {
 * PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
 * 
 * return new SequentialCommandGroup(
 * new InstantCommand(() -> {
 * // Reset odometry for the first path you run during auto
 * if (isFirstPath) {
 * swerve.resetOdometry(path.getPreviewStartingHolonomicPose());
 * isFirstPath = !isFirstPath;
 * }
 * }),
 * new FollowPathWithEvents(getHolonomicPathCommand(path), path,
 * swerve::getPose));
 * 
 */