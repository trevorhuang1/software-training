package frc.robot.commands.swerve;

import java.util.function.Consumer;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.Sim.PIDValues;

public class FollowPath {
  private static Swerve swerve = Robot.swerve;
  public static Consumer<Pose2d> pathTargetPose = pose -> swerve.logDesiredOdometry(pose);

  /***
   * @param swerve      the subsystem object. Do not make a new instance
   * @param trajectory  a viable trajectory object containing information
   *                    about where the robot should go
   * @param isFirstPath if it is, it will reset odometry at its current
   *                    position
   * @return a SwerveControllerCommand based on the trajectory
   * @summary takes a trajectory and moves on it
   */
  private static Command followTrajectoryCommand(PathPlannerPath path, boolean isFirstPath) {
    PathPlannerLogging.setLogTargetPoseCallback(pathTargetPose);

    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if (isFirstPath) {
            // swerve.resetGyro();
            // swerve.resetOdometry(path.getPreviewStartingHolonomicPose());
          }
        }),
        new FollowPathWithEvents(
            new FollowPathHolonomic(
                path,
                swerve::getPose, // Robot pose supplier
                swerve::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                swerve::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE
                // ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live
                    // in your Constants class
                    new PIDConstants(PIDValues.kP_PathPlannerDrive, 0.0, PIDValues.kD_PathPlannerDrive), // Translation PID constants
                    new PIDConstants(PIDValues.kP_PathPlannerTurn, 0.0, PIDValues.kD_PathPlannerTurn), // Rotation PID constants
                    5, // Max module speed, in m/s
                    Math.sqrt(2 * (DriveConstants.trackWidth * DriveConstants.trackWidth)), // Drivetrain radius
                    new ReplanningConfig() // Default path replanning config. See the API for the
                // options here
                ),
                swerve // Reference to this subsystem to set requirements
            ),
            path, // FollowPathWithEvents also requires the path
            swerve::getPose // FollowPathWithEvents also requires the robot pose supplier
        ));

  }

  public static Command followPath(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return followTrajectoryCommand(path, true);
  }

}
