package frc.robot.commands.swerve;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class Autos {

    public static Command getStraightLine() {
        return AutoUtils.timeCommand(AutoUtils.getAutoPath("straight line", new Pose2d(4, 5, new Rotation2d())));
    }
}
