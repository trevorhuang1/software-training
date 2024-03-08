package frc.robot.commands.swerve;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

public class Autos {

    public static Command getStraightLine() {
        return AutoUtils.getAutoPath("straight line", new Pose2d(4, 5, new Rotation2d()));
    }

    public static Command get4Piece() {
        // return AutoUtils.getCycle(0);
        return new SequentialCommandGroup(AutoUtils.getCycle(0),
                AutoUtils.getAutoPath("4 piece middle", new Pose2d(1.32, 5.44, new Rotation2d())),
                AutoUtils.getCycle(8));


        // return new SequentialCommandGroup(AutoUtils.getCycle(0),
        //         AutoUtils.getAutoPath("4 piece middle", new Pose2d(1.32, 5.44, new Rotation2d())),
        //         AutoUtils.getCycle(2));

    }

}
