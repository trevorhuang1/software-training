package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ShootPoseCalculator {
    private static final Translation2d redSpeakerPosition = new Translation2d(0, 265.8); // rounded need to change
    private static final Translation2d blueSpeakerPosition = new Translation2d(1659.1, 265.8); // rounded need to change

    public static Pose2d ShootingPose2DCalculate(Pose2d currentPose2d){

        
        return null;
    }

    private static boolean isInRange(Pose2d currentPose2d){
        Translation2d distanceVector;
        Rotation2d angle;

        if (DriverStation.getAlliance().get() == Alliance.Red){
            distanceVector = currentPose2d.getTranslation().minus(redSpeakerPosition);
        } else {
            distanceVector = currentPose2d.getTranslation().minus(blueSpeakerPosition);
        }

        angle = new Rotation2d(Math.PI/2 - Math.atan2(Math.abs(distanceVector.getY()), Math.abs(distanceVector.getX())));

        // TODO: ROHAN JUNEJA do the damn csv reading
        if (angle.getDegrees() > 42.109 && distanceVector.getNorm() < 100){ 
            return true;
        }

        return false;
    }
}
