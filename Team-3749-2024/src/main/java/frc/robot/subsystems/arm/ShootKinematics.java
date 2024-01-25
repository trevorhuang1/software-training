package frc.robot.subsystems.arm;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;

public class ShootKinematics {
    private static final Translation2d redSpeakerPosition = new Translation2d(0, 265.8); // rounded need to change
    private static final Translation2d blueSpeakerPosition = new Translation2d(1659.1, 265.8); // rounded need to change

    // 10.00 m = 1000
    // angle 0.0 = impossible to shoot from here
    private static final double[] distToAngle = new double[1001];
    private static double maxDist = 0.0;

    public static Pose2d ShootingPose2DCalculate(Pose2d currentPose2d){
        Translation2d distanceVector;
        Rotation2d angle;

        if (DriverStation.getAlliance().get() == Alliance.Red){
            distanceVector = currentPose2d.getTranslation().minus(redSpeakerPosition);
        } else {
            distanceVector = currentPose2d.getTranslation().minus(blueSpeakerPosition);
        }

        angle = new Rotation2d(Math.PI/2 - Math.atan2(Math.abs(distanceVector.getY()), Math.abs(distanceVector.getX())));

        if (angle.getDegrees() > 42.109 && getAngle(distanceVector.getNorm()) != 0.0){ 
            return changeRotation(currentPose2d, distanceVector);
        }
        
        return null;
    }

    private static Pose2d changeRotation(Pose2d currentPose2d, Translation2d distanceVector){
        return new Pose2d(currentPose2d.getTranslation(), distanceVector.getAngle().plus(new Rotation2d(Math.PI)));
    }

    private static double getAngle(double dist) {
        int distNum = (int)(Math.round(dist * 100.0));
        if (distNum < 0 || distNum > 1000) {
            return 0;
        }
        return distToAngle[distNum];
    }

    public static void loadDistCSV() throws FileNotFoundException, IOException {
        Path csvPath = Filesystem.getDeployDirectory().toPath().resolve("angles.csv");
        loadDistCSV(csvPath.toFile());
    }

    public static void loadDistCSV(File file) throws FileNotFoundException, IOException {
        BufferedReader reader = new BufferedReader(new FileReader(file));
        
        String line;
        while ((line = reader.readLine()) != null) {
            String[] values = line.split(",");
            double curDist = Double.parseDouble(values[0]);
            maxDist = Math.max(maxDist, curDist);
            distToAngle[(int)(curDist * 100)] = Double.parseDouble(values[1]);
        }

        reader.close();
    }

    // for testing load csv & other functionality
    public static void main(String[] args) throws FileNotFoundException, IOException {
        loadDistCSV(new File("src/main/deploy/angles.csv"));
        for (double i = 0.9; i < maxDist; i += 0.01) {
            i = Math.round(i*100)/100.0;
            double temp = i + Math.random()*.01;
            temp = Math.round(temp*1000)/1000.0;
            System.out.println(temp + " " + getAngle(temp));
        }
    }
}
