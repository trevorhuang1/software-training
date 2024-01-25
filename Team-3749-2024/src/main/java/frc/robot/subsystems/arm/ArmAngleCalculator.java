package frc.robot.subsystems.arm;

import java.util.*;

import frc.robot.utils.Constants;

import java.io.*;

public class ArmAngleCalculator {
    public static double y_speaker = Constants.ArmConstants.speakerHeight;
    public static double y_arm = Constants.ArmConstants.armHeightOffset; // how high up the arm is
    public static double v_initial = Constants.ArmConstants.shooterVelocity;

    public static double arm_length = Constants.ArmConstants.armLength;

    // Gravity
    public static final double g = 9.8;

    // Range of Angles & Minimum Shooting Distance
    public static final double initial_angle = 140.0;
    public static final double final_angle = 0.0;
    public static double min_distance = 0.9;

    // Differentials & Margins of Error
    public static final double angle_increment = 0.01;
    public static final double dist_increment = 0.01;
    public static final double margin_of_error = 0.001;

    public static void main(String[] args) throws IOException, Exception {
        double previous_angle = 0;
        double current_angle = 0.01;
        String csv = "";

        // if angle stops increasing, then shooting downwards (STOP)
        for (double i = min_distance; current_angle > previous_angle; i = round(i + dist_increment)) {
            double angle = calculateAngle(i);
            csv += i + "," + angle + "\n";
            
            previous_angle = current_angle;
            current_angle = angle;
        }

        // output CSV to file
        PrintWriter pw = new PrintWriter("src/main/java/frc/robot/subsystems/arm/angles.csv");
        pw.print(csv);
        pw.close();
    }

    public static double calculateAngle(double x_dist) throws Exception {
         // if angles decrease then it is shooting downwards (STOP)
         for (double i = initial_angle; i >= final_angle; i = round(i - angle_increment)) {
               double initial_angle_rad = Math.toRadians(i);
               double shoot_angle_rad = Math.toRadians(140-i);
               
               double vx = Math.cos(shoot_angle_rad) * v_initial;
               double t = ( x_dist + arm_length*Math.cos(initial_angle_rad) ) / vx;
               double y = ( Math.sin(shoot_angle_rad) * v_initial * t ) - ( g/2 * t*t ) + ( arm_length * Math.sin(initial_angle_rad) ) + y_arm;

               if (Math.abs(y_speaker - y) <= margin_of_error) {
                   return i;
               }
         }

         throw new Exception("No angle for distance " + x_dist);
    }
    
    
    public static double round(double num) {
        return Math.round(num * 100) / 100.0;
    }
}
