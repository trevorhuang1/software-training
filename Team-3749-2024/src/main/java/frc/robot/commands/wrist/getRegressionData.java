package frc.robot.commands.wrist;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class getRegressionData extends Command {
    private HashMap<String, ArrayList<Double>> dataColleciton = new HashMap<>();
    private double stopVelocity = 0.02;
    private double maxVelocity = 0.04;
    private double currentVoltage = 0;
    private double currentPosition = 0;
    private double currentVelocity = 0;
    private String mode = "move";
    private boolean isForward = true;
    private boolean scanned = false;
    private double largestStop;

    public getRegressionData(boolean isForward) {
        dataColleciton.put("Wrist Position Degrees", new ArrayList<Double>());
        dataColleciton.put("Arm Position Degrees", new ArrayList<Double>());
        dataColleciton.put("Voltage", new ArrayList<Double>());
        this.isForward = isForward;
        largestStop = isForward ? -100 : 300;
        addRequirements(Robot.wrist);
    }

    @Override
    public void initialize() {
        System.out.println("4 Bar Position Deg, Arm Position Deg, Gravity Voltage");
    }

    @Override
    public void execute() {

        Robot.wrist.setVoltage(currentVoltage);
        double currentVelocity = Units.radiansToDegrees(Robot.wrist.getVelocityRadPerSec());
        currentPosition = Units.radiansToDegrees(Robot.wrist.getPositionRad());

        // progress forward
        if (mode.equals("move")) {
            currentVoltage += isForward ? 0.00125 : -0.00125;

            if (Math.abs(currentVelocity) >= Math.abs(maxVelocity)) {
                if (isForward && currentPosition > largestStop) {
                    mode = "stop";
                } else if (!isForward && currentPosition < largestStop) {
                    mode = "stop";

                }
                // System.out.println("stop");

            }
        }
        // stop the bar
        if (mode.equals("stop")) {
            currentVoltage += currentVelocity > 0 ? -0.006 : 0.006;

            if (Math.abs(currentVelocity) <= Math.abs(stopVelocity)) {
                mode = "scan up";
                scanned = false;
                if (isForward && currentPosition > largestStop){
                    largestStop = currentPosition;
                } else if (!isForward && currentPosition<largestStop){
                    largestStop = currentPosition;
                }
            }

        }

        // increase voltage gradually
        if (mode.equals("scan up")) {
            currentVoltage += isForward ? 0.004 : -0.004;
            // System.out.println(currentVoltage);
            if (Math.abs(currentVelocity) >= Math.abs(stopVelocity)) {
                mode = "move";
            } else if (!scanned) {
                System.out
                        .println(currentPosition + "," + Robot.arm.getRotation2d().getDegrees() + "," + currentVoltage);
                scanned = true;
            }
        }

        SmartDashboard.putNumber("regression volts", currentVoltage);
        SmartDashboard.putString("Regression Mode", mode);
    }

    @Override
    public boolean isFinished() {
        // this only works on the way up
        if (isForward && currentVoltage < -0.0041 && currentVelocity < 0) {
            currentVoltage = 0;
            currentPosition = 0;
            currentVelocity = 0;
            mode = "move";
            scanned = false;
            return true;
        } else if (!isForward && currentVoltage > 0.0041 && currentVelocity > 0) {
            currentVoltage = 0;
            currentPosition = 0;
            currentVelocity = 0;
            mode = "move";
            scanned = false;
            return true;
        }
        return false;
    }
}
