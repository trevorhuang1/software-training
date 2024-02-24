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
    private double currentVoltage = 0;
    private double stopVelocity = 0.02;
    private double maxVelocity = 0.05;
    private double currentPosition = 0;
    private String mode = "move";
    private boolean isForward = true;

    public getRegressionData(boolean isForward) {
        dataColleciton.put("Wrist Position Degrees", new ArrayList<Double>());
        dataColleciton.put("Arm Position Degrees", new ArrayList<Double>());
        dataColleciton.put("Voltage", new ArrayList<Double>());
        this.isForward = isForward;
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
            currentVoltage += isForward ? 0.0025 : -0.0025;

            if (Math.abs(currentVelocity) >= Math.abs(maxVelocity)) {
                mode = "stop";
            }
        }
        // stop the bar
        if (mode.equals("stop")) {
            currentVoltage += isForward ? -0.005 : 0.005;

            if (Math.abs(currentVelocity) <= Math.abs(stopVelocity)) {
                mode = "scan up";
            }
        }

        // increase voltage gradually
        if (mode.equals("scan up")) {
            currentVoltage += isForward ? 0.002 : -0.002;
            if (Math.abs(currentVelocity) >= Math.abs(stopVelocity)) {
                mode = "move";
            }
        }

        // if we are stopped, record it
        if (Math.abs(currentVelocity) <= Math.abs(stopVelocity)) {
            System.out.println(currentPosition + "," + 0 + "," + currentVoltage);
        }

        SmartDashboard.putNumber("regression volts", currentVoltage);
        SmartDashboard.putString("Regression Mode", mode);
    }

    @Override
    public boolean isFinished() {
        // this only works on the way up
        if ((isForward && currentPosition > 60) || (!isForward && currentPosition<60)) {
            currentVoltage = 0;
            return true;
        }
        return false;
    }
}
