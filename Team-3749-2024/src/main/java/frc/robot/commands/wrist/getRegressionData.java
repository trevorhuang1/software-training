package frc.robot.commands.wrist;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class getRegressionData extends Command {
    private HashMap<String, ArrayList<Double>> dataColleciton = new HashMap<>();
    private double currentVoltage = 0;
    private double stopVelocity = 0.05;
    private double maxVelocity = 0.1;
    private double currentPosition = 0;
    private String mode = "forward";

    public getRegressionData() {
        dataColleciton.put("Wrist Position Degrees", new ArrayList<Double>());
        dataColleciton.put("Arm Position Degrees", new ArrayList<Double>());
        dataColleciton.put("Voltage", new ArrayList<Double>());

        addRequirements(Robot.wrist);
    }

    @Override
    public void execute() {

        Robot.wrist.setVoltage(currentVoltage);
        double currentVelocity = Robot.wrist.getVelocityRotation2d().getDegrees();
        currentPosition = Robot.wrist.getRotation2d().getDegrees();

        // progress forward
        if (mode.equals("forward")) {
            currentVoltage += 0.005;

            if (Math.abs(currentVelocity) >= Math.abs(maxVelocity)) {
                mode = "stop";
            }
        }
        // stop the bar
        if (mode.equals("stop")) {
            currentVoltage -= 0.005;

            if (Math.abs(currentVelocity) <= Math.abs(stopVelocity)) {
                mode = "scan up";
            }
        }

        // increase voltage gradually
        if (mode.equals("scan up")) {
            currentVoltage += 0.002;
            if (Math.abs(currentVelocity) >= Math.abs(stopVelocity)) {
                mode = "forward";
            }
        }

        // if we are stopped, record it
        if (Math.abs(currentVelocity) <= Math.abs(stopVelocity)) {
            dataColleciton.get("Wrist Position Degrees").add(currentPosition);
            dataColleciton.get("Arm Position Degrees").add(0.0);
            dataColleciton.get("Voltage").add(currentVoltage);
        }

    }

    @Override
    public boolean isFinished() {
        // this only works on the way up
        if (currentPosition > 60) {
            currentVoltage = 0;
            return true;
        }
        return false;
    }
}
