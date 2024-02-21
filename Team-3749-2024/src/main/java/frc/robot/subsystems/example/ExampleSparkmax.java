package frc.robot.subsystems.example;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.ModuleConstants;
import frc.robot.utils.Constants.Sim;

public class ExampleSparkmax implements ExampleIO {

    private CANSparkMax motorTop = new CANSparkMax(12, MotorType.kBrushless);
    private CANSparkMax motorBottom = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax motorIntake = new CANSparkMax(6, MotorType.kBrushless);

    private double appliedVolts = 0.0;
    private int currentLimit = 0;

    public ExampleSparkmax() {
        System.out.println("[Init] Creating ExampleIOSim");

        motorTop.setSmartCurrentLimit(40);
        motorBottom.setSmartCurrentLimit(40);
        motorIntake.setSmartCurrentLimit(40);
    }

    @Override
    public void updateData(ExampleData data) {

        // update sim values every 0.02 sec

        // distance traveled + Rad/Time * Time * diameter


    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        motorBottom.setVoltage(volts*1);
        motorTop.setVoltage(volts*1);
        motorBottom.setVoltage(-volts*.25);


    }

}
