package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.utils.Constants.Sim;

public class ArmSparkMax implements ArmIO {

    private CANSparkMax leftMotor = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax rightMotor = new CANSparkMax(1);


    private double appliedVolts = 0;
    private double previousVelocity = 0;

    public ArmSparkMax() {
        System.out.println("[Init] Creating ExampleIOSim");
    }

    @Override
    public void updateData(ArmData data) {
        previousVelocity = data.velocityRadPerSec;

        // update sim values every 0.02 sec
        armSim.update(Sim.loopPeriodSec);

        // distance traveled + Rad/Time * Time * diameter
        data.positionRad = armSim.getAngleRads();

        data.velocityRadPerSec =armSim.getVelocityRadPerSec();

        data.accelerationRadPerSecSquared = (armSim.getVelocityRadPerSec() - previousVelocity)/Sim.loopPeriodSec;

        data.appliedVolts = appliedVolts;
        // System.out.println(appliedVolts);

        data.currentAmps = Math.abs(armSim.getCurrentDrawAmps());

        data.tempCelcius = 0;
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -8.0, 8.0);
        armSim.setInputVoltage(appliedVolts);
    }

}
