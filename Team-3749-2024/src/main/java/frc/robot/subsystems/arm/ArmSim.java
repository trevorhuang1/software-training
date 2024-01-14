package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.utils.Constants.Sim;

public class ArmSim implements ArmIO {

    private SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getNEO(2),
        150,
        4,
        .93, 
        0,
        85,
        true,
        0);

    private double appliedVolts = 0;

    public ArmSim() {
        System.out.println("[Init] Creating ExampleIOSim");
    }

    @Override
    public void updateData(ArmData data) {

        // update sim values every 0.02 sec
        armSim.update(Sim.loopPeriodSec);

        // distance traveled + Rad/Time * Time * diameter
        data.positionRad = armSim.getAngleRads();

        data.velocityRadPerSec =armSim.getVelocityRadPerSec();
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
