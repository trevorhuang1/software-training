package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.utils.Constants.Sim;

public class ShooterSim implements ShooterIO {

    private FlywheelSim leftShooter = new FlywheelSim(DCMotor.getNEO(1),1, 0.04);
    private FlywheelSim rightShooter = new FlywheelSim(DCMotor.getNEO(1),1, 0.04);

    private double leftShooterGoalVolts = 0;
    private double rightShooterGoalVolts = 0;

    public ShooterSim() 
    {
        
    }

    @Override
    public void updateData(ShooterData data) 
    {
        leftShooter.update(Sim.loopPeriodSec);
        rightShooter.update(Sim.loopPeriodSec);

        data.leftShooterVolts = leftShooterGoalVolts;
        data.leftShooterVelocityRadPerSec = leftShooter.getAngularVelocityRadPerSec();
        data.leftShooterTempCelcius = 0;

        data.rightShooterVolts = rightShooterGoalVolts;
        data.rightShooterVelocityRadPerSec = rightShooter.getAngularVelocityRadPerSec();
        data.rightShooterTempCelcius = 0;
    }

   @Override
   public void setVoltage(double leftShooterVolts, double rightShooterVolts)
   {
        leftShooterGoalVolts = MathUtil.clamp(leftShooterVolts, -12, 12);
        rightShooterGoalVolts = MathUtil.clamp(rightShooterVolts, -12, 21);
        this.leftShooter.setInputVoltage(leftShooterGoalVolts);
        this.rightShooter.setInputVoltage(rightShooterGoalVolts);

   }

}
