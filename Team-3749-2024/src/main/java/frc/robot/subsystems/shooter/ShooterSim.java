package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.utils.MiscConstants.Sim;

public class ShooterSim implements ShooterIO {

    private FlywheelSim bottomShooter = new FlywheelSim(DCMotor.getNEO(1),1, 0.04);
    private FlywheelSim topShooter = new FlywheelSim(DCMotor.getNEO(1),1, 0.04);

    private double bottomShooterGoalVolts = 0;
    private double topShooterGoalVolts = 0;

    public ShooterSim() 
    {
        
    }

    @Override
    public void updateData(ShooterData data) 
    {
        bottomShooter.update(Sim.loopPeriodSec);
        topShooter.update(Sim.loopPeriodSec);

        data.bottomShooterVolts = bottomShooterGoalVolts;
        data.bottomShooterVelocityRadPerSec = bottomShooter.getAngularVelocityRadPerSec();
        data.bottomShooterTempCelcius = 0;

        data.topShooterVolts = topShooterGoalVolts;
        data.topShooterVelocityRadPerSec = topShooter.getAngularVelocityRadPerSec();
        data.topShooterTempCelcius = 0;
    }

   @Override
   public void setVoltage(double bottomShooterVolts, double topShooterVolts)
   {
        bottomShooterGoalVolts = MathUtil.clamp(bottomShooterVolts, -12, 12);
        topShooterGoalVolts = MathUtil.clamp(topShooterVolts, -12, 21);
        this.bottomShooter.setInputVoltage(bottomShooterGoalVolts);
        this.topShooter.setInputVoltage(topShooterGoalVolts);

   }

}
