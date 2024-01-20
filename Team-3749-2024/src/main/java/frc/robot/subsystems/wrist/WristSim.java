package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.utils.Constants.Sim;

public class WristSim implements WristIO {
    
    private FlywheelSim wristMotor = new FlywheelSim(DCMotor.getNEO(1),1, 0.01);
    private EncoderSim wristEncoder = new EncoderSim(new Encoder(1, 1));
    private double appliedVolts = 0.0;
    
    public WristSim() 
    {
        
    }

   @Override
   public void updateData(WristData data) 
   {
    wristMotor.update(Sim.loopPeriodSec);
    data.tempCelcius = 0;
    data.motorRPM = wristMotor.getAngularVelocityRPM();
    data.wristVoltage = wristMotor.getCurrentDrawAmps();
    data.appliedVolts = appliedVolts;
   }

   @Override
   public void setVoltage(double volts)
   {
    appliedVolts = MathUtil.clamp(volts,-8.0,8);
    wristMotor.setInputVoltage(appliedVolts);
   }

   @Override
   public double getEncoderValue()
   {
    return wristEncoder.getDistance(); //thihs has no relevance to the motor (woopsies!)
   }

}
