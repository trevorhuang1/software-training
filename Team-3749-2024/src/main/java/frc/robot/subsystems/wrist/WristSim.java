package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants.Sim;

public class WristSim implements WristIO {
    
    private SingleJointedArmSim wristMotor = new SingleJointedArmSim(DCMotor.getNEO(1),
        333.333,
        0.755, //2578.65
        Units.inchesToMeters(21.1), //  21.1 in
        //how it FEELS to steal from the arm branch (they're us but better)

        Units.degreesToRadians(0),
        Units.degreesToRadians(40),
        true,
        Units.degreesToRadians(0));
    private double appliedVolts = 0.0;
    private double distanceRotated = 0.0;
    
    public WristSim() 
    {
        
    }

   @Override
   public void updateData(WristData data) 
   {
    wristMotor.update(Sim.loopPeriodSec);
    data.tempCelcius = 0;
    data.velocityRadPerSec = wristMotor.getVelocityRadPerSec();
    data.wristVoltage = wristMotor.getCurrentDrawAmps();
    data.appliedVolts = appliedVolts;
    data.encoderDistance = (data.encoderDistance + (data.velocityRadPerSec * 0.02));
    distanceRotated = data.encoderDistance;
    SmartDashboard.putNumber("encoderDist", Math.toDegrees(data.encoderDistance));
    SmartDashboard.putNumber("wristVolts", data.appliedVolts);
   }

   @Override
   public void setVoltage(double volts)
   {
    appliedVolts = MathUtil.clamp(volts,-8.0,8);
    wristMotor.setInputVoltage(appliedVolts);
    SmartDashboard.putNumber("wristSpeed", appliedVolts);
   }

   @Override
   public double getEncoderValue()
   {
    return distanceRotated;
   }

}
