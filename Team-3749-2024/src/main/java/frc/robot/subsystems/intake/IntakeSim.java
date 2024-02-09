package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.utils.Constants.Sim;

public class IntakeSim implements IntakeIO {

    private FlywheelSim intakeMotor = new FlywheelSim(DCMotor.getNEO(1),1, 0.04);

    private double intakeGoalVolts = 0;

    public IntakeSim() 
    {
        
    }

    @Override
    public double getIntakeEncoder()
    {
        return intakeMotor.getAngularVelocityRadPerSec();
    }

    @Override
    public void updateData(IntakeData data) 
    {
        intakeMotor.update(Sim.loopPeriodSec);
        data.intakeVolts = intakeGoalVolts;
        data.intakeVelocityRadPerSec = intakeMotor.getAngularVelocityRadPerSec();
        data.intakeTempCelcius = 0; //see FTC battery fire for guidance https://www.youtube.com/watch?v=eO9vHakAloU
    }

   @Override
   public void setVoltage(double intakeVolts)
   {
        intakeGoalVolts = MathUtil.clamp(intakeVolts, -8, 8);
        this.intakeMotor.setInputVoltage(intakeVolts);
   }

}
