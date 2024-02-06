package frc.robot.subsystems.shintake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants.Sim;

public class ShintakeSim implements ShintakeIO {

    private FlywheelSim intakeMotor = new FlywheelSim(DCMotor.getNEO(1),1, 0.04);
    private FlywheelSim leftShooter = new FlywheelSim(DCMotor.getNEO(1),1, 0.04);
    private FlywheelSim rightShooter = new FlywheelSim(DCMotor.getNEO(1),1, 0.04);

    private double shintakeGoalVolts = 0;
    private double leftShooterGoalVolts = 0;
    private double rightShooterGoalVolts = 0;

    public ShintakeSim() 
    {
        
    }

    @Override
    public double[] getShooterEncoder()
    {
        double[] shooterEncoder = {leftShooter.getAngularVelocityRPM(),rightShooter.getAngularVelocityRPM()};
        return shooterEncoder;
    }

    @Override
    public double getIntakeEncoder()
    {
        return intakeMotor.getAngularVelocityRPM();
    }

    @Override
    public void updateData(ShintakeData data) 
    {
        intakeMotor.update(Sim.loopPeriodSec);
        leftShooter.update(Sim.loopPeriodSec);
        rightShooter.update(Sim.loopPeriodSec);
        data.intakeVolts = intakeMotor.getCurrentDrawAmps();
        data.intakeVelocityRadPerSec = intakeMotor.getAngularVelocityRadPerSec();
        data.intakeTempCelcius = 0; //see FTC battery fire for guidance https://www.youtube.com/watch?v=eO9vHakAloU

        data.leftShooterVolts = leftShooter.getCurrentDrawAmps();
        data.leftShooterVelocityRadPerSec = leftShooter.getAngularVelocityRadPerSec();
        data.leftShooterTempCelcius = 0;

        data.rightShooterVolts = rightShooter.getCurrentDrawAmps();
        data.rightShooterVelocityRadPerSec = rightShooter.getAngularVelocityRadPerSec();
        data.rightShooterTempCelcius = 0;
    }

   @Override
   public void setVoltage(double intakeVolts, double leftShooterVolts, double rightShooterVolts)
   {
        shintakeGoalVolts = MathUtil.clamp(intakeVolts, -8, 8);
        leftShooterGoalVolts = MathUtil.clamp(leftShooterVolts, -8, 8);
        rightShooterGoalVolts = MathUtil.clamp(rightShooterVolts, -8, 8);
        this.leftShooter.setInputVoltage(leftShooterGoalVolts);
        this.rightShooter.setInputVoltage(rightShooterGoalVolts);
        this.intakeMotor.setInputVoltage(shintakeGoalVolts);
    SmartDashboard.putNumber("intakeVolts",intakeMotor.getCurrentDrawAmps());
    SmartDashboard.putNumber("shooterVolts", leftShooter.getCurrentDrawAmps());
    SmartDashboard.putNumber("intakeVelocity", intakeMotor.getAngularVelocityRPM());
    SmartDashboard.putNumber("shooterVelocity", leftShooter.getAngularVelocityRPM());
   }

}
