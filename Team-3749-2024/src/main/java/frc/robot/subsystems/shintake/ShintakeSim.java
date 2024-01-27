package frc.robot.subsystems.shintake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShintakeSim implements ShintakeIO {

    private FlywheelSim intakeMotor = new FlywheelSim(DCMotor.getNEO(1),1, 0.01);
    private FlywheelSim leftShooter = new FlywheelSim(DCMotor.getNEO(1),1, 0.01);
    private FlywheelSim rightShooter = new FlywheelSim(DCMotor.getNEO(1),1, 0.01);

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
        data.intakeVolts = intakeMotor.getCurrentDrawAmps();
        data.intakeVelocity = intakeMotor.getAngularVelocityRPM();
        data.intakeTempCelcius = 0; //see FTC battery fire for guidance https://www.youtube.com/watch?v=eO9vHakAloU

        data.leftShooterVolts = leftShooter.getCurrentDrawAmps();
        data.leftShooterVelocity = leftShooter.getAngularVelocityRPM();
        data.leftShooterTempCelcius = 0;

        data.rightShooterVolts = rightShooter.getCurrentDrawAmps();
        data.rightShooterVelocity = rightShooter.getAngularVelocityRPM();
        data.rightShooterTempCelcius = 0;
    }

   @Override
   public void setVoltage(double intakeVolts, double leftShooterVolts, double rightShooterVolts)
   {
        shintakeGoalVolts = MathUtil.clamp(intakeVolts, -8, 8);
        leftShooterGoalVolts = MathUtil.clamp(leftShooterVolts, -8, 8);
        rightShooterGoalVolts = MathUtil.clamp(rightShooterVolts, -8, 8);
        leftShooter.setInputVoltage(leftShooterVolts);
        rightShooter.setInputVoltage(rightShooterVolts);
        intakeMotor.setInputVoltage(shintakeGoalVolts);
        //System.out.println("oc");

    SmartDashboard.putNumber("intakeVolts",intakeMotor.getCurrentDrawAmps());
    SmartDashboard.putNumber("shooterVolts", leftShooter.getCurrentDrawAmps());
   }

}
