package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

public class ShooterSparkMax implements ShooterIO {

    private CANSparkMax leftShooter = new CANSparkMax(1,MotorType.kBrushless);
    private CANSparkMax rightShooter = new CANSparkMax(2,MotorType.kBrushless);

    private RelativeEncoder leftEncoder = leftShooter.getEncoder();
    private RelativeEncoder rightEncoder = rightShooter.getEncoder();

    private double leftShooterGoalVolts = 0;
    private double rightShooterGoalVolts = 0;

    public ShooterSparkMax() 
    {
        rightShooter.setInverted(true);
        leftEncoder.setVelocityConversionFactor((2*Math.PI)/60);
        rightEncoder.setVelocityConversionFactor((2*Math.PI)/60);
    }

    @Override
    public void updateData(ShooterData data) 
    {
        data.leftShooterVolts = leftShooter.getBusVoltage();
        data.leftShooterVelocityRadPerSec = leftEncoder.getVelocity();
        data.leftShooterTempCelcius = leftShooter.getMotorTemperature();

        data.rightShooterVolts = rightShooter.getBusVoltage();
        data.rightShooterVelocityRadPerSec = rightEncoder.getVelocity();
        data.rightShooterTempCelcius = rightShooter.getMotorTemperature();
    }

   @Override
   public void setVoltage(double leftShooterVolts, double rightShooterVolts)
   {
        leftShooterGoalVolts = MathUtil.clamp(leftShooterVolts, -8, 8);
        rightShooterGoalVolts = MathUtil.clamp(rightShooterVolts, -8, 8);
        leftShooter.setVoltage(leftShooterGoalVolts);
        rightShooter.setVoltage(rightShooterGoalVolts);
   }

}
