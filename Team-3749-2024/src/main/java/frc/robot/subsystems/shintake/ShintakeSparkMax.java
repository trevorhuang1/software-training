package frc.robot.subsystems.shintake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShintakeSparkMax implements ShintakeIO {

    private CANSparkMax intakeMotor = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax leftShooter = new CANSparkMax(1,MotorType.kBrushless);
    private CANSparkMax rightShooter = new CANSparkMax(2,MotorType.kBrushless);


    private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
    private RelativeEncoder leftEncoder = leftShooter.getEncoder();
    private RelativeEncoder rightEncoder = rightShooter.getEncoder();

    private double shintakeGoalVolts = 0;
    private double leftShooterGoalVolts = 0;
    private double rightShooterGoalVolts = 0;

    public ShintakeSparkMax() 
    {
        rightShooter.setInverted(true);
    }

    @Override
    public double[] getShooterEncoder()
    {
        double[] shooterEncoder = {leftEncoder.getPosition(),rightEncoder.getPosition()};
        return shooterEncoder;
    }

    @Override
    public double getIntakeEncoder()
    {
        return intakeEncoder.getPosition();
    }

    @Override
    public void updateData(ShintakeData data) 
    {
        data.intakeVolts = intakeMotor.getBusVoltage();
        data.intakeVelocity = intakeEncoder.getVelocity();
        data.intakeTempCelcius = intakeMotor.getMotorTemperature();

        data.leftShooterVolts = leftShooter.getBusVoltage();
        data.leftShooterVelocity = leftEncoder.getVelocity();
        data.leftShooterTempCelcius = leftShooter.getMotorTemperature();

        data.rightShooterVolts = rightShooter.getBusVoltage();
        data.rightShooterVelocity = rightEncoder.getVelocity();
        data.rightShooterTempCelcius = rightShooter.getMotorTemperature();
    }

   @Override
   public void setVoltage(double intakeVolts, double leftShooterVolts, double rightShooterVolts)
   {
        shintakeGoalVolts = MathUtil.clamp(intakeVolts, -8, 8);
        leftShooterGoalVolts = MathUtil.clamp(leftShooterVolts, -8, 8);
        rightShooterGoalVolts = MathUtil.clamp(rightShooterVolts, -8, 8);
        leftShooter.setVoltage(leftShooterVolts);
        rightShooter.setVoltage(rightShooterVolts);
        intakeMotor.setVoltage(shintakeGoalVolts);


    SmartDashboard.putNumber("intakeVolts",intakeMotor.getBusVoltage());
    SmartDashboard.putNumber("shooterVolts", leftShooter.getBusVoltage());
   }

}
