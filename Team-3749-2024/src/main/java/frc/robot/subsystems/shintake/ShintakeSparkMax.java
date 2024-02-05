package frc.robot.subsystems.shintake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShintakeSparkMax implements ShintakeIO {

    private CANSparkMax intakeMotor = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax leftShooter = new CANSparkMax(1,MotorType.kBrushless);
    private CANSparkMax rightShooter = new CANSparkMax(2,MotorType.kBrushless);

    private DigitalInput photoelectricSensor = new DigitalInput(0);


    private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
    private RelativeEncoder leftEncoder = leftShooter.getEncoder();
    private RelativeEncoder rightEncoder = rightShooter.getEncoder();

    private double shintakeGoalVolts = 0;
    private double leftShooterGoalVolts = 0;
    private double rightShooterGoalVolts = 0;

    public ShintakeSparkMax() 
    {
        rightShooter.setInverted(true);
        leftEncoder.setVelocityConversionFactor((2*Math.PI)/60);
        rightEncoder.setVelocityConversionFactor((2*Math.PI)/60);
        intakeEncoder.setVelocityConversionFactor((2*Math.PI)/60);
    }

    @Override
    public double[] getShooterEncoder()
    {
        double[] shooterEncoder = {leftEncoder.getVelocity(),rightEncoder.getVelocity()};
        return shooterEncoder;
    }

    @Override
    public double getIntakeEncoder()
    {
        return intakeEncoder.getVelocity();
    }

    @Override
    public void updateData(ShintakeData data) 
    {
        data.intakeVolts = intakeMotor.getBusVoltage();
        data.intakeVelocityRadPerSec = intakeEncoder.getVelocity();
        data.intakeTempCelcius = intakeMotor.getMotorTemperature();

        data.leftShooterVolts = leftShooter.getBusVoltage();
        data.leftShooterVelocityRadPerSec = leftEncoder.getVelocity();
        data.leftShooterTempCelcius = leftShooter.getMotorTemperature();

        data.rightShooterVolts = rightShooter.getBusVoltage();
        data.rightShooterVelocityRadPerSec = rightEncoder.getVelocity();
        data.rightShooterTempCelcius = rightShooter.getMotorTemperature();

        data.sensorTripped = photoelectricSensor.get();
        SmartDashboard.putBoolean("sensorTripped", photoelectricSensor.get());
        /* 
        if(data.sensorTripped)
        {
            Robot.led.setLEDPattern(LEDPattern.GREEN);
        }
        */
    }

   @Override
   public void setVoltage(double intakeVolts, double leftShooterVolts, double rightShooterVolts)
   {
        shintakeGoalVolts = MathUtil.clamp(intakeVolts, -8, 8);
        leftShooterGoalVolts = MathUtil.clamp(leftShooterVolts, -8, 8);
        rightShooterGoalVolts = MathUtil.clamp(rightShooterVolts, -8, 8);
        leftShooter.setVoltage(leftShooterGoalVolts);
        rightShooter.setVoltage(rightShooterGoalVolts);
        intakeMotor.setVoltage(shintakeGoalVolts);


    SmartDashboard.putNumber("intakeVolts",intakeMotor.getBusVoltage());
    SmartDashboard.putNumber("shooterVolts", leftShooter.getBusVoltage());
   }

}
