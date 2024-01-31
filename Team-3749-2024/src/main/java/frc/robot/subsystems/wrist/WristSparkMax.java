package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

/*
 * note from jonathan:
 * always start the robot with the robot stowed!!!!
 * 
 * if you do not i will be very sad (and it also might destory itself)
 */

public class WristSparkMax implements WristIO {

    private CANSparkMax wristMotor = new CANSparkMax(3,MotorType.kBrushless);
    private RelativeEncoder wristEncoder = wristMotor.getEncoder();
    private double appliedVolts = 0;

    public WristSparkMax()
    {
        wristEncoder.setPositionConversionFactor(Math.PI/180);
    }

    @Override
    public void updateData(WristData data) 
    {
        data.tempCelcius = wristMotor.getMotorTemperature();
        data.velocityRadPerSec = (wristEncoder.getVelocity()/60)*(2*Math.PI);     // RPM -> rad/s
        data.wristVoltage = wristMotor.getBusVoltage();
        data.appliedVolts = appliedVolts;
        data.encoderDistance = wristEncoder.getPosition();
    }

    @Override
    public void setVoltage(double volts) 
    {
        appliedVolts = MathUtil.clamp(volts, -8, 8);
        wristMotor.setVoltage(appliedVolts);
    }

    public double getEncoderValue()
   {
    return wristEncoder.getPosition();
   }

}
