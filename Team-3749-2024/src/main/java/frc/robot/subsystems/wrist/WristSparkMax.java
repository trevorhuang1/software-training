package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import frc.robot.utils.Constants.WristConstants;

/*
 * note from jonathan:
 * always start the robot with the robot stowed!!!!
 * 
 * if you do not i will be very sad (and it also might destory itself)
 */

public class WristSparkMax implements WristIO {

    private CANSparkMax wristMotor = new CANSparkMax(WristConstants.wristId, MotorType.kBrushless);
    private SparkAbsoluteEncoder wristEncoder = wristMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    private double appliedVolts = 0;

    public WristSparkMax() {
        wristEncoder.setPositionConversionFactor(2 * Math.PI / WristConstants.gearRatio);
        wristEncoder.setVelocityConversionFactor(2 * Math.PI / WristConstants.gearRatio * 1 /60);
        wristEncoder.setInverted(false);
        
    }

    @Override
    public void updateData(WristData data) {
        data.tempCelcius = wristMotor.getMotorTemperature();
        data.velocityRadPerSec = (wristEncoder.getVelocity() / 60) * (2 * Math.PI); // RPM -> rad/s
        data.wristVoltage = wristMotor.getBusVoltage();
        data.appliedVolts = appliedVolts;
        data.positionRad = wristEncoder.getPosition();
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12, 12);
        wristMotor.setVoltage(appliedVolts);
    }

}
