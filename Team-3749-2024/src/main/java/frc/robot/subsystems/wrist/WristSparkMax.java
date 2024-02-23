package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
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
    private double previousVelocity = 0;

    public WristSparkMax() {
        
        wristEncoder.setPositionConversionFactor(2 * Math.PI );
        wristEncoder.setVelocityConversionFactor(2 * Math.PI * 1 /60.0);
        wristMotor.setInverted(true);
        wristMotor.setSmartCurrentLimit(40);
        wristMotor.setIdleMode(IdleMode.kCoast);

    }

    private double getAbsolutePosition(){
        double pos = wristEncoder.getPosition()-WristConstants.wristOffsetRad;
        if (pos > 2 * Math.PI){
            pos -= 2 * Math.PI;
        }
        if (pos<0){
            pos += 2 * Math.PI;
        }
        return pos;

    }
    

    @Override
    public void updateData(WristData data) {
        previousVelocity = data.velocityRadPerSec;

        data.positionRad = getAbsolutePosition() ;
        data.velocityRadPerSec = wristEncoder.getVelocity();
        data.accelerationRadPerSecSquared = (wristEncoder.getVelocity() - previousVelocity) / 0.02;

        data.appliedVolts = wristMotor.getBusVoltage() * wristMotor.getAppliedOutput();
        data.currentAmps = wristMotor.getOutputCurrent();
        data.tempCelcius = wristMotor.getMotorTemperature();

    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12, 12);
        wristMotor.setVoltage(appliedVolts);
    }

}
