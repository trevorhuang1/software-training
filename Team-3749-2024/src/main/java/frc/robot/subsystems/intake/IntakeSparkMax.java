package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeSparkMax implements IntakeIO {

    private CANSparkMax intakeMotor = new CANSparkMax(0, MotorType.kBrushless);
    private DigitalInput photoelectricSensor = new DigitalInput(0);
    private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
    private double intakeGoalVolts = 0;

    public IntakeSparkMax() 
    {
        intakeEncoder.setVelocityConversionFactor((2*Math.PI)/60);
    }

    @Override
    public void updateData(IntakeData data) 
    {
        data.intakeVolts = intakeMotor.getBusVoltage();
        data.intakeVelocityRadPerSec = intakeEncoder.getVelocity();
        data.intakeTempCelcius = intakeMotor.getMotorTemperature();
        data.sensorTripped = photoelectricSensor.get();
        /* 
        if(data.sensorTripped)
        {
            Robot.led.setLEDPattern(LEDPattern.GREEN);
        }
        */
    }

   @Override
   public void setVoltage(double intakeVolts)
   {
        intakeGoalVolts = MathUtil.clamp(intakeVolts, -8, 8);
        intakeMotor.setVoltage(intakeGoalVolts);
   }

}
