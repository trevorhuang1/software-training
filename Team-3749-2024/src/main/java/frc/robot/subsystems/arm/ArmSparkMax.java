package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.utils.Constants.ArmConstants;
import frc.robot.utils.Constants.Sim;

public class ArmSparkMax implements ArmIO {

    private CANSparkMax leftMotor = new CANSparkMax(ArmConstants.leftID, CANSparkMax.MotorType.kBrushless);
    private CANSparkMax rightMotor = new CANSparkMax(ArmConstants.rightID, CANSparkMax.MotorType.kBrushless);

    private RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private RelativeEncoder rightEncoder = rightMotor.getEncoder();

    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(ArmConstants.encoderID);

    private double appliedVolts = 0;
    private double previousVelocity = 0;

    public ArmSparkMax() {
        System.out.println("[Init] Creating ExampleIOSim");
        absoluteEncoder.setPositionOffset(ArmConstants.encoderOffsetRad / (2 * Math.PI));
        absoluteEncoder.setDistancePerRotation(Math.PI * 2);
        leftEncoder.setVelocityConversionFactor(ArmConstants.relativeEncoderVelocityConversionFactor);
        rightEncoder.setVelocityConversionFactor(ArmConstants.relativeEncoderVelocityConversionFactor);
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        rightMotor.setInverted(true);


    }

    private double getAbsolutePositionRad() {
        return (absoluteEncoder.getAbsolutePosition() - absoluteEncoder.getPositionOffset())
                * absoluteEncoder.getDistancePerRotation();
    }

    private double getVelocityRadPerSec(){
        return (leftEncoder.getVelocity()+rightEncoder.getVelocity())/2;
    }


    @Override
    public void updateData(ArmData data) {
        previousVelocity = data.velocityRadPerSec;
        // distance traveled + Rad/Time * Time * diameter
        data.positionRad = getAbsolutePositionRad();

        data.velocityRadPerSec = getVelocityRadPerSec();

        data.accelerationRadPerSecSquared = (getVelocityRadPerSec() - previousVelocity) / 0.02;

        data.appliedVolts = appliedVolts;

        data.leftCurrentAmps = Math.abs(leftMotor.getOutputCurrent());
        data.rightCurrentAmps = Math.abs(rightMotor.getOutputCurrent());

        data.leftTempCelcius = leftMotor.getMotorTemperature();
        data.rightTempCelcius = rightMotor.getMotorTemperature();

    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -8.0, 8.0);
        leftMotor.setVoltage(appliedVolts);
        rightMotor.setVoltage(appliedVolts);

    }

}
