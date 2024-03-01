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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        // absoluteEncoder.setPositionOffset(ArmConstants.encoderOffsetRad / (2 * Math.PI));
        // absoluteEncoder.setDistancePerRotation(Math.PI * 2);
        
        leftEncoder.setVelocityConversionFactor(1 / ArmConstants.gearRatio * Units.rotationsPerMinuteToRadiansPerSecond(1));
        leftEncoder.setPositionConversionFactor(1 / ArmConstants.gearRatio * Units.rotationsToRadians(1));

        rightEncoder.setVelocityConversionFactor(1 / ArmConstants.gearRatio * Units.rotationsPerMinuteToRadiansPerSecond(1));
        rightEncoder.setPositionConversionFactor(1 / ArmConstants.gearRatio * Units.rotationsToRadians(1));


        rightMotor.setInverted(true);

        rightMotor.setSmartCurrentLimit(40);
        leftMotor.setSmartCurrentLimit(40);
        rightMotor.setIdleMode(IdleMode.kCoast);
        leftMotor.setIdleMode(IdleMode.kCoast);


    }

    private double getAbsolutePositionRad() {
        return (absoluteEncoder.getAbsolutePosition() - absoluteEncoder.getPositionOffset())
                * absoluteEncoder.getDistancePerRotation();
    }

    private double getRelativePositionRad(){
        SmartDashboard.putNumber("Left", leftEncoder.getPosition());
        SmartDashboard.putNumber("right", rightEncoder.getPosition());
        
        return (rightEncoder.getPosition() + leftEncoder.getPosition())/2;
    }

    private double getVelocityRadPerSec(){
        return (leftEncoder.getVelocity()+rightEncoder.getVelocity())/2;
    }


    @Override
    public void updateData(ArmData data) {
        previousVelocity = data.velocityRadPerSec;
        // distance traveled + Rad/Time * Time * diameter
        data.positionRad = getRelativePositionRad();

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
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        leftMotor.setVoltage(appliedVolts);
        rightMotor.setVoltage(appliedVolts);

    }

    @Override
    public void setBreakMode(){
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
    }
    @Override
    public void setCoastMode(){
        leftMotor.setIdleMode(IdleMode.kCoast);
        rightMotor.setIdleMode(IdleMode.kCoast);

    }

}
