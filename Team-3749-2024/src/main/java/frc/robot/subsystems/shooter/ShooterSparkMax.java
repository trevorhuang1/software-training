package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import frc.robot.utils.Constants.ShintakeConstants;

public class ShooterSparkMax implements ShooterIO {

    private CANSparkMax topShooter = new CANSparkMax(ShintakeConstants.shooterTopId, MotorType.kBrushless);
    private CANSparkMax bottomShooter = new CANSparkMax(ShintakeConstants.shooterBottomId, MotorType.kBrushless);

    private RelativeEncoder topEncoder = topShooter.getEncoder();
    private RelativeEncoder bottomEncoder = bottomShooter.getEncoder();

    private double bottomShooterGoalVolts = 0;
    private double topShooterGoalVolts = 0;

    public ShooterSparkMax() {
        bottomShooter.setInverted(true);

        // bottomShooter.setIdleMode(IdleMode.kBrake);
        // topShooter.setIdleMode(IdleMode.kBrake);
        topShooter.setSmartCurrentLimit(40);
        bottomShooter.setSmartCurrentLimit(40);

        bottomEncoder.setVelocityConversionFactor((2 * Math.PI) / 60);
        topEncoder.setVelocityConversionFactor((2 * Math.PI) / 60);

        topEncoder.setPositionConversionFactor(2 * Math.PI);
        bottomEncoder.setPositionConversionFactor(2 * Math.PI);

        topShooter.setIdleMode(IdleMode.kCoast);
        bottomShooter.setIdleMode(IdleMode.kCoast);     
    }


    @Override
    public void updateData(ShooterData data) {
        data.topShooterVolts = topShooter.getBusVoltage() * topShooter.getAppliedOutput();
        data.topShooterVelocityRadPerSec = topEncoder.getVelocity();
        data.topShooterTempCelcius = topShooter.getMotorTemperature();
        data.topShooterPositionRad = topEncoder.getPosition();

        data.bottomShooterVolts = bottomShooter.getBusVoltage() * bottomShooter.getAppliedOutput();
        data.bottomShooterVelocityRadPerSec = bottomEncoder.getVelocity();
        data.bottomShooterTempCelcius = bottomShooter.getMotorTemperature();
        data.bottomShooterPositionRad = bottomEncoder.getPosition();
    }

    @Override
    public void setVoltage(double topShooterVolts, double bottomShooterVolts) {
        bottomShooterGoalVolts = MathUtil.clamp(bottomShooterVolts, -12, 12);
        topShooterGoalVolts = MathUtil.clamp(topShooterVolts, -12, 12);
        topShooter.setVoltage(topShooterGoalVolts);
        bottomShooter.setVoltage(bottomShooterGoalVolts);
    }

}
