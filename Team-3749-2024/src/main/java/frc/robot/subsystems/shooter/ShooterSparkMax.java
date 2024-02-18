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

    private RelativeEncoder leftEncoder = topShooter.getEncoder();
    private RelativeEncoder rightEncoder = bottomShooter.getEncoder();

    private double leftShooterGoalVolts = 0;
    private double rightShooterGoalVolts = 0;

    public ShooterSparkMax() {
        bottomShooter.setInverted(true);
        bottomShooter.setIdleMode(IdleMode.kBrake);
        topShooter.setIdleMode(IdleMode.kBrake);
        topShooter.setSmartCurrentLimit(40);
                bottomShooter.setSmartCurrentLimit(40);

        leftEncoder.setVelocityConversionFactor((2 * Math.PI) / 60);
        rightEncoder.setVelocityConversionFactor((2 * Math.PI) / 60);
        topShooter.setIdleMode(IdleMode.kCoast);
        bottomShooter.setIdleMode(IdleMode.kCoast);     
    }

    @Override
    public void updateData(ShooterData data) {
        data.leftShooterVolts = topShooter.getBusVoltage();
        data.leftShooterVelocityRadPerSec = leftEncoder.getVelocity();
        data.leftShooterTempCelcius = topShooter.getMotorTemperature();

        data.rightShooterVolts = bottomShooter.getBusVoltage();
        data.rightShooterVelocityRadPerSec = rightEncoder.getVelocity();
        data.rightShooterTempCelcius = bottomShooter.getMotorTemperature();
    }

    @Override
    public void setVoltage(double leftShooterVolts, double rightShooterVolts) {
        leftShooterGoalVolts = MathUtil.clamp(leftShooterVolts, -8, 8);
        rightShooterGoalVolts = MathUtil.clamp(rightShooterVolts, -8, 8);
        topShooter.setVoltage(leftShooterGoalVolts);
        bottomShooter.setVoltage(rightShooterGoalVolts);
    }

}
