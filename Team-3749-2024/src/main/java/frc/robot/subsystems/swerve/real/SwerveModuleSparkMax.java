package frc.robot.subsystems.swerve.real;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.ModuleConstants;

public class SwerveModuleSparkMax implements SwerveModuleIO {
    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;

    private RelativeEncoder driveEncoder;
    private CANcoder absoluteEncoder;
    private double absoluteEncoderOffsetRad;

    private double driveAppliedVolts;
    private double turnAppliedVolts;

    public SwerveModuleSparkMax(int index) {
        driveMotor = new CANSparkMax(Constants.DriveConstants.driveMotorPorts[index], CANSparkMax.MotorType.kBrushless);
        turnMotor = new CANSparkMax(Constants.DriveConstants.turningMotorPorts[index],
                CANSparkMax.MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();

        absoluteEncoder = new CANcoder(Constants.DriveConstants.absoluteEncoderPorts[index]);
        absoluteEncoderOffsetRad = DriveConstants.driveAbsoluteEncoderOffsetDeg[index] / 180 * Math.PI;

        driveMotor.setInverted(DriveConstants.driveEncoderReversed[index]);
        driveEncoder.setInverted(DriveConstants.driveEncoderReversed[index]);
        driveEncoder.setPositionConversionFactor((1 / ModuleConstants.driveMotorGearRatio) * Math.PI
                * ModuleConstants.wheelDiameterMeters);
        driveEncoder.setVelocityConversionFactor((1 / ModuleConstants.driveMotorGearRatio) * (Math.PI
                * ModuleConstants.wheelDiameterMeters) * (1 / 60));

        turnMotor.setInverted(Constants.DriveConstants.turningEncoderReversed[index]);

        driveMotor.setSmartCurrentLimit(Constants.DriveConstants.driveMotorStallLimit,
                Constants.DriveConstants.driveMotorFreeLimit);
        turnMotor.setSmartCurrentLimit(Constants.DriveConstants.turnMotorStallLimit,
                Constants.DriveConstants.turnMotorFreeLimit);

    };

    @Override
    public void updateData(ModuleData data) {

        driveAppliedVolts = driveMotor.getBusVoltage();
        turnAppliedVolts = turnMotor.getBusVoltage();

        data.drivePositionM = getDrivePositionMeters();
        data.driveVelocityMPerSec = getDriveVelocityMetersPerSec();
        data.driveAppliedVolts = driveAppliedVolts;
        data.driveCurrentAmps = Math.abs(driveMotor.getOutputCurrent());
        data.driveTempCelcius = driveMotor.getMotorTemperature();

        data.turnAbsolutePositionRad = getTurningPositionRad();
        data.turnVelocityRadPerSec = getTurningVelocityRadPerSec();
        data.turnAppliedVolts = turnAppliedVolts;
        data.turnCurrentAmps = Math.abs(turnMotor.getOutputCurrent());
        data.turnTempCelcius = turnMotor.getMotorTemperature();
    };

    @Override
    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -Constants.DriveConstants.maxMotorVolts,
                Constants.DriveConstants.maxMotorVolts);
        driveMotor.setVoltage(driveAppliedVolts);
    };

    @Override
    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -Constants.DriveConstants.maxMotorVolts,
                Constants.DriveConstants.maxMotorVolts);
        turnMotor.setVoltage(turnAppliedVolts);
    };

    private double getDrivePositionMeters() {
        return driveEncoder.getPosition();
    };

    private double getTurningPositionRad() {
        return absoluteEncoder.getPosition().getValueAsDouble() * (2 * Math.PI) + absoluteEncoderOffsetRad;
    };

    private double getDriveVelocityMetersPerSec() {
        return driveEncoder.getVelocity();
    };

    private double getTurningVelocityRadPerSec() {
        return absoluteEncoder.getVelocity().getValueAsDouble() * (2 * Math.PI);
    };

    private void resetDriveEncoder() {
        driveEncoder.setPosition(0);
    };
}