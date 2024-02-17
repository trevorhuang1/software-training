package frc.robot.subsystems.swerve.real;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.ModuleConstants;

public class SwerveModuleSparkMax implements SwerveModuleIO {
    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;

    private CANcoder absoluteEncoder;
    private double absoluteEncoderOffsetRad;

    private double driveAppliedVolts;
    private double turnAppliedVolts;

    private int index;

    public SwerveModuleSparkMax(int index) {
        driveMotor = new CANSparkMax(Constants.DriveConstants.driveMotorPorts[index], CANSparkMax.MotorType.kBrushless);
        turnMotor = new CANSparkMax(Constants.DriveConstants.turningMotorPorts[index],
                CANSparkMax.MotorType.kBrushless);

        absoluteEncoder = new CANcoder(Constants.DriveConstants.absoluteEncoderPorts[index]);
        absoluteEncoderOffsetRad = Units.degreesToRadians(DriveConstants.absoluteEncoderOffsetDeg[index]);

        SmartDashboard.putNumber("offset" + index, Units.radiansToDegrees(absoluteEncoderOffsetRad));
        SmartDashboard.putNumber("rotations" + index, Units.radiansToDegrees(getAbsoluteTurningPositionRad()));

        turnMotor.setInverted(Constants.DriveConstants.turningMotorReversed[index]);
        turnMotor.getEncoder().setPositionConversionFactor(1 / ModuleConstants.turnMotorGearRatio * (2 * Math.PI));
        turnMotor.getEncoder()
                .setVelocityConversionFactor(1 / ModuleConstants.turnMotorGearRatio * (2 * Math.PI) * (1 / 60));
        // turnMotor.getEncoder().setPosition(Units.radiansToRotations(getAbsoluteTurningPositionRad()));

        driveMotor.setInverted(DriveConstants.driveMotorReversed[index]);

        driveMotor.getEncoder().setPositionConversionFactor((1 / ModuleConstants.driveMotorGearRatio) * Math.PI
                * ModuleConstants.wheelDiameterMeters);

        
        // driveMotor.getEncoder().setVelocityConversionFactor((1 / ModuleConstants.driveMotorGearRatio) * (Math.PI
        //         * ModuleConstants.wheelDiameterMeters) * (1 / 60));

                
        driveMotor.setSmartCurrentLimit(Constants.DriveConstants.driveMotorStallLimit,
                Constants.DriveConstants.driveMotorFreeLimit);
        turnMotor.setSmartCurrentLimit(Constants.DriveConstants.turnMotorStallLimit,
                Constants.DriveConstants.turnMotorFreeLimit);

        driveMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setIdleMode(IdleMode.kBrake);

        this.index = index;
        System.out.println("Conversion factor: " + index);
        System.out.println(driveMotor.getEncoder().getVelocityConversionFactor());


    };

    @Override
    public void updateData(ModuleData data) {

        driveAppliedVolts = driveMotor.getBusVoltage() * driveMotor.getAppliedOutput();
        turnAppliedVolts = turnMotor.getBusVoltage() * turnMotor.getAppliedOutput();

        data.drivePositionM = getDrivePositionMeters();
        data.driveVelocityMPerSec = getDriveVelocityMetersPerSec();
        data.driveAppliedVolts = driveAppliedVolts;
        data.driveCurrentAmps = Math.abs(driveMotor.getOutputCurrent());
        data.driveTempCelcius = driveMotor.getMotorTemperature();

        data.turnAbsolutePositionRad = getAbsoluteTurningPositionRad();
        data.turnVelocityRadPerSec = getAbsoluteTurninVelocityRadPerSec();
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
        return driveMotor.getEncoder().getPosition();
    };

    private double getAbsoluteTurningPositionRad() {
        return Units.rotationsToRadians(absoluteEncoder.getPosition().getValueAsDouble()) + absoluteEncoderOffsetRad;
    };

    private double getAbsoluteTurninVelocityRadPerSec() {
        return Units.rotationsToRadians(absoluteEncoder.getVelocity().getValueAsDouble());
    };

    private double getDriveVelocityMetersPerSec() {
   
        return driveMotor.getEncoder().getVelocity();
    };

}