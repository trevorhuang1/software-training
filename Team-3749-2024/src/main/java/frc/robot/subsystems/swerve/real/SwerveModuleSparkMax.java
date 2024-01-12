package frc.robot.subsystems.swerve.real;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleIO.ModuleData;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.ModuleConstants;
import frc.robot.utils.Constants.Sim;

public class SwerveModuleSparkMax implements SwerveModuleIO {

    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder turnEncoder;

    private double drivePositionRad;
    private double turnPositionRad;

    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public SwerveModuleSparkMax(int drivePort, int turnPort) { // taken from drive constants
        
        driveMotor = new CANSparkMax(Constants.DriveConstants.driveMotorPorts[drivePort], CANSparkMax.MotorType.kBrushless);
        turnMotor = new CANSparkMax(Constants.DriveConstants.turningMotorPorts[turnPort], CANSparkMax.MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        drivePositionRad = Units.rotationsToRadians(driveEncoder.getPosition());
        turnPositionRad = Units.rotationsToRadians(turnEncoder.getPosition());

        driveMotor.setSmartCurrentLimit(30, 60);
        turnMotor.setSmartCurrentLimit(30, 60);

    };

    @Override
    public void updateData(ModuleData data) {

        double driveRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity());
        double turnRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(turnEncoder.getVelocity());

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        drivePositionRad = driveEncoder.getPosition();
        turnPositionRad = turnEncoder.getPosition();
  
        data.drivePositionM = data.drivePositionM + (turnRadPerSec * 0.02 * ModuleConstants.wheelDiameterMeters) / 2;
        data.driveVelocityMPerSec = driveRadPerSec * ModuleConstants.wheelDiameterMeters / 2;
        data.driveAppliedVolts = driveAppliedVolts;
        data.driveCurrentAmps = Math.abs(driveMotor.getOutputCurrent());
        data.driveTempCelcius = 0;
        data.turnAbsolutePositionRad = turnPositionRad;
        data.turnVelocityRadPerSec = turnRadPerSec;
        data.turnAppliedVolts = turnAppliedVolts;
        data.turnCurrentAmps = Math.abs(turnMotor.getOutputCurrent());
        data.turnTempCelcius = 0;

    };

    @Override
    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -8.0, 8.0);
        driveMotor.setVoltage(driveAppliedVolts);
    };

    @Override
    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -8.0, 8.0);
        turnMotor.setVoltage(turnAppliedVolts);
    };
}