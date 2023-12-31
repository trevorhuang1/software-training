package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.GyroIO.GyroData;
import frc.robot.subsystems.swerve.SwerveModuleIO.ModuleData;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.ModuleConstants;
import frc.robot.utils.Constants.RobotType;

public class SwerveModule {
    
    private double index;
    private SwerveModuleState desiredState = new SwerveModuleState();
    private final PIDController turningPidController;
    private final PIDController drivingPidController;
    private final SimpleMotorFeedforward drivingFeedFordward;

    private ModuleData moduleData = new ModuleData();
    private SwerveModuleIO moduleIO;

    
    public SwerveModule(int i) {
        index = i;

        if (Constants.ROBOT_TYPE == RobotType.SIM) {

            moduleIO = new SwerveModuleSim();

            drivingPidController = new PIDController(ModuleConstants.kPDrivingSim, 0, 0);
            drivingFeedFordward = new SimpleMotorFeedforward(ModuleConstants.kSDrivingSim, ModuleConstants.kVDrivingSim);
            turningPidController = new PIDController(ModuleConstants.kPTurningSim, 0, 0);
            turningPidController.enableContinuousInput(0, 2 * Math.PI);

        }
         else {
            // real swerve module instatiation here

            drivingPidController = new PIDController(ModuleConstants.kPDrivingReal, 0, 0);
            drivingFeedFordward = new SimpleMotorFeedforward(ModuleConstants.kSDrivingReal, ModuleConstants.kVDrivingReal);
            turningPidController = new PIDController(ModuleConstants.kPTurningReal, 0, 0);
            turningPidController.enableContinuousInput(0, 2 * Math.PI);


        }

    }

    
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                moduleData.driveVelocityMPerSec * Constants.ModuleConstants.wheelDiameterMeters / 2,
                new Rotation2d(moduleData.turnAbsolutePositionRad));
    }
    
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(moduleData.drivePositionM, new Rotation2d(moduleData.turnAbsolutePositionRad));
    }

    public SwerveModuleState getDesiredState(){
        return desiredState;
    }



    public void setDesiredState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);
        this.desiredState = state;
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        double drive_volts = drivingFeedFordward.calculate(state.speedMetersPerSecond)
                + drivingPidController.calculate(moduleData.driveVelocityMPerSec, state.speedMetersPerSecond);

        double turning_volts = turningPidController.calculate(moduleData.turnAbsolutePositionRad, state.angle.getRadians());
        // Make a drive PID Controller
        moduleIO.setDriveVoltage(drive_volts);
        moduleIO.setTurnVoltage(turning_volts);

    }

    public void stop() {
        moduleIO.setDriveVoltage(0);
        moduleIO.setTurnVoltage(0);
    }
    // called within the swerve subsystem's periodic
    public void periodic(){
        moduleIO.updateData(moduleData);
    }
}
