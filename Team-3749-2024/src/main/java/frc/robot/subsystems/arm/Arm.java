package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.arm.ArmIO.ArmData;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.SuperStructureStates;
import frc.robot.utils.UtilityFunctions;

public class Arm extends SubsystemBase {

    private ArmData data = new ArmData();
    private ArmIO armIO;

    private ProfiledPIDController feedback = new ProfiledPIDController(ArmConstants.stowedPID.kP,
            ArmConstants.stowedPID.kI,
            ArmConstants.stowedPID.kD,
            ArmConstants.deployedConstraints);

    private ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.stowedkS,
            ArmConstants.stowedkG,
            ArmConstants.stowedkV);

    private Mechanism2d mechanism = new Mechanism2d(2.5, 2);
    private MechanismRoot2d mechanismArmPivot = mechanism.getRoot("mechanism arm pivot", 1, 0.5);
    private MechanismLigament2d mechanismArm = mechanismArmPivot
            .append(new MechanismLigament2d("mechanism arm", .93, 0));
    private ShuffleData<Double> positionLog = new ShuffleData<Double>(this.getName(), "position",
            0.0);
    private ShuffleData<Double> velocityLog = new ShuffleData<Double>(this.getName(), "velocity",
            0.0);
    private ShuffleData<Double> accelerationLog = new ShuffleData<Double>(this.getName(), "acceleration",
            0.0);
    private ShuffleData<Double> voltageLog = new ShuffleData<Double>(this.getName(), "voltage",
            0.0);
    private ShuffleData<Double> leftCurrentLog = new ShuffleData<Double>(this.getName(), "left current",
            0.0);
    private ShuffleData<Double> rightCurrentLog = new ShuffleData<Double>(this.getName(), "right current",
            0.0);
    private ShuffleData<Double> goalLog = new ShuffleData<Double>(this.getName(), "goal",
            0.0);
    private ShuffleData<Double> setpointPositionLog = new ShuffleData<Double>(this.getName(), "setpoint position",
            0.0);
    private ShuffleData<Double> setpointVelocityLog = new ShuffleData<Double>(this.getName(), "setpoint velocity",
            0.0);
    private ShuffleData<Double> setpointAccelerationLog = new ShuffleData<Double>(this.getName(),
            "setpoint acceleration", 0.0);
    private ShuffleData<Double> errorPositionLog = new ShuffleData<Double>(this.getName(), "error position",
            0.0);
    private ShuffleData<Double> errorVelocityLog = new ShuffleData<Double>(this.getName(), "error velocity",
            0.0);
    private ShuffleData<Double> errorAccelerationLog = new ShuffleData<Double>(this.getName(), "error acceleration",
            0.0);

    private ShuffleData<Boolean> deployedModeLog = new ShuffleData<Boolean>(this.getName(), "deployed mode",
            false);
    private ShuffleData<String> stateLog = new ShuffleData<String>(this.getName(), "state",
            ArmStates.STOW.name());

    private double accelerationSetpoint = 0;
    private double prevSetpointVelocity = 0;

    private boolean isKilled = false;
    private boolean isEnabled = false;

    private boolean deployedMode = false;
    private ArmStates state = ArmStates.STOW;

    public Arm() {
        if (Robot.isSimulation()) {
            armIO = new ArmSim();
        } else {
            armIO = new ArmSparkMax();
        }
        feedback.setGoal(ArmConstants.stowPositionRad);

    }

    public double getPositionRad() {
        return data.positionRad;
    }

    public double getVelocityRadPerSec() {
        return data.velocityRadPerSec;
    }

    public void setGoal(ArmStates state) {
        if (state == ArmStates.AMP) {
            feedback.setGoal(ArmConstants.ampPositionRad);
        }
        if (state == ArmStates.STOW) {
            feedback.setGoal(ArmConstants.stowPositionRad);
        }
        if (state == ArmStates.CLIMB) {
            feedback.setGoal(ArmConstants.climbPositionRad);
        }

    }

    public void setGoal(double goalRad) {
        // state = ArmStates.SHOOT;
        feedback.setGoal(goalRad);
    }

    public double getGoal() {
        return feedback.getGoal().position;
    }

    public State getSetpoint() {

        return feedback.getSetpoint();
    }

    public ArmStates getState() {
        return state;
    }

    public void setDeployedMode(boolean isDeployed) {
        deployedMode = isDeployed;
        if (isDeployed) {
            feedback.setConstraints(ArmConstants.deployedConstraints);
            feedforward = new ArmFeedforward(ArmConstants.deployedkS,
                    ArmConstants.deployedkG,
                    ArmConstants.deployedkV);
        } else {
            feedback.setConstraints(ArmConstants.stowedConstraints);
            feedforward = new ArmFeedforward(ArmConstants.stowedkS,
                    ArmConstants.stowedkG,
                    ArmConstants.stowedkV);
        }
    }

    public void setVoltage(double volts) {
        // if (isKilled) {
        //     armIO.setVoltage(0);
        // } else {
            armIO.setVoltage(volts);
        // }
    }

    private ShuffleData<Double> kPData = new ShuffleData(this.getName(),
            "kpdata", 0.0);
    private ShuffleData<Double> kVData = new ShuffleData(this.getName(),
            "kVdata", 0.0);

    private ShuffleData<Double> kAData = new ShuffleData(this.getName(),
            "kAdata", 0.0);
    private ShuffleData<Double> kSData = new ShuffleData(this.getName(),
            "kSdata", 0.0);
    private ShuffleData<Double> kGData = new ShuffleData(this.getName(),
            "kGdata", 0.0);
    // private ShuffleData<Double> kDData = new ShuffleData(this.getName(),
    // "kDdata", 0.0);

    public void moveToGoal() {

        double feedback = calculatePID(getPositionRad());
        State setpoint = getSetpoint();
        double accelerationSetpoint = (setpoint.velocity - prevSetpointVelocity) / 0.02;
        prevSetpointVelocity = setpoint.velocity;

        if (setpoint.position == ArmConstants.stowPositionRad
                && UtilityFunctions.withinMargin(0.035, getPositionRad(), ArmConstants.stowPositionRad)
                && UtilityFunctions.withinMargin(0.1, getVelocityRadPerSec(), 0)) {
            setVoltage(0);
            return;
        }

        double feedforward;
        feedforward = calculateFF(getPositionRad(), setpoint.velocity, accelerationSetpoint);
        if (setpoint.velocity == 0) {
            // have the kS help the PID when stationary
            feedforward += Math.signum(feedback) * ArmConstants.stowedkS * 0.6;
        }

        setVoltage(feedforward + feedback);

        // double volts = 0;
        // volts +=kSData.get(); //Math.signum(setpoint.velocity)
        // volts += Math.cos(getPositionRad()) * kGData.get();
        // volts += setpoint.velocity * kVData.get();
        // volts += accelerationSetpoint * kAData.get();
        // volts += (setpoint.position - getPositionRad()) * kPData.get();
        // if (setpoint.velocity == 0) {
        //     // have the kS help the PID when stationary
        //     volts += Math.signum((setpoint.position - getPositionRad()) * kPData.get()) * kSData.get() * 0.5;
        // }
        // setVoltage(volts);

    }

    public void toggleKill() {
        isKilled = !isKilled;
    }

    double lengthCFSToAxleX = Units.inchesToMeters(14);
    double lengthCFSToAxleY = Units.inchesToMeters(7);
    double lengthAxleToCFSAttatched = Units.inchesToMeters(15);

    double lengthCFSToAxle = Math.hypot(lengthCFSToAxleX, lengthCFSToAxleY);
    double axleToCFSTheta = Math.atan(lengthCFSToAxleY / lengthCFSToAxleX);


    public double calculateFF(double currentPositionRad, double setpointVelocityRadPerSec,
            double setpointAccelerationRadPerSecSquared) {
        // mathing the constant force spring angle for a seperate kG since its a
        // changing vector angle of constant magnitude

        double lengthCFS = Math.sqrt(Math.pow(lengthAxleToCFSAttatched, 2) + Math.pow(lengthCFSToAxle, 2)
                - 2 * lengthAxleToCFSAttatched * lengthCFSToAxle * Math.cos(currentPositionRad + axleToCFSTheta));

        double angleCFS = Math.acos((Math.pow(lengthAxleToCFSAttatched, 2) + Math.pow(lengthCFSToAxle, 2)
                - Math.pow(lengthAxleToCFSAttatched, 2)) / (2 * lengthCFS * lengthCFSToAxle));

        SmartDashboard.putNumber("angle cfs", angleCFS);

        double armFF = feedforward.calculate(currentPositionRad, setpointVelocityRadPerSec, accelerationSetpoint);

        double CFSFF;
        if (deployedMode) {
            CFSFF = Math.cos(angleCFS) * ArmConstants.deployedkG;
        } else {
            CFSFF = Math.cos(angleCFS) * ArmConstants.stowedkG;
        }
        return armFF + CFSFF;
    }

    public double calculatePID(double currentPositionRad) {
        feedback.setConstraints(ArmConstants.deployedConstraints);
        return feedback.calculate(currentPositionRad);
    }

    private boolean atGoal() {
        return (Math.abs(data.positionRad - getGoal()) < 0.1);
    }

    public void updateState() {
        if (!atGoal() || Math.abs(getVelocityRadPerSec()) > 0.1) {
            state = ArmStates.IN_TRANIST;
            return;
        }
        if (getGoal() == ArmConstants.stowPositionRad) {
            state = ArmStates.STOW;
            return;
        }
        if (getGoal() == ArmConstants.climbPositionRad) {
            state = ArmStates.CLIMB;
            return;

        }
        if (getGoal() == ArmConstants.ampPositionRad) {
            state = ArmStates.AMP;
            return;
        }
        if (getGoal() == ArmConstants.subwooferPositionRad) {
            state = ArmStates.SUBWOOFER;
            return;
        }
        // if (Robot.state == SuperStructureStates.SHOOT) {
        //     state = ArmStates.SHOOT;
        //     return;
        // }

        if (getGoal() == ArmConstants.groundIntakepositionRad){
            state = ArmStates.GROUND_INTAKE;
        }
    }

    // runs every 0.02 sec
    @Override
    public void periodic() {
        // System.out.println("0,0,0");
        armIO.updateData(data);
        updateState();
        stateLog.set(state.name());
        moveToGoal();

        if (Robot.wrist.getState() == WristStates.IN_TRANIST) {
            if (Robot.wrist.getWristGoal().position == WristConstants.stowGoalRad) {

                setDeployedMode(false);
                deployedModeLog.set(false);

            } else {
                setDeployedMode(true);
                deployedModeLog.set(true);
            }
        }

        positionLog.set(Units.radiansToDegrees(getPositionRad()));
        velocityLog.set(Units.radiansToDegrees(data.velocityRadPerSec));
        accelerationLog.set(Units.radiansToDegrees(data.accelerationRadPerSecSquared));
        voltageLog.set(data.appliedVolts);
        leftCurrentLog.set(data.leftCurrentAmps);
        rightCurrentLog.set(data.rightCurrentAmps);

        goalLog.set(Units.radiansToDegrees(feedback.getGoal().position));
        setpointPositionLog.set(Units.radiansToDegrees(feedback.getSetpoint().position));
        setpointVelocityLog.set(Units.radiansToDegrees(feedback.getSetpoint().velocity));
        setpointAccelerationLog.set(Units.radiansToDegrees(accelerationSetpoint));

        errorPositionLog
                .set(Units.radiansToDegrees(feedback.getSetpoint().position - data.positionRad));
        errorVelocityLog.set(
                Units.radiansToDegrees(feedback.getSetpoint().velocity - data.velocityRadPerSec));
        errorAccelerationLog.set(Units.radiansToDegrees(accelerationSetpoint - data.accelerationRadPerSecSquared));

        // mechanismArm.setAngle();
        // SmartDashboard.putData("mech", mechanism);

        boolean driverStationStatus = DriverStation.isEnabled();
        if (driverStationStatus && !isEnabled) {
            isEnabled = driverStationStatus;
            armIO.setBreakMode();
        }
        if (!driverStationStatus && isEnabled) {
            armIO.setCoastMode();
            isEnabled = driverStationStatus;
        }

    }

}