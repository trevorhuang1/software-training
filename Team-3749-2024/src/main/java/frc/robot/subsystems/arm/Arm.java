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
import frc.robot.subsystems.arm.ArmIO.ArmData;
import frc.robot.utils.Constants.ArmConstants;
import frc.robot.utils.ShuffleData;

public class Arm extends SubsystemBase {

    private ArmData data = new ArmData();
    private ArmIO armIO;

    private ProfiledPIDController profiledFeedbackController = new ProfiledPIDController(ArmConstants.PID.kP,
            ArmConstants.PID.kI,
            ArmConstants.PID.kD,
            ArmConstants.constraints);

    private ArmFeedforward feedForwardController = new ArmFeedforward(ArmConstants.kS,
            ArmConstants.kG,
            ArmConstants.kV);



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
    private ShuffleData<Double> setpointAccelerationLog = new ShuffleData<Double>(this.getName(), "setpoint acceleration", 0.0);
    private ShuffleData<Double> errorPositionLog = new ShuffleData<Double>(this.getName(), "error position",
            0.0);
    private ShuffleData<Double> errorVelocityLog = new ShuffleData<Double>(this.getName(), "error velocity",
            0.0);
    private ShuffleData<Double> errorAccelerationLog = new ShuffleData<Double>(this.getName(), "error acceleration", 0.0);

    private double accelerationSetpoint = 0;
    private double prevSetpointVelocity = 0;

    private boolean isKilled = false;
    private boolean isEnabled = false;

    public Arm() {
        if (Robot.isSimulation()) {
            armIO = new ArmSim();
        } else {
            armIO = new ArmSparkMax();
        }
    }

    public double getPositionRad() {
        return data.positionRad;
    }

    public double getVelocityRadPerSec(){
        return data.velocityRadPerSec;
    }

    public void setGoal(double positionRad) {
        profiledFeedbackController.setGoal(positionRad);
        SmartDashboard.putNumber("DON POSE REAL", positionRad);
    }

    public double getGoal() {
        return profiledFeedbackController.getGoal().position;
    }

    public State getSetpoint() {
        return profiledFeedbackController.getSetpoint();
    }

    public void setVoltage(double volts) {
        if (isKilled) {
            armIO.setVoltage(0);
        } else {
            armIO.setVoltage(volts);
        }
    }

    public void moveToGoal(){
        State setpoint = getSetpoint();
        double accelerationSetpoint = (setpoint.velocity - prevSetpointVelocity) / 0.02;
        prevSetpointVelocity = setpoint.velocity;
        double feedback = calculatePID(getPositionRad());

        // if resting on the hard stop, don't waste voltage on kG
        if (setpoint.position == 0 && Units.radiansToDegrees(getPositionRad()) < 2) {
            setVoltage(0);

            return;
        }
        // if 4bar is deployed, switch kG
        if (setpoint.velocity == 0 && Robot.wrist.getIsDeployed()) {
            // ks, kg, and P
            double error = (setpoint.position - getPositionRad());
            double voltage = Math.signum(error) * ArmConstants.kS
                    + ArmConstants.deployedKG * Math.cos(getPositionRad())
                    + ArmConstants.deployedKP * error;
            setVoltage(voltage);
            return;
        }

        double feedforward;
        if (setpoint.velocity != 0) {
            feedforward = calculateFF(getPositionRad(),setpoint.velocity, accelerationSetpoint);
        } else {
            // have the kS help the PID when stationary
            feedforward = Math.signum(feedback) * ArmConstants.kS;
        }
        setVoltage(feedforward + feedback);
    }

    public void toggleKill() {
        isKilled = !isKilled;
    }

    public double calculateFF(double currentPositionRad, double setpointVelocityRadPerSec, double setpointAccelerationRadPerSecSquared){
        return feedForwardController.calculate(currentPositionRad, setpointVelocityRadPerSec, accelerationSetpoint);

    }

    public double calculatePID(double currentPositionRad){
        return profiledFeedbackController.calculate(currentPositionRad);
    }


    // runs every 0.02 sec
    @Override
    public void periodic() {
        // System.out.println("0,0,0");
        armIO.updateData(data);
        moveToGoal();

        // positionLog.set(Units.radiansToDegrees(getPositionRad()));
        // velocityLog.set(data.velocityRadPerSec);
        // accelerationLog.set(data.accelerationRadPerSecSquared);
        // voltageLog.set(data.appliedVolts);
        // leftCurrentLog.set(data.leftCurrentAmps);
        // rightCurrentLog.set(data.rightCurrentAmps);
        
        // goalLog.set(profiledFeedbackController.getGoal().position);
        // setpointPositionLog.set(profiledFeedbackController.getSetpoint().position);
        // setpointVelocityLog.set(profiledFeedbackController.getSetpoint().velocity);
        // setpointAccelerationLog.set(accelerationSetpoint);

        // errorPositionLog.set(profiledFeedbackController.getSetpoint().position - data.positionRad);
        // errorVelocityLog.set(profiledFeedbackController.getSetpoint().velocity - data.velocityRadPerSec);
        // errorAccelerationLog.set(accelerationSetpoint - data.accelerationRadPerSecSquared);

        // mechanismArm.setAngle(getRotation2d());
        // SmartDashboard.putData("mech", mechanism);

        // SmartDashboard.putNumber("Arm Currentleft", data.leftCurrentAmps);
        // SmartDashboard.putNumber("Arm Currentright", data.rightCurrentAmps);
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