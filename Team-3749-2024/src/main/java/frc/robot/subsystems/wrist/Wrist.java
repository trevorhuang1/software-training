package frc.robot.subsystems.wrist;

import java.util.HashMap;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.wrist.WristIO.WristData;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.UtilityFunctions;

public class Wrist extends SubsystemBase {
    // hello test
    private WristIO wristIO;
    private WristData data = new WristData();

    private ProfiledPIDController wristController = new ProfiledPIDController(WristConstants.PID.kP,
            WristConstants.PID.kI, WristConstants.PID.kD,
            WristConstants.trapezoidConstraint);

    private ArmFeedforward wristFF = new ArmFeedforward(WristConstants.simkS, WristConstants.simkG,
            WristConstants.simkV);

    private HashMap<Boolean, Double> setpointToggle = new HashMap<Boolean, Double>();

    private boolean isDeployed = false;

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
    private ShuffleData<Double> currentLog = new ShuffleData<Double>(this.getName(), "current",
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

    public Wrist() {
        setpointToggle.put(true, WristConstants.groundGoalRad);
        setpointToggle.put(false, WristConstants.stowGoalRad);
        wristIO = new WristSparkMax();
        if (Robot.isSimulation()) {
            wristIO = new WristSim();
        }
    }

    // runs twice???
    public void toggleWristGoal() {
        this.isDeployed = !this.isDeployed;
        wristController.setGoal(setpointToggle.get(this.isDeployed));
        System.out.println(isDeployed);
    }

    public void setGoalGround() {
        System.out.println("ground");
        wristController.setGoal(setpointToggle.get(true));
        isDeployed = true;
    }

    public void setGoalStow() {
        System.out.println("stow");
        isDeployed = false;
        wristController.setGoal(setpointToggle.get(false));

    }

    public State getWristGoal() {
        return wristController.getGoal();
    }

    public State getWristSetpoint() {
        return wristController.getSetpoint();
    }

    public boolean getIsDeployed() {
        return isDeployed;
    }

    public double getPositionRad() {
        return (data.positionRad);
    }

    public double getVelocityRadPerSec() {
        return (data.velocityRadPerSec);
    }

    private ShuffleData<Double> kPData = new ShuffleData(this.getName(), "kpdata", 0.0);
    private ShuffleData<Double> kVData = new ShuffleData(this.getName(), "kVdata", 0.0);

    public void moveWristToAngle() {
        double pidGain = wristController.calculate(data.positionRad);

        State setpoint = Robot.wrist.getWristSetpoint();
        double velocityRadPerSec = setpoint.velocity;
        double positionRad = setpoint.position;

        double voltage = UtilityFunctions.withinMargin(0.35, getWristGoal().position, data.positionRad)
                ? pidGain
                : 0;

        // double voltage = 0;

        // voltage += kPData.get() * (positionRad - data.positionRad);
        // if getting stowed, close to the setpoint, and not moving, then greatly reduce voltage 
        if ((positionRad == WristConstants.stowGoalRad
                && UtilityFunctions.withinMargin(0.15, positionRad, data.positionRad))
                && Math.abs(data.velocityRadPerSec) < 0.05) {
            setVoltage(-Math.signum(pidGain) * 0.05);
            return;
        }

        if (Robot.isSimulation()) {

            voltage += wristFF.calculate(data.positionRad, velocityRadPerSec); // is getting the goal redundant?
        } else {
            voltage += Math.signum(pidGain) * WristConstants.realkS;
            voltage += getWristGoal().position == WristConstants.groundGoalRad
                    // ? velocityRadPerSec * kVData.get()
                    // : velocityRadPerSec * kVData.get();
                    ? velocityRadPerSec * WristConstants.realkVForward
                    : velocityRadPerSec * WristConstants.realkVBackward;
            voltage += calculateGravityFeedForward(data.positionRad, Robot.arm.getPositionRad());
        }

        setVoltage(voltage);
        // System.out.println(wristController.getGoal().position);
    }

    public void setVoltage(double volts) {
        wristIO.setVoltage(volts);
    }

    // private ShuffleData<Double> kGData = new ShuffleData<Double>("wrist",
    // "kGData", 0.0);

    public void runFF(double add) {

        wristIO.setVoltage(calculateGravityFeedForward(data.positionRad, Robot.arm.getPositionRad()) + add);
    }

    public double calculateGravityFeedForward(double wristPositionRad, double armPositionRad) {
        // the way data was collected has it on the front end of the FF rather than the
        // middle. the 0.6 helps to alliviate this at the cost of working well at
        // variable arm angle. Yes, that kinda defeats some of the purpose of the
        // regression, but due to other decisions, we will not really be moving the 4bar
        // at variable angles anyway. The thing can be retuned and made cool again later
        return 0.6 * (WristConstants.kYIntercept
                + WristConstants.kBar * wristPositionRad
                + WristConstants.kBarSquared * Math.pow(wristPositionRad, 2)
                + WristConstants.kBarCubed * Math.pow(wristPositionRad, 3)
                + WristConstants.kArm * armPositionRad
                + WristConstants.kArmSquared * Math.pow(armPositionRad, 2)
                + WristConstants.kBarArm * wristPositionRad * armPositionRad
                + WristConstants.kBarSquaredArm * Math.pow(wristPositionRad, 2) * armPositionRad
                + WristConstants.kBarCubedArm * Math.pow(wristPositionRad, 3) * armPositionRad
                + WristConstants.kBarArmSquared * wristPositionRad * Math.pow(armPositionRad, 2)
                + WristConstants.kBarSquaredArmSquared * Math.pow(wristPositionRad, 2) * Math.pow(armPositionRad, 2)
                + WristConstants.kBarCubedArmSquared * Math.pow(wristPositionRad, 3) * Math.pow(armPositionRad, 2));
    }

    @Override
    public void periodic() {
        wristIO.updateData(data);
        moveWristToAngle();

        // mechanismArm.setAngle(data.positionRad);
        // SmartDashboard.putData("Mech2d", mechanism);
        // mechanismArm.setAngle(Math.toDegrees(data.positionRad));

        positionLog.set(Units.radiansToDegrees(data.positionRad));
        velocityLog.set(Units.radiansToDegrees(data.velocityRadPerSec));
        accelerationLog.set(Units.radiansToDegrees(data.accelerationRadPerSecSquared));
        goalLog.set(Units.radiansToDegrees(getWristGoal().position));
        setpointPositionLog.set(Units.radiansToDegrees(getWristSetpoint().position));
        setpointVelocityLog.set(Units.radiansToDegrees(getWristSetpoint().velocity));
        voltageLog.set(data.appliedVolts);
        currentLog.set(data.currentAmps);
        errorPositionLog.set(Units.radiansToDegrees(getWristSetpoint().position - data.positionRad));
        errorVelocityLog.set(Units.radiansToDegrees(getWristSetpoint().velocity - data.velocityRadPerSec));

        SmartDashboard.putNumber("FF",
                calculateGravityFeedForward(data.positionRad, Robot.arm.getPositionRad()));

        // test
    }

}
