package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.intake.IntakeIO.IntakeData;
import frc.robot.utils.ShuffleData;

public class Intake extends SubsystemBase {

    private IntakeIO intakeIO;
    private IntakeData data = new IntakeData();

    private PIDController feedback = new PIDController(
            IntakeConstants.intakePID.kP,
            IntakeConstants.intakePID.kI,
            IntakeConstants.intakePID.kD);

    private SimpleMotorFeedforward intakeFF = new SimpleMotorFeedforward(
            0,
            IntakeConstants.kV,
            0);

    private ShuffleData<Double> IntakeVelocityLog = new ShuffleData<Double>(this.getName(), " intake velocity", 0.0);
    private ShuffleData<Double> IntakevoltageLog = new ShuffleData<Double>(this.getName(), " intake voltage", 0.0);
    private ShuffleData<Double> IntakecurrentLog = new ShuffleData<Double>(this.getName(), " intake current", 0.0);

    public Intake() {
        intakeIO = new IntakeSparkMax();
        if (Robot.isSimulation()) {
            intakeIO = new IntakeSim();
        }
    }

    ShuffleData<Double> kVData = new ShuffleData<Double>(this.getName(), "kVData", 0.0);
    ShuffleData<Double> kPData = new ShuffleData<Double>(this.getName(), "kPData", 0.0);
    ShuffleData<Double> velData = new ShuffleData<Double>(this.getName(), "velData", 0.0);

    public void setIntakeVelocity(double velocityRadPerSec) {

        double voltage = feedback.calculate(
                data.intakeVelocityRadPerSec,
                velocityRadPerSec) +
                    intakeFF.calculate(velocityRadPerSec);

        // voltage = velData.get() * kVData.get() + (velData.get() - data.intakeVelocityRadPerSec) * kPData.get();
        // System.out.println(voltage);
        // System.out.println(velData.get());

        setVoltage(voltage);
    }

    public void setVoltage(double volts) {
        intakeIO.setVoltage(volts);
    }

    @Override
    public void periodic() {

        intakeIO.updateData(data);

        // IntakeVelocityLog.set(data.intakeVelocityRadPerSec);

        // IntakevoltageLog.set(data.intakeVolts);

        // IntakecurrentLog.set(data.currentAmps);
    }

}
