package frc.robot.subsystems.intake;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.subsystems.intake.IntakeIO.IntakeData;
import frc.robot.utils.Constants;

public class Intake extends SubsystemBase {

    private IntakeIO intakeIO;
    private IntakeData data = new IntakeData();
    private PIDController intakeController = new PIDController(Constants.IntakeConstants.intakePID.kP,Constants.IntakeConstants.intakePID.kI,Constants.IntakeConstants.intakePID.kD);
        
    private SimpleMotorFeedforward intakeFF = new SimpleMotorFeedforward(0, 1);
    private double intakeVelocity = 0;

    public Intake() 
        {
        intakeIO = new IntakeSparkMax();
         if(Robot.isSimulation()) 
         {
            intakeIO = new IntakeSim();
         }
    }

    public void setIntakeVelocity(double velocity)
   {
    this.intakeVelocity = velocity;
   }

   public void moveIntake()
   {
    double voltage = intakeController.calculate(data.intakeVelocityRadPerSec,intakeVelocity) + intakeFF.calculate(intakeVelocity);
    setVoltage(voltage);
   }
   public void setVoltage(double volts){
    intakeIO.setVoltage(volts);
   }

    @Override
    public void periodic() {
        intakeIO.updateData(data);

        SmartDashboard.putNumber("intakeVolts",data.intakeVolts);
        SmartDashboard.putNumber("intakeVelocityRadPerSec", data.intakeVelocityRadPerSec);
        SmartDashboard.putNumber("intakeTempCelsius", data.intakeTempCelcius);
        SmartDashboard.putBoolean("intakeHasPiece", data.sensorTripped);

        intakeAbsPos += data.intakeVelocityRadPerSec * 0.02;
    }

    // initiate changable measures for voltage, angle, and velocity
    private final MutableMeasure<Voltage> voltageMeasure = mutable(Volts.of(0));
    private final MutableMeasure<Angle> angleMeasure = mutable(Radians.of(0));
    private final MutableMeasure<Velocity<Angle>> velocityMeasure = mutable(RadiansPerSecond.of(0)); 

    private static double intakeAbsPos = 0;

    SysIdRoutine intakeRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.per(Seconds).of(1),
            Volts.of(12),
            Seconds.of(10)
          ),
        new SysIdRoutine.Mechanism( // takes in voltage consumer, log consumer, and name of mechanism

            (Measure<Voltage> volts) -> {
                setVoltage(volts.magnitude());},

            (log) -> {
                log
                .motor("intake-motor")
                .voltage(
                    voltageMeasure.mut_replace( 
                        data.intakeVolts, // update mutable "voltage" with data
                        Volts // the unit is volts
                    )
                ).angularPosition(
                    angleMeasure.mut_replace(
                        intakeAbsPos, // update mutable "angle" with data
                        Radians // the unit is radians
                    )
                ).angularVelocity(
                    velocityMeasure.mut_replace(
                        data.intakeVelocityRadPerSec, // update mutable "angle" with data
                        RadiansPerSecond // the unit is radians per second
                    )
                );
            },

            this
        )
    );
    
    public Command getIntakeSysIDQuasistaticForwardTest() {
        return intakeRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command getIntakeSysIDQuasistaticReverseTest() {
        return intakeRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command getIntakeSysIDDynamicForwardTest() {
        return intakeRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command getIntakeSysIDDynamicReverseTest() {
        return intakeRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }
}
