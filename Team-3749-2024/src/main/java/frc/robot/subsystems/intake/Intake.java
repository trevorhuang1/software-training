package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.intake.IntakeIO.IntakeData;
import frc.robot.utils.Constants;

public class Intake extends SubsystemBase {

    private IntakeIO intakeIO;
    private IntakeData data = new IntakeData();
    private PIDController intakeController = new PIDController(Constants.ShintakeConstants.intakePID.kP,Constants.ShintakeConstants.intakePID.kI,Constants.ShintakeConstants.intakePID.kD);
        
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
        SmartDashboard.putNumber("intakeTemp", data.intakeTempCelcius);
        SmartDashboard.putBoolean("intakeHasPiece", data.sensorTripped);
    }

}
