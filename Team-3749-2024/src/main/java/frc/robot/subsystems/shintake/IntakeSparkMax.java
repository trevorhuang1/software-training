package frc.robot.subsystems.shintake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class IntakeSparkMax extends SubsystemBase {

    private CANSparkMax intakeMotor = new CANSparkMax(0, MotorType.kBrushless);
    private RelativeEncoder intakEncoder = intakeMotor.getEncoder();
    private PIDController intakeController = new PIDController(1, 0, 0);
    private SimpleMotorFeedforward intakeFF = new SimpleMotorFeedforward(1, 0);
    private double intakeVelocity = Constants.ShintakeConstants.idleVoltage;

    public IntakeSparkMax() 
    {

    }
    
   public void setIntakeVolts(double velocity)
   {
    this.intakeVelocity = velocity;
   }

   public void stop()
   {
    intakeMotor.stopMotor();
   }

   @Override
   public void periodic()
   {
    intakeMotor.setVoltage(
        intakeFF.calculate(intakeVelocity) + 
        intakeController.calculate(intakEncoder.getVelocity(),intakeVelocity)
    );
    SmartDashboard.putNumber("intakeVolts",intakeMotor.getBusVoltage());
   }

}
