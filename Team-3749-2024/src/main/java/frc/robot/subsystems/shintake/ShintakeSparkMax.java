package frc.robot.subsystems.shintake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class ShintakeSparkMax extends SubsystemBase {

    private CANSparkMax intakeMotor = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax leftShooter = new CANSparkMax(1,MotorType.kBrushless);
    private CANSparkMax rightShooter = new CANSparkMax(2,MotorType.kBrushless);


    private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
    private RelativeEncoder leftEncoder = leftShooter.getEncoder();
    private RelativeEncoder rightEncoder = rightShooter.getEncoder();

    private PIDController intakeController = new PIDController(1, 0, 0);
    private PIDController shooterController = new PIDController(1, 0, 0);

    private SimpleMotorFeedforward intakeFF = new SimpleMotorFeedforward(1, 0);
    private SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(1, 0);   

    private double intakeVelocity = Constants.ShintakeConstants.intakeIdleVoltage;
    private double shooterVelocity = Constants.ShintakeConstants.shooterIdleVoltage;

    public ShintakeSparkMax() 
    {
        rightShooter.setInverted(true);
    }
    
   public void setIntakeVelocity(double velocity)
   {
    this.intakeVelocity = velocity;
   }

   public void setShooterVelocity(double velocity)
   {
    this.shooterVelocity = velocity;
   }

   public void stopIntake()
   {
    intakeMotor.stopMotor();
   }

   public void stopShooter()
   {
    leftShooter.stopMotor();
    rightShooter.stopMotor();
   }

   @Override
   public void periodic()
   {
    intakeMotor.setVoltage(
        intakeFF.calculate(intakeVelocity) + 
        intakeController.calculate(intakeEncoder.getVelocity(),intakeVelocity)
    );

    leftShooter.setVoltage(
        shooterFF.calculate(shooterVelocity) +
        shooterController.calculate(leftEncoder.getVelocity(),shooterVelocity)
    );

    rightShooter.setVoltage(
        shooterFF.calculate(shooterVelocity) + 
        shooterController.calculate(rightEncoder.getVelocity(),shooterVelocity)
    );

    SmartDashboard.putNumber("intakeVolts",intakeMotor.getBusVoltage());
    SmartDashboard.putNumber("shooterVolts", leftShooter.getBusVoltage());
   }

}
