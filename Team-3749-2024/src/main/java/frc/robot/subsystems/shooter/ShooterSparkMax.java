package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSparkMax extends SubsystemBase {

    private CANSparkMax shooterMotorLeft = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax shooterMotorRight = new CANSparkMax(1,MotorType.kBrushless);
    private CANSparkMax wristMotor = new CANSparkMax(1,MotorType.kBrushless);
    private PIDController wristController = new PIDController(1, 0, 0);
    private RelativeEncoder wristEncoder = wristMotor.getEncoder();
    private ArmFeedforward wristFF = new ArmFeedforward(0, 0,0);
    private double shooterVoltage = 0;
    private double wristVoltage = 0;

    public ShooterSparkMax() {
        System.out.println("[Init] Creating ExampleIOSim");
        shooterMotorRight.setInverted(true);
    }
    
   public void setShooterVolts(double volts)
   {
    this.shooterVoltage = volts;
   }

   public void stop()
   {
    shooterMotorLeft.stopMotor();
    shooterMotorRight.stopMotor();
   }

   public void setWristSetpoint(double setpoint)
   {
    wristVoltage = setpoint;
   }

   @Override
   public void periodic()
   {
    shooterMotorLeft.setVoltage(shooterVoltage); //does motor controller group work for this?
    shooterMotorRight.setVoltage(shooterVoltage);
    wristMotor.setVoltage(wristFF.calculate(wristVoltage,1) + wristController.calculate(wristEncoder.getPosition(),wristVoltage));
    SmartDashboard.putNumber("shooterVolts",shooterMotorLeft.getBusVoltage()); //could document the other one but its probably fine
    SmartDashboard.putNumber("shooterWristVolts", wristMotor.getBusVoltage());
   }

}
