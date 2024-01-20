package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;


/*
 * note from jonathan:
 * always start the robot with the robot stowed!!!!
 * 
 * if you do not i will be very sad (and it also might destory itself)
 */

public class WristSparkMax extends SubsystemBase {

    private CANSparkMax wristMotor = new CANSparkMax(3,MotorType.kBrushless);
    private PIDController wristController = new PIDController(1, 0, 0);
    private RelativeEncoder wristEncoder = wristMotor.getEncoder();
    private SimpleMotorFeedforward wristFF = new SimpleMotorFeedforward(1, 0);
    private double targetAngle = Constants.WristConstants.stowSetpoint;

    public WristSparkMax() 
    {
        wristEncoder.setPosition(Constants.WristConstants.wristOffset); 
    }

   public void setWristAngle(double angle)
   {
    this.targetAngle = angle;
   }

   public void moveWristAngle()
   {
    wristMotor.setVoltage(
        wristController.calculate(wristEncoder.getPosition(),targetAngle) + 
        wristFF.calculate(targetAngle)
    );
   }

   @Override
   public void periodic()
   {
    moveWristAngle();
    SmartDashboard.putNumber("WristVolts", wristMotor.getBusVoltage());
    SmartDashboard.putNumber("WristAngle", wristEncoder.getPosition());
   }

}
