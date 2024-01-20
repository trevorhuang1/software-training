package frc.robot.subsystems.wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.wrist.WristIO.WristData;
import frc.robot.utils.Constants;

public class Wrist extends SubsystemBase {

    private WristIO wristModule;
    private WristData data = new WristData();
    private PIDController wristController = new PIDController(Constants.WristConstants.PID.kP, Constants.WristConstants.PID.kI, Constants.WristConstants.PID.kD);
    private SimpleMotorFeedforward wristFF = new SimpleMotorFeedforward(1, 0);
    private double targetAngle = Constants.WristConstants.stowSetpoint;
    private double targetVolt = 0;

    public Wrist() 
    {
        if(Robot.isReal())
        {
            wristModule = new WristSparkMax();
        }
        else
        {
            wristModule = new WristSim();
        }
    }
    
    public void setWristAngle(double angle)
    {
        this.targetAngle = angle;
    }

    public void moveWristToAngle()
    {
        this.targetVolt = wristController.calculate(wristModule.getEncoderValue(),targetAngle) +
        wristFF.calculate(targetAngle);
    }

    @Override
    public void periodic() 
    {
        wristModule.setVoltage(targetVolt);
        wristModule.updateData(data); 
    }
   

}
