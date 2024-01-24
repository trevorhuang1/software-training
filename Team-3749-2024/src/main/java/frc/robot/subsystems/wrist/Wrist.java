package frc.robot.subsystems.wrist;

import java.util.HashMap;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.wrist.WristIO.WristData;
import frc.robot.utils.Constants;

public class Wrist extends SubsystemBase {

    private WristIO wristModule;
    private WristData data = new WristData();
    private ProfiledPIDController wristController = new ProfiledPIDController(Constants.WristConstants.PID.kP,
            Constants.WristConstants.PID.kI, Constants.WristConstants.PID.kD,Constants.WristConstants.trapezoidConstraint);
    private SimpleMotorFeedforward wristFF = new SimpleMotorFeedforward(1, 0);
    private HashMap<Boolean, Double> setpointToggle = new HashMap<Boolean,Double>();
    private boolean isGroundIntake = false;

    public Wrist() 
        {
        setpointToggle.put(true,Constants.WristConstants.groundGoal);
        setpointToggle.put(false,Constants.WristConstants.stowGoal);
        wristModule = new WristSparkMax();
         if(Robot.isSimulation()) 
         {
            wristModule = new WristSim();
         }
    }
    
    public void toggleWristGoal() 
    {
        this.isGroundIntake = !this.isGroundIntake;
        wristController.setGoal(setpointToggle.get(this.isGroundIntake));
    }

    public State getWristGoal()
    {
        return wristController.getGoal();
    }

    public State getWristSetpoint()
    {
        return wristController.getSetpoint();
    }

    public void moveWristToAngle() {
        
        wristModule.setVoltage(
            wristController.calculate(wristModule.getEncoderValue()) + //is getting the goal redundant?
                wristFF.calculate(wristController.getSetpoint().velocity)
        );
        System.out.println(wristController.getGoal().position);
    }

    @Override
    public void periodic() {
        wristModule.updateData(data);
    }

}
