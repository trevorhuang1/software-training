package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utils.Constants.Sim;

public class Wrist extends SubsystemBase {

    public Wrist() 
    {
        if(Robot.isReal())
        {

        }
    }

    public void setWristAngle(double angle)
    {
     this.targetAngle = angle;
    }
    
   

}
