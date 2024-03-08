package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class JTVisiSight implements PhotoelectricIO{

    private DigitalInput photoelectricSensor = new DigitalInput(5);

    public JTVisiSight(){

    }

    @Override
    public void updateData(PhotoelectricData data){
        data.sensing = photoelectricSensor.get();
        // photoelectricSensor.
    }
    
    
}
