package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Led extends SubsystemBase {

    private AddressableLED LEDs = new AddressableLED(0); //port
    private AddressableLEDBuffer LEDBuffer = new AddressableLEDBuffer(Constants.LEDConstants.length);


    public Led()
    {
       LEDs.setLength(LEDBuffer.getLength());
    }

    public void setLEDOneColorRGB(int R, int G, int B)
    {
        for(int i=0;i<LEDBuffer.getLength();i++)
        {
            LEDBuffer.setRGB(i, R, G, B);
        }
    }

    public void setLEDOneColorHSV(int H, int S, int V)
    {
        for(int i=0;i<LEDBuffer.getLength();i++)
        {
            LEDBuffer.setRGB(i, H, S, V);
        }
    }

    // runs every 0.02 sec
    @Override
    public void periodic()
    {

    }
    
}
