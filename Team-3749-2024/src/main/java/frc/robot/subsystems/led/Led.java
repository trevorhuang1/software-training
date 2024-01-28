package frc.robot.subsystems.led;

import java.sql.Driver;
import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.LEDConstants.LEDPattern;

public class Led extends SubsystemBase {

    private AddressableLED LEDs = new AddressableLED(0); //port
    private AddressableLEDBuffer LEDBuffer = new AddressableLEDBuffer(Constants.LEDConstants.length);
    private LEDPattern currentPattern = teamColorLED();
    private int hue = 0;

    public Led()
    {
       LEDs.setLength(LEDBuffer.getLength());
       LEDs.setData(LEDBuffer);
       LEDs.start();
    }

    private LEDPattern teamColorLED()
    {
       Optional<Alliance> team = DriverStation.getAlliance(); //i hate doing it this way but it throws an error without it
       return team.get() == Alliance.Blue ? LEDPattern.BLUE : LEDPattern.RED;
    }

    private void setLEDOneColorRGB(int R, int G, int B)
    {
        for(int i=0;i<LEDBuffer.getLength();i++)
        {
            LEDBuffer.setRGB(i, R, G, B);
        }
    }

    private void setLEDOneColorHSV(int H, int S, int V)
    {
        for(int i=0;i<LEDBuffer.getLength();i++)
        {
            LEDBuffer.setRGB(i, H, S, V);
        }
    }

    private void setLEDRainbow() //requires a loop
    {
        hue++;
        setLEDOneColorHSV(hue, 255, 255);
        if(hue >= 350)
        {
            hue = 0;
        }
    }

    // runs every 0.02 sec
    @Override
    public void periodic()
    {
        switch(this.currentPattern)
        {
            case RED:
                setLEDOneColorRGB(255,0,0);
            break;

            case BLUE:
                setLEDOneColorRGB(0,0,255);
            break;

            case GREEN:
                setLEDOneColorRGB(0, 255, 0);
            break;

            case RAINBOW:
                setLEDRainbow();
            break;

            default:
                setLEDOneColorRGB(0, 0, 0);
            break;
        }
        LEDs.setData(LEDBuffer);
    }
    
}
