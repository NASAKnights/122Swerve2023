package frc.robot.lights;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;

public class LEDLights {
    private Spark blinkin;
    private Color lastColor;

    public LEDLights() {
        blinkin = new Spark(Constants.LEDConstants.kBlinkinID);
    }

    public void set(Color color) {
        this.lastColor = color;
        blinkin.set(color.power);
    }

    public void turnOff() {
        this.lastColor = null;
        blinkin.set(0);
    }

    public boolean isOn() {
        return blinkin.get() != 0;
    }

    public Color getLastColor() {
        return this.lastColor;
    }
}
