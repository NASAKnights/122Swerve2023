package frc.robot.lights.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.lights.LEDLights;

public class LightsOff extends InstantCommand {
    public LightsOff(LEDLights lights) {
        super(lights::turnOff);
    }
}
