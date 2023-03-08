package frc.robot.lights.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.lights.Color;
import frc.robot.lights.LEDLights;

public class DisplayColor extends InstantCommand {
    public DisplayColor(LEDLights lights, Color color) {
        super(() -> { lights.set(color); } );
    }
}
