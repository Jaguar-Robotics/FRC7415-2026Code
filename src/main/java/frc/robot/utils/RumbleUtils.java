package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleUtils {

    /**
     * Creates a command that rumbles a controller for a set duration.
     *
     * @param controller The controller to rumble
     * @param strength   Rumble strength from 0.0 to 1.0
     * @param seconds    How long to rumble in seconds
     * @return Command that can be scheduled
     */
    public static Command rumble(CommandXboxController controller, double strength, double seconds) {
        return Commands.startEnd(
            () -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, strength),
            () -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
        ).withTimeout(seconds);
    }
}