package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * Runs all actions that should be run after the robot successfully boots/initializes/connects,
 * but BEFORE the match actually starts
 */
public class PregameCommand extends InstantCommand {
    public PregameCommand(SwerveDrive drive) {
        super(() -> {
            drive.homeEncoders();
            drive.cacheAllianceColor();
            System.out.println("Pregame complete");
        });
    }

    @Override
    public String getName() { return "PREGAME"; }

    @Override
    public boolean runsWhenDisabled() { return true; }
}
