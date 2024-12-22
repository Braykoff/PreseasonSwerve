package frc.robot.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.PregameCommand;
import frc.robot.subsystems.swerve.SwerveDrive;

public class Dashboard {
    public Dashboard(SwerveDrive swerve) {
        // Drive Tab
        ShuffleboardTab drive = Shuffleboard.getTab("Drive");

        // Pregame command button
        drive.add(new PregameCommand(swerve))
            .withPosition(0, 0)
            .withSize(1, 1);
    }
}
