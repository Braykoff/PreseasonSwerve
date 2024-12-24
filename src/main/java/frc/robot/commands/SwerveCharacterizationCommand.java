package frc.robot.commands;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.GlobalAlerts;

public class SwerveCharacterizationCommand extends SequentialCommandGroup {
    private final boolean quasistatic, direction;

    /**
     * @param drive
     * @param quasistatic true for quasistatic, false for dynamic
     * @param direction true for forward, false for backward
     */
    public SwerveCharacterizationCommand(SwerveDrive drive, boolean quasistatic, boolean direction) {
        super(
            // Init Signal Logger
            new InstantCommand(() -> {
                SignalLogger.start();
                GlobalAlerts.SignalLoggerRunning.set(true);
                System.out.println("Started CTRE SignalLogger");
            }),
            // Check if FMS is connected
            new ConditionalCommand(
                // FMS connected, do not characterize:
                new PrintCommand("Will not characterize while FMS is connected!"), 
                // No FMS connected, begin characterization:
                quasistatic ? 
                    drive.getRoutine().quasistatic(direction ? Direction.kForward : Direction.kReverse)
                    : drive.getRoutine().dynamic(direction ? Direction.kForward : Direction.kReverse), 
                DriverStation::isFMSAttached
            )
        );

        this.quasistatic = quasistatic;
        this.direction = direction;
    }

    @Override
    public String getName() {
        return String.format("SwerveSysID(quasistatic: %b, direction: %b)", quasistatic, direction);
    }
}
