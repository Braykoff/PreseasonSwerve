package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Contains Alerts that may be triggered from anywhere in the robot program.
 */
public final class GlobalAlerts {
    public static final Alert SignalLoggerRunning = new Alert("CTRE SignalLogger is running", AlertType.kInfo);
}