// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.Dashboard;

public class RobotContainer {
  // Subsystems
  private SwerveDrive base = new SwerveDrive();

  // Controllers
  private CommandXboxController controller = new CommandXboxController(0);
  private Dashboard dashboard = new Dashboard(base);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // SwerveDrive default command (teleop driving)
    base.setDefaultCommand(
      new TeleopDriveCommand(
        base, 
        controller::getLeftX, 
        controller::getLeftY, 
        controller::getRightX, 
        controller.leftTrigger())
    );
  }

  /** Logs everything, called periodically */
  public void checkHardware() {
    base.checkHardware();
  }

  /** Logs everything, called periodically */
  public void log() {
    base.log();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
