package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.HardwareConstants;

public class SwerveDrive implements Subsystem {
  // Swerve Modules
  private final SwerveModule[] modules = {
    new SwerveModule(
        "FL", 
        HardwareConstants.kFL_DRIVE_KRAKEN_CAN, HardwareConstants.kFL_STEER_NEO_CAN, 
        HardwareConstants.kFL_STEER_CANCODER_CAN, SwerveConstants.kFL_CANCODER_OFFSET
    ), new SwerveModule(
        "FR", 
        HardwareConstants.kFR_DRIVE_KRAKEN_CAN, HardwareConstants.kFR_STEER_NEO_CAN, 
        HardwareConstants.kFR_STEER_CANCODER_CAN, SwerveConstants.kFR_CANCODER_OFFSET
    ), new SwerveModule(
        "RL", 
        HardwareConstants.kRL_DRIVE_KRAKEN_CAN, HardwareConstants.kRL_STEER_NEO_CAN, 
        HardwareConstants.kRL_STEER_CANCODER_CAN, SwerveConstants.kRL_CANCODER_OFFSET
    ), new SwerveModule(
        "FL", 
        HardwareConstants.kRR_DRIVE_KRAKEN_CAN, HardwareConstants.kRR_STEER_NEO_CAN, 
        HardwareConstants.kRR_STEER_CANCODER_CAN, SwerveConstants.kRR_CANCODER_OFFSET
    )
  };

  // Kinematics
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveConstants.kPOSITIONS);

  // Pose estimation
  
}
