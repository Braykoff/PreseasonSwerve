package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.BuildConstants;
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

  // NT Logging
  private final StructArrayPublisher<SwerveModuleState> statePublisher;
  private final StructPublisher<Pose2d> posePublisher;
  private final IntegerPublisher successfulDAQPublisher, failedDAQPublisher;

  // DL Logging
  private final StructArrayLogEntry<SwerveModuleState> stateLogger = 
    StructArrayLogEntry.create(DataLogManager.getLog(), "Swerve/State", SwerveModuleState.struct);
  private final StructLogEntry<Pose2d> poseLogger =
    StructLogEntry.create(DataLogManager.getLog(), "Swerve/Pose", Pose2d.struct);
  private final IntegerLogEntry successfulDAQLogger = 
    new IntegerLogEntry(DataLogManager.getLog(), "Swerve/Successful_DAQs");
  private final IntegerLogEntry failedDAQLogger =
    new IntegerLogEntry(DataLogManager.getLog(), "Swerve/Failed_DAQs");

  private final SwerveModuleState[] loggedStates = {
    modules[0].state, modules[1].state, modules[2].state, modules[3].state
  };

  // Pose estimation
  private SwerveOdometry odometry = new SwerveOdometry(modules, kinematics);

  public SwerveDrive() {
    // Init NT publishers if used
    if (BuildConstants.kPUBLISH_EVERYTHING) {
      NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Swerve");

      statePublisher = ntTable.getStructArrayTopic("State", SwerveModuleState.struct).publish();
      posePublisher = ntTable.getStructTopic("Pose", Pose2d.struct).publish();
      successfulDAQPublisher = ntTable.getIntegerTopic("Successful_DAQs").publish();
      failedDAQPublisher = ntTable.getIntegerTopic("Failed_DAQs").publish();
    }
  }

  /**
   * @param y Left is +, m/s
   * @param x Fwd is +, m/s
   * @param theta CCW is +, rad/sec
   * @param closedLoop
   * @param fieldOriented
   */
  public void drive(double y, double x, double theta, boolean closedLoop, boolean fieldOriented) {
    ChassisSpeeds speeds = fieldOriented ?
      // Field oriented driving
      ChassisSpeeds.fromRobotRelativeSpeeds(x, y, theta, odometry.getFieldRelativePosition().getRotation())
      // Robot oriented driving
      : new ChassisSpeeds(x, y, theta);

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.kMAX_WHEEL_VELOCITY);

    for (int m = 0; m < 4; m++) {
      modules[m].setRequest(states[m], closedLoop);
    }
  }

  /** Homes all swerve modules. Run in pregame. */
  public void homeEncoders() {
    for (int m = 0; m < 4; m++) {
      modules[m].homeEncoder();
    }
  }

  /** Caches alliance color for odometry. Run in pregame. */
  public void cacheAllianceColor() {
    odometry.cacheAllianceColor();
  }

  public void log() {
    // Update all states
    for (int m = 0; m < 4; m++) {
      modules[m].updateSwerveState();
    }

    Pose2d pose = odometry.getFieldRelativePosition();

    // Log
    stateLogger.append(loggedStates);
    poseLogger.append(pose);
    successfulDAQLogger.append(odometry.getSuccessfulDAQs());
    failedDAQLogger.append(odometry.getFailedDAQs());

    if (BuildConstants.kPUBLISH_EVERYTHING) {
      statePublisher.set(loggedStates);
      posePublisher.set(pose);
      successfulDAQPublisher.set(odometry.getSuccessfulDAQs());
      failedDAQPublisher.set(odometry.getFailedDAQs());
    }
  }

  public void checkHardware() {
    for (int m = 0; m < 4; m++) {
      modules[m].checkHardware();
    }
  }
}
