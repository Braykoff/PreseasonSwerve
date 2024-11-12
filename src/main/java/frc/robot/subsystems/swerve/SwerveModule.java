package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.HardwareConstants;

public class SwerveModule {
  private final String name;

  // Alert
  private final Alert driveMotorAlert, steerMotorAlert, steerCANCoderAlert, steerNotHomedAlert;

  // Drive hardware
  private final TalonFX driveMotor;
  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;

  private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0.0);
  private final DutyCycleOut driveDutyCycleRequest = new DutyCycleOut(0.0);
  private final VoltageOut driveVoltageRequest = new VoltageOut(0.0);

  // Steer hardware
  private final SparkMax steerMotor;
  private final SparkClosedLoopController steerPID;
  private final RelativeEncoder steerBuiltInEncoder;

  private final CANcoder steerCANCoder;
  private final StatusSignal<Angle> steerAngle;

  // Position/state accessors
  public final SwerveModuleState state = new SwerveModuleState();
  public final SwerveModulePosition position = new SwerveModulePosition();

  /**
   * Initializes a new Swerve Module. 
   * Kraken X60 for drive, NEO for steer, CANCoder for absolute angle.
   * @param name Unique readable name to identify this module.
   * @param driveMotorCAN Drive Kraken X60 CAN id.
   * @param steerMotorCAN Steer CAN Spark Max CAN id.
   * @param steerEncoderCAN Steer CANCoder CAN id.
   * @param steerCANCoderOffsetRots Steer CANCoder absolute position offset, in rotations.
   */
  public SwerveModule(
    String name, 
    int driveMotorCAN, 
    int steerMotorCAN, 
    int steerEncoderCAN, 
    double steerCANCoderOffsetRots
  ) {
    this.name = name;

    // Initialize Alerts
    driveMotorAlert = new Alert("Swerve Module " + name + " drive motor error", AlertType.kError);
    steerMotorAlert = new Alert("Swerve Module " + name + " steer motor error", AlertType.kError);
    steerCANCoderAlert = new Alert("Swerve Module " + name + " steer CANCoder error", AlertType.kError);
    steerNotHomedAlert = new Alert("Swerve Module " + name + " steer encoder failed to home", AlertType.kError);

    // Initialize DRIVE MOTOR
    driveMotor = new TalonFX(driveMotorCAN, HardwareConstants.kCanivore);
    driveMotor.getConfigurator().apply(SwerveConstants.kDRIVE_CONFIG);
    drivePosition = driveMotor.getPosition();
    driveVelocity = driveMotor.getVelocity();

    // Initialize STEER ENCODER
    steerCANCoder = new CANcoder(steerEncoderCAN, HardwareConstants.kCanivore);

    MagnetSensorConfigs cancoderConfig = new MagnetSensorConfigs()
      .withAbsoluteSensorRange(SwerveConstants.kCANCODER_RANGE)
      .withSensorDirection(SwerveConstants.kCANCODER_DIRECTION)
      .withMagnetOffset(steerCANCoderOffsetRots);

    steerCANCoder.getConfigurator().apply(cancoderConfig);
    steerAngle = steerCANCoder.getAbsolutePosition();

    // Initialize STEER MOTOR
    steerMotor = new SparkMax(steerMotorCAN, MotorType.kBrushless);
    steerMotor.setInverted(true);
    steerMotor.configure(SwerveConstants.kSTEER_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    steerBuiltInEncoder = steerMotor.getEncoder();
    steerPID = steerMotor.getClosedLoopController();

    // Default state
    setBrakeMode(true);
    setRequest(new SwerveModuleState(), true); // zero velocity, zero angle
  }

  /**
   * @return The drive motor's velocity status signal.
   */
  public StatusSignal<AngularVelocity> getDriveVelocity() {
    return driveVelocity;//.clone();
  }

  public StatusSignal<Angle> getDrivePosition() {
    return drivePosition;
  }

  /**
   * @return The steer motor's position status signal.
   */
  public StatusSignal<Angle> getSteerAngle() {
    return steerAngle;//.clone();
  }

  /**
   * Updates the position and state objects in place.
   */
  public void updatePositionAndState() {
    Rotation2d angle = Rotation2d.fromRotations(steerAngle.getValueAsDouble());

    position.angle = angle;
    position.distanceMeters = drivePosition.getValueAsDouble() * SwerveConstants.kWHEEL_CIRCUMFERENCE / SwerveConstants.kDRIVE_RATIO;

    state.angle = angle;
    state.speedMetersPerSecond = driveVelocity.getValueAsDouble() * SwerveConstants.kWHEEL_CIRCUMFERENCE / SwerveConstants.kDRIVE_RATIO;
  }

  /**
   * Homes the steer Neo's built-in relative encoder.
   */
  public void homeEncoder() {
    System.out.printf(
      "SwerveModule %s homed from %f deg to %f deg.\n",
      name,
      steerBuiltInEncoder.getPosition() / SwerveConstants.kSTEER_RATIO * 360.0,
      steerAngle.getValueAsDouble() * 360.0
    );

    REVLibError resp = steerBuiltInEncoder.setPosition(steerAngle.getValueAsDouble() * SwerveConstants.kSTEER_RATIO);
    steerNotHomedAlert.set(!resp.equals(REVLibError.kOk));
  }

  /**
   * @param brake Whether the steer and drive motors should have brake mode enabled.
   */
  public void setBrakeMode(boolean brake) {
    //steerMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast); // TODO fix this
    driveMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  /**
   * @param request Velocity and angle to apply.
   * @param closedLoop Open loop control (teleop) or closed loop control (auto).
   */
  public void setRequest(SwerveModuleState request, boolean closedLoop) {
    // Optimize
    Rotation2d currentAngle = Rotation2d.fromRotations(steerAngle.getValueAsDouble());
    request.optimize(currentAngle);
    request.cosineScale(currentAngle);

    // Set angle
    steerPID.setReference(
      request.angle.getRotations() * SwerveConstants.kSTEER_RATIO, ControlType.kPosition);

    // Set velocity
    if (closedLoop) {
      // Closed loop velocity control (auto)
      driveMotor.setControl(driveVelocityRequest.withVelocity(
        request.speedMetersPerSecond / SwerveConstants.kWHEEL_CIRCUMFERENCE * SwerveConstants.kDRIVE_RATIO
      ));
    } else {
      // Open loop control (teleop)
      driveMotor.setControl(driveDutyCycleRequest.withOutput(
        request.speedMetersPerSecond / SwerveConstants.kMAX_WHEEL_VELOCITY
      ));
    }
  }

  /**
   * Steer heading will default to 0.0 degrees (forward).
   * @param voltage Voltage request (for characterization).
   */
  public void setVoltageRequest(double voltage) {
    steerPID.setReference(0.0, ControlType.kPosition);
    driveMotor.setControl(driveVoltageRequest.withOutput(voltage));
  }

  /**
   * Logs data to a DataLog.
   */
  public void log() {

  }

  /**
   * Checks all the hardware and triggers an alert.
   */
  public void checkHardware() {
    // Check drive motor faults
    driveMotorAlert.set(!driveMotor.isAlive() || !driveMotor.isConnected());

    // Check steer motor faults
    steerMotorAlert.set(steerMotor.hasActiveFault() || steerMotor.hasActiveWarning());

    // Check steer encoder faults
    steerCANCoderAlert.set(steerCANCoder.isConnected());
  }
}
