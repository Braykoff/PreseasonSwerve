package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

/**
 * Constants specific to the SwerveDrive base
 */
public final class SwerveConstants {
  // Speed configs
  public static final double kMAX_WHEEL_VELOCITY = 5.0; // Absolute max wheel m/s

  // Max drive speeds
  public static final double kFAST_TRANSLATIONAL_SPEED = 5.0; // m/s
  public static final double kFAST_ROTATIONAL_SPEED = 2.0 * 2.0 * Math.PI; // rad/s

  public static final double kSLOW_TRANSLATIONAL_SPEED = kFAST_TRANSLATIONAL_SPEED * 0.5;
  public static final double kSLOW_ROTATIONAL_SPEED = kFAST_TRANSLATIONAL_SPEED * 0.5;

  // CANCoder magnet offsets (in rotations, CCW+, -0.5 to 0.5 range)
  public static final double kFL_CANCODER_OFFSET = 0.1;
  public static final double kFR_CANCODER_OFFSET = 0.1;
  public static final double kRL_CANCODER_OFFSET = 0.1;
  public static final double kRR_CANCODER_OFFSET = 0.1;

  public static final double kCANCODER_RANGE = 0.5;
  public static final SensorDirectionValue kCANCODER_DIRECTION = 
    SensorDirectionValue.CounterClockwise_Positive;

  // Drive Kraken X60
  public static final double kWHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0) * Math.PI;
  public static final double kDRIVE_RATIO = 6.75; // SDS Mk4i L2

  private static final Slot0Configs kDRIVE_MOTOR_GAINS = new Slot0Configs()
    .withKP(0.1).withKI(0.02).withKD(0.0)
    .withKS(0.0).withKV(0.0).withKA(0.0);

  private static final CurrentLimitsConfigs kDRIVE_MOTOR_CURRENT = new CurrentLimitsConfigs()
    .withSupplyCurrentLimit(45).withSupplyCurrentLimitEnable(true)
    .withStatorCurrentLimit(80).withStatorCurrentLimitEnable(true);

  private static final FeedbackConfigs kDRIVE_ENCODER = new FeedbackConfigs()
    .withSensorToMechanismRatio(kDRIVE_RATIO);

  private static final AudioConfigs kDRIVE_MOTOR_AUDIO = new AudioConfigs()
    .withBeepOnBoot(false).withBeepOnConfig(true);

  public static final TalonFXConfiguration kDRIVE_CONFIG = new TalonFXConfiguration()
    .withSlot0(kDRIVE_MOTOR_GAINS)
    .withCurrentLimits(kDRIVE_MOTOR_CURRENT)
    .withFeedback(kDRIVE_ENCODER)
    .withAudio(kDRIVE_MOTOR_AUDIO);

  // Steer NEO
  public static final double kSTEER_RATIO = 150.0 / 7.0;

  public static final SparkBaseConfig kSTEER_CONFIG = new SparkMaxConfig()
    .inverted(true)
    .smartCurrentLimit(20)
    .idleMode(IdleMode.kBrake)
    .apply(new ClosedLoopConfig()
      .pidf(0.45, 0.00001, 0.0, 0.0)
      .positionWrappingInputRange(-0.5 * kSTEER_RATIO, 0.5 * kSTEER_RATIO)
      .positionWrappingEnabled(true)
    );

  // ADIS16470 Gyro
  public static final IMUAxis kGYRO_YAW = IMUAxis.kX;
  public static final IMUAxis kGYRO_PITCH = IMUAxis.kY;
  public static final IMUAxis kGYRO_ROLL = IMUAxis.kZ;
  public static final CalibrationTime kGYRO_CALIBRATION = CalibrationTime._4s;

  // Base size
  public static final Translation2d kBASE_DIMENSIONS = 
    new Translation2d(Units.inchesToMeters(23), Units.inchesToMeters(30));

  // Module positions
  /*
   * Positive x values represent moving toward the front of the robot whereas positive y values 
   * represent moving toward the left of the robot.
   */
  private static final double kEDGE = Units.inchesToMeters(2.625); // Distance edge of module to center of wheel

  public static final Translation2d[] kPOSITIONS = {
    new Translation2d(kBASE_DIMENSIONS.getX() / 2.0 - kEDGE, kBASE_DIMENSIONS.getY() / 2.0 - kEDGE), // FL
    new Translation2d(kBASE_DIMENSIONS.getX() / 2.0 - kEDGE, kBASE_DIMENSIONS.getY() / -2.0 + kEDGE), // FR
    new Translation2d(kBASE_DIMENSIONS.getX() / -2.0 + kEDGE, kBASE_DIMENSIONS.getY() / 2.0 - kEDGE), // RL
    new Translation2d(kBASE_DIMENSIONS.getX() / -2.0 + kEDGE, kBASE_DIMENSIONS.getY() / -2.0 + kEDGE)  // RR
  };
}
