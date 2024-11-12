package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Constants specific to the SwerveDrive base
 */
public final class SwerveConstants {
  // Speed configs
  public static final double kMAX_WHEEL_VELOCITY = 5.0; // Absolute max wheel m/s

  // CANCoder magnet offsets (in rotations)
  public static final double kFL_CANCODER_OFFSET = 0.1;
  public static final double kFR_CANCODER_OFFSET = 0.1;
  public static final double kRL_CANCODER_OFFSET = 0.1;
  public static final double kRR_CANCODER_OFFSET = 0.1;

  public static final AbsoluteSensorRangeValue kCANCODER_RANGE = 
    AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
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

  private static final AudioConfigs kDRIVE_MOTOR_AUDIO = new AudioConfigs()
    .withBeepOnBoot(false).withBeepOnConfig(true);

  public static final TalonFXConfiguration kDRIVE_CONFIG = new TalonFXConfiguration()
    .withSlot0(kDRIVE_MOTOR_GAINS)
    .withCurrentLimits(kDRIVE_MOTOR_CURRENT)
    .withAudio(kDRIVE_MOTOR_AUDIO);

  // Steer NEO
  public static final double kSTEER_RATIO = 150.0 / 7.0;

  public static final SparkBaseConfig kSTEER_CONFIG = new SparkMaxConfig()
    .smartCurrentLimit(20)
    .idleMode(IdleMode.kBrake)
    .apply(new ClosedLoopConfig()
      .pidf(0.45, 0.00001, 0.0, 0.0)
      .positionWrappingInputRange(-0.5 * kSTEER_RATIO, 0.5 * kSTEER_RATIO)
      .positionWrappingEnabled(true)
    );

  // Module positions
  public static final Translation2d[] kPOSITIONS = {
    new Translation2d(0.1, 0.1), // FL
    new Translation2d(0.1, 0.1), // FR
    new Translation2d(0.1, 0.1), // RL
    new Translation2d(0.1, 0.1)  // RR
  };
}
