package frc.robot.subsystems.swerve;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Threads;

/**
 * 4-module SwerveOdometry thread based off of CTRE's SwerveBase.
 */
public class SwerveOdometry {
    private static final int UpdateFreq = 250;

    private final Thread thread;
    private final SwerveModule[] modules;
    private final BaseStatusSignal[] allSignals;

    // Public field accessors
    public final ReadWriteLock stateLock = new ReentrantReadWriteLock();
    public final SwerveDrivePoseEstimator poseEstimator;
    public final SwerveModuleState[] swerveStates = new SwerveModuleState[4];
    public final SwerveModulePosition[] swervePositions = new SwerveModulePosition[4];

    public SwerveOdometry(SwerveModule[] modules, SwerveDriveKinematics kinematics) {
        this.modules = modules;
        
        // Init thread
        thread = new Thread(this::run);
        thread.setDaemon(true);

        // Get status signals, positions, and states (positions and states are updated in-place)
        allSignals = new BaseStatusSignal[4*3];
        for (int m = 0; m < 4; m++) {
            allSignals[m*4+0] = modules[m].getDrivePosition();
            allSignals[m*4+1] = modules[m].getDriveVelocity();
            allSignals[m*4+2] = modules[m].getSteerAngle();

            swervePositions[m] = modules[m].position;
            swerveStates[m] = modules[m].state;
        }

        // Init pose estimator
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, null, swervePositions, Pose2d.kZero);

        // Start the thread immediately
        thread.start();
    }

    /**
     * @param newPosition New Position to completely reset odometry to.
     */
    public void resetPosition(Pose2d newPosition) {
        try {
            stateLock.writeLock().lock();
            poseEstimator.resetPosition(null, swervePositions, newPosition);
        } finally {
            stateLock.writeLock().unlock();
        }
    }

    public void run() {
        // Init update frequency
        BaseStatusSignal.setUpdateFrequencyForAll(UpdateFreq, allSignals);
        Threads.setCurrentThreadPriority(true, 1); // Priority 1

        // Run as fast as possible
        while (true) {
            // Wait up to twice period of update frequency
            StatusCode status = BaseStatusSignal.waitForAll(2.0 / UpdateFreq, allSignals);

            try {
                stateLock.writeLock().lock();

                // Update position and states
                for (int m = 0; m < 4; m++) {
                    modules[m].updatePositionAndState();
                }

                poseEstimator.update(null, swervePositions);
            } finally {
                stateLock.writeLock().unlock();
            }
        }
    }

    public Pose2d getPosition() {
        try {
            stateLock.readLock().lock();
            return poseEstimator.getEstimatedPosition();
        } finally {
            stateLock.readLock().unlock();
        }
    }

    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters, 
        double timestampSeconds, 
        Matrix<N3, N1> visionMeasurementStdDevs) {
        try {
            stateLock.writeLock().lock();
            poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
        }  finally {
            stateLock.writeLock().unlock();
        }
    }
}
