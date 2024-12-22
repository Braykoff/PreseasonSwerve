package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class TeleopDriveCommand extends Command {
    private SwerveDrive drive;
    private Supplier<Double> ySupplier, xSupplier, thetaSupplier;
    private BooleanSupplier fastMode;

    /**
     * @param drive
     * @param ySupplier Left is +, percentage
     * @param xSupplier Forward is +, percentage
     * @param thetaSupplier CCW+, percentage
     * @param fastMode
     */
    public TeleopDriveCommand(
        SwerveDrive drive, 
        Supplier<Double> ySupplier, 
        Supplier<Double> xSupplier, 
        Supplier<Double> thetaSupplier,
        BooleanSupplier fastMode
    ) {
        this.drive = drive;
        this.ySupplier = ySupplier;
        this.xSupplier = xSupplier;
        this.thetaSupplier = thetaSupplier;
        this.fastMode = fastMode;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.drive(
            ySupplier.get() * (fastMode.getAsBoolean() ? SwerveConstants.kFAST_TRANSLATIONAL_SPEED : SwerveConstants.kSLOW_TRANSLATIONAL_SPEED), 
            xSupplier.get() * (fastMode.getAsBoolean() ? SwerveConstants.kFAST_TRANSLATIONAL_SPEED : SwerveConstants.kSLOW_TRANSLATIONAL_SPEED), 
            thetaSupplier.get() * (fastMode.getAsBoolean() ? SwerveConstants.kFAST_ROTATIONAL_SPEED : SwerveConstants.kSLOW_ROTATIONAL_SPEED), 
            false, 
            true
        );
    }

    @Override
    public void end(boolean terminated) {
        drive.drive(0.0, 0.0, 0.0, false, false);
    }
}
