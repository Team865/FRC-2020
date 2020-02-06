package ca.warp7.frc2020.auton.commands;

import ca.warp7.frc2020.lib.control.PIDController;
import ca.warp7.frc2020.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

public class QuickTurnCommand extends CommandBase {
    private DriveTrain driveTrain = DriveTrain.getInstance();

    // this returns a Rotation2d because we don't want the angle to overflow
    // --- we try to get the error to 0
    private Supplier<Rotation2d> errorSupplier;
    private PIDController pidController;
    private Runnable onInitialize;

    public QuickTurnCommand(
            Supplier<Rotation2d> errorSupplier,
            PIDController pidController
    ) {
        this(null, errorSupplier, pidController);
    }

    public QuickTurnCommand(
            Runnable onInitialize,
            Supplier<Rotation2d> errorSupplier,
            PIDController pidController
    ) {
        this.onInitialize = onInitialize;
        this.errorSupplier = errorSupplier;
        this.pidController = pidController;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        if (onInitialize != null) {
            onInitialize.run();
        }
    }

    @Override
    public void execute() {
        // this is ensured to be between [-180, 180]
        double errorDegrees = errorSupplier.get().getDegrees();

        double correction = pidController.calculate(errorDegrees);
        driveTrain.setChassisVelocity(0, correction);
    }

    @Override
    public boolean isFinished() {
        return pidController.isDone();
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.neutralOutput();
    }
}
