package ca.warp7.frc2020.auton.commands;

import ca.warp7.frc2020.subsystems.Limelight;
import ca.warp7.frc2020.lib.control.PID;
import ca.warp7.frc2020.lib.control.PIDController;
import ca.warp7.frc2020.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import java.util.function.Supplier;

import static ca.warp7.frc2020.Constants.kQuickTurnPID;

@SuppressWarnings("unused")
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

        double result = pidController.calculate(errorDegrees);
        double correction = MathUtil.clamp(
                result + driveTrain.getTransmission().ks * Math.signum(result) / 12.0,
                -1, 1
        );

        driveTrain.setPercentOutput(-correction, correction);
    }

    @Override
    public boolean isFinished() {
        return pidController.isDone();
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.neutralOutput();
    }

    /**
     * Move to an absolute angle
     */
    public static Command ofFieldOrientedAngle(Rotation2d target) {
        DriveTrain driveTrain = DriveTrain.getInstance();
        return new QuickTurnCommand(
                () -> target.minus(driveTrain.getRobotState().getRotation()),
                new PIDController(kQuickTurnPID)
        );
    }

    /**
     * Move by a certain relative angle
     */
    public static Command ofRelativeAngle(Rotation2d delta) {
        DriveTrain driveTrain = DriveTrain.getInstance();
        var targetState = new Object() { Rotation2d value; };
        var controller = new PIDController(kQuickTurnPID);
        controller.errorEpsilon = 2.0;
        controller.dErrorEpsilon = 2.0;
        controller.minTimeInEpsilon = 0.5;
        return new QuickTurnCommand(
                () -> targetState.value = driveTrain.getYaw().plus(delta),
                () -> targetState.value.minus(driveTrain.getYaw()),
                new PIDController(kQuickTurnPID)
        );
    }

    /**
     * Align with Limelight by driving the error towards 0
     */
    public static Command ofLimelight() {
        DriveTrain driveTrain = DriveTrain.getInstance();
        Limelight limelight = Limelight.getInstance();
        return new QuickTurnCommand(
                () -> Rotation2d.fromDegrees(-limelight.getHorizontalAngle()),
                new PIDController(new PID(0.0, 0.0, 0.0, 0.0))
        );
    }
}
