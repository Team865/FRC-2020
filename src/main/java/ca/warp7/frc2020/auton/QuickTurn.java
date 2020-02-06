package ca.warp7.frc2020.auton;

import ca.warp7.frc2020.auton.commands.QuickTurnCommand;
import ca.warp7.frc2020.auton.vision.Limelight;
import ca.warp7.frc2020.lib.control.PID;
import ca.warp7.frc2020.lib.control.PIDController;
import ca.warp7.frc2020.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class QuickTurn {
    public static Command ofFieldOrientedAngle(Rotation2d target) {
        DriveTrain driveTrain = DriveTrain.getInstance();
        return new QuickTurnCommand(
                () -> target.minus(driveTrain.getRobotState().getRotation()),
                new PIDController(new PID(0.0, 0.0, 0.0, 0.0))
        );
    }

    public static Command ofRelativeAngle(Rotation2d delta) {
        DriveTrain driveTrain = DriveTrain.getInstance();
        var targetState = new Object() { Rotation2d value; };
        return new QuickTurnCommand(
                () -> targetState.value = driveTrain.getYaw().plus(delta),
                () -> targetState.value.minus(driveTrain.getYaw()),
                new PIDController(new PID(0.0, 0.0, 0.0, 0.0))
        );
    }

    public static Command ofLimelight() {
        DriveTrain driveTrain = DriveTrain.getInstance();
        Limelight limelight = Limelight.getInstance();
        return new QuickTurnCommand(
                () -> Rotation2d.fromDegrees(-limelight.getHorizontalOffset()),
                new PIDController(new PID(0.0, 0.0, 0.0, 0.0))
        );
    }
}
