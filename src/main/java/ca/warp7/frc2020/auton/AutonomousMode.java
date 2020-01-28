package ca.warp7.frc2020.auton;

import ca.warp7.frc2020.auton.commands.DriveTrajectoryCommand;
import ca.warp7.frc2020.auton.commands.FlywheelCharacterizationCommand;
import ca.warp7.frc2020.commands.SingleFunctionCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

@SuppressWarnings("unused")
public class AutonomousMode {
    public static Command nothingMode() {
        return new InstantCommand();
    }

    public static Command testMode() {
        return new SequentialCommandGroup(
                SingleFunctionCommand.getSetDriveNativeVelocityPID(),
                SingleFunctionCommand.getSetDriveAutonomousLowGear(),
                SingleFunctionCommand.getIntakeExtensionToggle(),
                new DriveTrajectoryCommand(AutonomousPath.getInitLineShootingToTrench())
        );
    }

    public static Command flywheelCharacterizationMode() {
        return new FlywheelCharacterizationCommand();
    }
}
