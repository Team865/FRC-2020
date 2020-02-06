package ca.warp7.frc2020.auton;

import ca.warp7.frc2020.auton.commands.DriveCharacterizationCommand;
import ca.warp7.frc2020.auton.commands.FlywheelCharacterizationCommand;
import ca.warp7.frc2020.commands.SingleFunctionCommand;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;

@SuppressWarnings("unused")
public class AutonomousMode {
    public static Command nothingMode() {
        return new InstantCommand();
    }

    public static Command testMode() {
        return new SequentialCommandGroup(
                SingleFunctionCommand.getRobotStateEstimation(),
                SingleFunctionCommand.getSetDriveNativeVelocityPID(),
                SingleFunctionCommand.getSetDriveAutonomousLowGear(),
                SingleFunctionCommand.getIntakeExtensionToggle(),
                AutonomousPath.getInitLineShootingToTrench()
        );
    }

    public static Command flywheelCharacterizationMode() {
        return new FlywheelCharacterizationCommand();
    }

    public static Command lowGearCharacterizationMode() {
        return new SequentialCommandGroup(
                SingleFunctionCommand.getSetDriveLowGear(),
                new DriveCharacterizationCommand()
        );
    }

    public static Command highGearCharacterizationMode() {
        return new SequentialCommandGroup(
                SingleFunctionCommand.getSetDriveHighGear(),
                new DriveCharacterizationCommand()
        );
    }

    private static Command directShootThenTrenchIntakeMode() {
        return new SequentialCommandGroup(
                new ScheduleCommand(
                        SingleFunctionCommand.getRobotStateEstimation()
                ),
                new ParallelCommandGroup(
                        SingleFunctionCommand.getSetDriveNativeVelocityPID(),
                        SingleFunctionCommand.getSetDriveAutonomousLowGear(),
                        SingleFunctionCommand.getIntakeExtensionToggle()
                ),
                QuickTurn.ofFieldOrientedAngle(Rotation2d.fromDegrees(30.0)),
                AutonomousPath.getInitLineShootingToTrench()
        );
    }
}
