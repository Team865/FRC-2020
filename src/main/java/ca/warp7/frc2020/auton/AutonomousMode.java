package ca.warp7.frc2020.auton;

import ca.warp7.frc2020.Constants;
import ca.warp7.frc2020.auton.commands.DriveCharacterizationCommand;
import ca.warp7.frc2020.auton.commands.QuickTurnCommand;
import ca.warp7.frc2020.auton.commands.FeedAutoCommand;
import ca.warp7.frc2020.auton.commands.ResetRobotStateCommand;
import ca.warp7.frc2020.commands.FlywheelSpeedCommand;
import ca.warp7.frc2020.commands.SingleFunctionCommand;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;

@SuppressWarnings("unused")
public class AutonomousMode {

    public static Command testMode() {
        return new SequentialCommandGroup(
                SingleFunctionCommand.getSetDriveAutonomousLowGear(),
                SingleFunctionCommand.getIntakeExtensionToggle(),
                AutonomousPath.getInitLineShootingToTrench()
        );
    }

    public static Command driveCharacterizationMode() {
        return new SequentialCommandGroup(
                SingleFunctionCommand.getSetDriveAutonomousLowGear(),
                SingleFunctionCommand.getZeroYaw(),
                new DriveCharacterizationCommand()
        );
    }

    public static Command simplePathMode() {
        return new SequentialCommandGroup(
                SingleFunctionCommand.getSetDriveAutonomousLowGear(),
                SingleFunctionCommand.getZeroYaw(),
                new ResetRobotStateCommand(),
                AutonomousPath.getSimplePath()
        );
    }
    
    public static Command shooterTest(){
        return new ParallelCommandGroup(
                new FlywheelSpeedCommand(() -> Constants.flywheelDefaultCloseRPS), 
                new FeedAutoCommand());
    }

    private static Command directShootThenTrenchIntakeMode() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        SingleFunctionCommand.getSetDriveAutonomousLowGear(),
                        SingleFunctionCommand.getIntakeExtensionToggle()
                ),
                QuickTurnCommand.ofFieldOrientedAngle(Rotation2d.fromDegrees(30.0)),
                AutonomousPath.getInitLineShootingToTrench()
        );
    }
}
