package ca.warp7.frc2020.auton;

import ca.warp7.frc2020.auton.commands.DriveCharacterizationCommand;
import ca.warp7.frc2020.auton.commands.ResetRobotStateCommand;
import ca.warp7.frc2020.auton.commands.ShootBallsCloseCommand;
import ca.warp7.frc2020.auton.commands.VisionAlignCommand;
import ca.warp7.frc2020.commands.IntakingCommand;
import ca.warp7.frc2020.commands.SingleFunctionCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

@SuppressWarnings("unused")
public class AutonomousMode {

    public static Command shootThreeBalls() {
        return new ShootBallsCloseCommand(3);
    }

    public static Command shoot3_intake3_shoot3() {
        return new SequentialCommandGroup(
                SingleFunctionCommand.getResetAutonomousDrive(),
                new ResetRobotStateCommand(AutonomousPath.kRightSideFacingOuterGoal),
                new ShootBallsCloseCommand(3),
                AutonomousPath.getTrenchThreeBalls()
                        .deadlineWith(new IntakingCommand(() -> 1.0)),
                AutonomousPath.getTrenchThreeBallsToCorner()
                        .deadlineWith(new IntakingCommand(() -> 0.0)),
                new ShootBallsCloseCommand(3)
        );
    }

    public static Command intakeThreeBalls() {
        return new SequentialCommandGroup(
                SingleFunctionCommand.getResetAutonomousDrive(),
                new ResetRobotStateCommand(AutonomousPath.kRightSideFacingOuterGoal),
                new ParallelDeadlineGroup(
                        AutonomousPath.getTrenchThreeBalls(),
                        new IntakingCommand(() -> 1.0)
                ),
                new ParallelDeadlineGroup(
                        AutonomousPath.getTrenchThreeBallsToCorner(),
                        new IntakingCommand(() -> 0.0)
                )
        );
    }

    public static Command driveLowGearCharacterization() {
        return new SequentialCommandGroup(
                SingleFunctionCommand.getResetAutonomousDrive(),
                SingleFunctionCommand.getStopCompressor(),
                new DriveCharacterizationCommand()
        );
    }

    public static Command driveHighGearCharacterization() {
        return new SequentialCommandGroup(
                SingleFunctionCommand.getResetAutonomousDrive(),
                SingleFunctionCommand.getSetDriveHighGear(),
                SingleFunctionCommand.getStopCompressor(),
                new DriveCharacterizationCommand()
        );
    }

    public static Command simplePath() {
        return new SequentialCommandGroup(
                SingleFunctionCommand.getResetAutonomousDrive(),
                new ResetRobotStateCommand(AutonomousPath.kRightSideFacingOuterGoal),
                AutonomousPath.getTrenchOneBall()
        );
    }
}
