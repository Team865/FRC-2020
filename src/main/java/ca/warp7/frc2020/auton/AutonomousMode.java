package ca.warp7.frc2020.auton;

import ca.warp7.frc2020.Constants;
import ca.warp7.frc2020.auton.commands.*;
import ca.warp7.frc2020.commands.FlywheelSpeedCommand;
import ca.warp7.frc2020.commands.IntakingCommand;
import ca.warp7.frc2020.commands.SingleFunctionCommand;
import edu.wpi.first.wpilibj2.command.*;

@SuppressWarnings("unused")
public class AutonomousMode {

    public static Command shootThreeBalls() {
        return new ParallelDeadlineGroup(
                new ShootBallsCloseCommand(3),
                new LimelightCalculationCommand()
        );
    }

    public static Command shootThreeBallsThenIntake() {
        return new SequentialCommandGroup(
                SingleFunctionCommand.getResetAutonomousDrive(),
                new ResetRobotStateCommand(AutonomousPath.kRightSideFacingOuterGoal),
                new ShootBallsCloseCommand(3),
                new ParallelDeadlineGroup(
                        AutonomousPath.getTrenchThreeBalls(),
                        new IntakingCommand(() -> 1.0)
                ),
                AutonomousPath.getTrenchToCentreShooting(),
                new ParallelDeadlineGroup(
                        new WaitCommand(2.0),
                        new VisionAlignCommand(() -> 0)
                ),
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
                        AutonomousPath.getTrenchThreeBallsReversed(),
                        new IntakingCommand(() -> 0.0)
                )
        );
    }

    public static Command driveCharacterization() {
        return new SequentialCommandGroup(
                SingleFunctionCommand.getResetAutonomousDrive(),
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

    public static Command shooterTest() {
        return new ParallelCommandGroup(
                new FlywheelSpeedCommand(() -> Constants.flywheelDefaultCloseRPS),
                new FeedAutoCommand());
    }
}
