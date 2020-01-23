package ca.warp7.frc2020.auton;

import ca.warp7.frc2020.commands.SingleFunctionCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

public class AutonomousCommand extends CommandBase {

    private Command zeroYawCommand = SingleFunctionCommand.getZeroYaw();
    private Command resetRobotStateCommand = SingleFunctionCommand.getResetRobotState();
    private Command robotStateEstimationCommand = SingleFunctionCommand.getRobotStateEstimation();

    private Supplier<Command> autonomousModeSupplier = AutonomousMode::nothingMode;

    @Override
    public void initialize() {
        zeroYawCommand.schedule();
        resetRobotStateCommand.schedule();
        robotStateEstimationCommand.schedule();

        Command mode = autonomousModeSupplier.get();
        if (mode != null) {
            mode.schedule();
        }
    }
}
