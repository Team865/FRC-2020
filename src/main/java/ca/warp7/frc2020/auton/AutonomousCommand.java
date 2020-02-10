package ca.warp7.frc2020.auton;

import ca.warp7.frc2020.commands.SingleFunctionCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonomousCommand extends CommandBase {

    private Command zeroYawCommand = SingleFunctionCommand.getZeroYaw();
    private Command resetRobotStateCommand = SingleFunctionCommand.getResetRobotState();

    private AutonomousSelector selector = AutonomousSelector.getInstance();

    @Override
    public void initialize() {
        zeroYawCommand.schedule();
        resetRobotStateCommand.schedule();

        Command mode = selector.getSelectedMode().create();
        if (mode != null) {
            mode.schedule();
        }
    }
}
