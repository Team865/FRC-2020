package ca.warp7.frc2020.auton;

import ca.warp7.frc2020.auton.commands.ShootBallsCloseCommand;
import ca.warp7.frc2020.commands.SingleFunctionCommand;
import ca.warp7.frc2020.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.*;

public class AutonomousCommand extends CommandBase {

    private Command zeroYawCommand = SingleFunctionCommand.getZeroYaw();

    private AutonomousSelector selector = AutonomousSelector.getInstance();

    @Override
    public void initialize() {
        zeroYawCommand.schedule();

//        Command mode = AutonomousMode.simplePathMode();//selector.getSelectedMode().create();

        Command mode =   new ShootBallsCloseCommand(3);
        if (mode != null) {
            mode.schedule();
        }
    }
}
