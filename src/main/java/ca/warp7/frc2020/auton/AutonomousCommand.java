package ca.warp7.frc2020.auton;

import ca.warp7.frc2020.auton.commands.LimelightCalculationCommand;
import ca.warp7.frc2020.auton.commands.ShootBallsCloseCommand;
import ca.warp7.frc2020.auton.commands.VisionAlignCommand;
import ca.warp7.frc2020.auton.vision.Limelight;
import ca.warp7.frc2020.commands.SingleFunctionCommand;
import ca.warp7.frc2020.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.*;

import static ca.warp7.frc2020.commands.SingleFunctionCommand.getFlywheelSetHoodCloseCommand;

public class AutonomousCommand extends CommandBase {

    private Command zeroYawCommand = SingleFunctionCommand.getZeroYaw();
    private Command limelightCalculationCommand = new LimelightCalculationCommand();

    private AutonomousSelector selector = AutonomousSelector.getInstance();

    @Override
    public void initialize() {
        zeroYawCommand.schedule();
        limelightCalculationCommand.schedule();

//        Command mode = AutonomousMode.simplePathMode();//selector.getSelectedMode().create();

        DriveTrain driveTrain = DriveTrain.getInstance();
        Command mode =   new ShootBallsCloseCommand(3);
        if (mode != null) {
            mode.schedule();
        }
    }
}
