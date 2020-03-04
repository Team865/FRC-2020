package ca.warp7.frc2020.auton.commands;

import ca.warp7.frc2020.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

public class RobotStateCommand extends CommandBase {
    private DriveTrain driveTrain = DriveTrain.getInstance();
    private Supplier<Pose2d> robotStateSupplier;

    public RobotStateCommand(Supplier<Pose2d> robotStateSupplier) {
        this.robotStateSupplier = robotStateSupplier;
    }

    public RobotStateCommand(Pose2d robotState) {
        this(() -> robotState);
    }

    public RobotStateCommand() {
        this(new Pose2d());
    }

    @Override
    public void initialize() {
        Pose2d robotState = robotStateSupplier.get();
        System.out.println("Setting Robot State to " + robotState);
        driveTrain.setRobotState(robotState);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
