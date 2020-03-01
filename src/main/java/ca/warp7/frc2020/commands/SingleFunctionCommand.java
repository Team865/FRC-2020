package ca.warp7.frc2020.commands;

import ca.warp7.frc2020.subsystems.*;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static ca.warp7.frc2020.Constants.*;


@SuppressWarnings("unused")
public class SingleFunctionCommand {
    public static Command getSetDriveHighGear() {
        DriveTrain driveTrain = DriveTrain.getInstance();
        return new InstantCommand(() -> {
            driveTrain.configureRampRate(kHighGearRampRate);
            driveTrain.configurePID(kTeleopHighGearVelocityPID);
            driveTrain.setHighGear(true);
        });
    }

    public static Command getSetDriveLowGear() {
        DriveTrain driveTrain = DriveTrain.getInstance();
        return new InstantCommand(() -> {
            driveTrain.configureRampRate(kLowGearRampRate);
            driveTrain.configurePID(kTeleopLowGearVelocityPID);
            driveTrain.setHighGear(false);
        });
    }

    /**
     * Resets the drive train for auto driving. Make sure to call
     * at the start of all routines!
     */
    public static Command getResetAutonomousDrive() {
        DriveTrain driveTrain = DriveTrain.getInstance();
        return new InstantCommand(() -> {
            driveTrain.neutralOutput();
            driveTrain.configureRampRate(kLowGearRampRate);
            driveTrain.configurePID(kAutonLowGearVelocityPID);
            driveTrain.setHighGear(false);
            driveTrain.setBrake();
            driveTrain.setEncoderPosition(0, 0);
            driveTrain.setRobotState(new Pose2d());
            driveTrain.zeroYaw();
        });
    }

    public static Command getSetDriveBrakeMode() {
        DriveTrain driveTrain = DriveTrain.getInstance();
        return new InstantCommand(driveTrain::setBrake);
    }

    public static Command getDriveServo() {
        DriveTrain driveTrain = DriveTrain.getInstance();
        return new FunctionalCommand(
                () -> driveTrain.setEncoderPosition(0, 0),
                () -> driveTrain.setWheelPositionPID(0, 0),
                (interrupted) -> driveTrain.neutralOutput(),
                () -> false,
                driveTrain
        );
    }

    public static Command getZeroYaw() {
        DriveTrain driveTrain = DriveTrain.getInstance();
        return new InstantCommand(driveTrain::zeroYaw);
    }

    public static Command getClimbLockToggle() {
        Climber climber = Climber.getInstance();
        return new InstantCommand(climber::toggleLock);
    }

    public static Command getIntakeExtensionToggle() {
        Intake intake = Intake.getInstance();
        return new InstantCommand(intake::toggle);
    }


    public static Command getFlywheelSetHoodCloseCommand() {
        Flywheel flywheel = Flywheel.getInstance();
        return new InstantCommand(() -> flywheel.setHoodCloseShot(true));
    }

    public static Command getFlywheelSetHoodFarCommand() {
        Flywheel flywheel = Flywheel.getInstance();
        return new InstantCommand(() -> flywheel.setHoodCloseShot(false));
    }

    public static Command getStartCompressor() {
        Infrastructure infrastructure = Infrastructure.getInstance();
        return new InstantCommand(infrastructure::startCompressor);
    }

    public static Command getStopCompressor() {
        Infrastructure infrastructure = Infrastructure.getInstance();
        return new InstantCommand(infrastructure::stopCompressor);
    }
}
