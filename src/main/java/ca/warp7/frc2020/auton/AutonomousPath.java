package ca.warp7.frc2020.auton;

import ca.warp7.frc2020.Constants;
import ca.warp7.frc2020.auton.commands.DriveTrajectoryCommand;
import ca.warp7.frc2020.auton.pathfollower.RamseteFollower;
import ca.warp7.frc2020.lib.trajectory.TimedPath2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

@SuppressWarnings({"unused", "Unused"})
public class AutonomousPath {
    public static class InitLineLocation {
        public static final Pose2d kFacingTarget = new Pose2d(3.2,  1.7, Rotation2d.fromDegrees(-180));
        public static final Pose2d kCentre = new Pose2d(3.2,  0, Rotation2d.fromDegrees(-180));
    }

    public static class PowerCellIntakeLocation {
        public static final Pose2d kTrenchPowerCell3 = new Pose2d(8.0, 3.4, new Rotation2d());
    }

    public static class ShootingLocation {

    }

    public static Command getInitLineShootingToTrench() {
        return new TimedPath2d("InitLineShootingToTrench", InitLineLocation.kFacingTarget)
                .addPoint(PowerCellIntakeLocation.kTrenchPowerCell3)
                .setConfig(Constants.LowGear.kTrajectoryConfig)
                .setFollower(new RamseteFollower())
                .convertTo(DriveTrajectoryCommand::new);
    }

    public static Command getTrenchToInitLineShooting() {
        return new TimedPath2d("TrenchToInitLineShooting", InitLineLocation.kFacingTarget)
                .addPoint(PowerCellIntakeLocation.kTrenchPowerCell3)
                .setConfig(Constants.LowGear.kTrajectoryConfig)
                .setFollower(new RamseteFollower())
                .convertTo(DriveTrajectoryCommand::new);
    }
}
