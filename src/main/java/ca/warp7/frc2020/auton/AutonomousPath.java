package ca.warp7.frc2020.auton;

import ca.warp7.frc2020.Constants;
import ca.warp7.frc2020.auton.commands.DriveTrajectoryCommand;
import ca.warp7.frc2020.auton.pathfollower.RamseteFollower;
import ca.warp7.frc2020.auton.pathfollower.SimpleFollower;
import ca.warp7.frc2020.lib.trajectory.TimedPath2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.Command;

@SuppressWarnings({"unused", "Unused"})
public class AutonomousPath {

    public static final DifferentialDriveKinematics kKinematics =
            new DifferentialDriveKinematics(Constants.kWheelBaseRadius * 2);

    public static final TrajectoryConstraint kKinematicsConstraint =
            new DifferentialDriveKinematicsConstraint(kKinematics, 10.0);

    public static TrajectoryConfig createTrajectoryConfig() {
        return new TrajectoryConfig(2.2, 1.0)
                .addConstraint(kKinematicsConstraint);
    }

    public static final Pose2d kTrench1 = new Pose2d(5.85, 3.38, new Rotation2d());
    public static final Pose2d kTrench3 = new Pose2d(7.70, 3.38, new Rotation2d());

    public static final Pose2d kRightSideFacingOuterGoal =
            new Pose2d(3.1, 3.0, Rotation2d.fromDegrees(23));

    public static Command getSimplePath() {
        return new TimedPath2d("Simple", new Pose2d())
                .addPoint(2, 0, 0)
                .addPoint(4, -2, -90)
                .setConfig(createTrajectoryConfig())
                .setFollower(new RamseteFollower())
                .convertTo(DriveTrajectoryCommand::new);
    }

    public static Command getTrenchThreeBalls() {
        return new TimedPath2d("one ball", kRightSideFacingOuterGoal)
                .addPoint(kTrench1)
                .addPoint(kTrench3)
                .setConfig(createTrajectoryConfig())
                .setFollower(new RamseteFollower())
                .convertTo(DriveTrajectoryCommand::new);
    }

    public static Command getTrenchThreeBallsReversed() {
        return new TimedPath2d("one ball reversed", kTrench3)
                .addPoint(kTrench1)
                .addPoint(kRightSideFacingOuterGoal)
                .setConfig(createTrajectoryConfig())
                .setReversed(true)
                .setFollower(new RamseteFollower())
                .convertTo(DriveTrajectoryCommand::new);
    }

    public static Command getTrenchOneBall() {
        return new TimedPath2d("three balls", kRightSideFacingOuterGoal)
                .addPoint(kTrench1)
                .setConfig(createTrajectoryConfig())
                .setFollower(new RamseteFollower())
                .convertTo(DriveTrajectoryCommand::new);
    }

    public static Command getTrenchToCentreShooting() {
        return new TimedPath2d("trench to shooting", kRightSideFacingOuterGoal)
                .addPoint(kTrench3)
                .addPoint(kRightSideFacingOuterGoal)
                .setConfig(createTrajectoryConfig())
                .setFollower(new SimpleFollower())
                .convertTo(DriveTrajectoryCommand::new);
    }
}

