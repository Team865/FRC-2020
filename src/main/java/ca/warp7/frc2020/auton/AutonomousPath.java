package ca.warp7.frc2020.auton;

import ca.warp7.frc2020.Constants;
import ca.warp7.frc2020.auton.commands.DriveTrajectoryCommand;
import ca.warp7.frc2020.auton.pathfollower.RamseteFollower;
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
        return new TrajectoryConfig(2.2, 2.2)
                .addConstraint(kKinematicsConstraint);
    }

    public static final Pose2d kTrench1 = new Pose2d(5.85, 3.38, new Rotation2d());
    public static final Pose2d kTrench3 = new Pose2d(7.80, 3.38, new Rotation2d());

    public static final Pose2d kRightSideFacingOuterGoal =
            new Pose2d(3.1, 3.0, Rotation2d.fromDegrees(23));

    public static final Pose2d kLeftInitLine =
            new Pose2d(3.5, -3.38, new Rotation2d());

    public static final Pose2d kOpponentTrench2 =
            new Pose2d(6.0, -3.38, new Rotation2d());

    public static final Pose2d kCentreFieldshoot =
            new Pose2d(3.1, 0.0, Rotation2d.fromDegrees(-28));

    public static final Pose2d kTrenchCorner =
            new Pose2d(5.3, 2.9, Rotation2d.fromDegrees(13));

    public static Command getTrenchThreeBalls() {
        return new TimedPath2d("one ball", kRightSideFacingOuterGoal)
                .addPoint(kTrench1)
                .addPoint(kTrench3)
                .setConfig(createTrajectoryConfig())
                .setFollower(new RamseteFollower())
                .convertTo(DriveTrajectoryCommand::new);
    }

    public static Command getTrenchThreeBallsToCorner() {
        return new TimedPath2d("one ball reversed", kTrench3)
                .addPoint(kTrenchCorner)
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

    public static Command getOneMetreForward() {
        return new TimedPath2d("one metre forwatrd", new Pose2d())
                .addPoint(1.0, 0.0, 0.0)
                .setConfig(createTrajectoryConfig())
                .setFollower(new RamseteFollower())
                .convertTo(DriveTrajectoryCommand::new);
    }

    public static Command getOpponentTrenchTwoBalls() {
        return new TimedPath2d("opponent trench two balls", kLeftInitLine)
                .addPoint(kOpponentTrench2)
                .setConfig(createTrajectoryConfig())
                .setFollower(new RamseteFollower())
                .convertTo(DriveTrajectoryCommand::new);
    }

    public static Command getOpponentTrechTwoBallsToShoot() {
        return new TimedPath2d("OpponentTrechTwoBallsToShoot", kOpponentTrench2)
                .addPoint(kCentreFieldshoot)
                .setConfig(createTrajectoryConfig())
                .setReversed(true)
                .setFollower(new RamseteFollower())
                .convertTo(DriveTrajectoryCommand::new);
    }
}

