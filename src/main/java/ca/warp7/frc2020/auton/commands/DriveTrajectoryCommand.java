/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.auton.commands;

import ca.warp7.frc2020.lib.trajectory.ChassisVelocity;
import ca.warp7.frc2020.lib.trajectory.PathFollower;
import ca.warp7.frc2020.lib.trajectory.TimedPath2d;
import ca.warp7.frc2020.lib.trajectory.TrajectoryLogger;
import ca.warp7.frc2020.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.List;
import java.util.Objects;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.FutureTask;

public class DriveTrajectoryCommand extends CommandBase {
    private DriveTrain driveTrain = DriveTrain.getInstance();

    private PathFollower follower;
    private boolean debug;
    private TrajectoryLogger logger;
    private double rioTime;
    private double trajectoryTime;
    private double totalTrajectoryTime;
    private boolean notifierStarted;
    private Pose2d offset;

    private TimedPath2d path;
    private List<Trajectory> trajectories;
    private FutureTask<List<Trajectory>> trajectoryGenerator;
    private double generationTimeMs;
    private int generationLoopCount;

    // we put trajectory tracking in a notifier
    private Notifier calculationNotifier = new Notifier(this::calculateTrajectory);

    public DriveTrajectoryCommand(TimedPath2d path) {
        this(path, false);
    }

    public DriveTrajectoryCommand(TimedPath2d path, boolean debug) {
        Objects.requireNonNull(path, "path cannot be null");
        this.debug = debug;
        this.follower = path.getFollower();
        this.logger = new TrajectoryLogger(path.getName() + "+" + follower.getClass().getSimpleName());
        this.path = path;
        addRequirements(driveTrain);
    }

    private void calculateTrajectory() {
        double newTime = Timer.getFPGATimestamp();
        double dt = newTime - rioTime;
        rioTime = newTime;

        // First make sure that we know where we are
        driveTrain.updateRobotStateEstimation();
        Pose2d robotState = driveTrain.getRobotState();

        Pose2d robotRelativeToTrajectory = robotState.relativeTo(offset);

        // Add dt to the amount of time tracked
        trajectoryTime += dt;

        // Lookup the current trajectory in the list
        var trajectoryFinishedTime = 0;
        var currentTrajectory = trajectories.get(0);

        for (Trajectory trajectory : trajectories) {
            if ((trajectoryFinishedTime + trajectory.getTotalTimeSeconds()) > trajectoryTime) {
                currentTrajectory = trajectory;
                break;
            }
            trajectoryFinishedTime += trajectory.getTotalTimeSeconds();
        }

        // Calculate the relative time to the current trajectory
        var relativeTime = trajectoryTime - trajectoryFinishedTime;

        // Sample based on relative time
        Trajectory.State sample = currentTrajectory.sample(relativeTime);

        Pose2d targetPose = sample.poseMeters;

        // Compute the error
        Transform2d error = targetPose.minus(robotRelativeToTrajectory);

        // Correct for the error using the follower
        ChassisVelocity correctedVelocity = follower
                .calculateTrajectory(currentTrajectory, sample, error);

        if (debug) {
            System.out.println("Goal: " + targetPose + "\t Error:" + error);
        }

        // Send signal to drive train
        driveTrain.setChassisVelocity(correctedVelocity.getLinear(), correctedVelocity.getAngular());

        // Write logs
        logger.writeToBuffer(
                targetPose.getTranslation().getX(),
                targetPose.getTranslation().getY(),
                targetPose.getRotation().getDegrees(),
                sample.velocityMetersPerSecond,
                sample.accelerationMetersPerSecondSq,
                sample.curvatureRadPerMeter,
                robotState.getTranslation().getX(),
                robotState.getTranslation().getY(),
                robotState.getRotation().getDegrees(),
                error.getTranslation().getX(),
                error.getTranslation().getY(),
                error.getRotation().getDegrees(),
                correctedVelocity.getLinear(),
                correctedVelocity.getAngular()
        );
    }

    @Override
    public void initialize() {
        if (debug) {
            System.out.println("Start Generating Trajectory: " + logger.getName());
        }
        driveTrain.neutralOutput();
        if (trajectoryGenerator != null) {
            throw new IllegalStateException("Trajectory is already generated");
        }
        trajectoryGenerator = new FutureTask<>(() -> {
            long initialTime = System.nanoTime();
            List<Trajectory> result = path.asTrajectory();
            generationTimeMs = (System.nanoTime() - initialTime) / 1E6;
            return result;
        });
        new Thread(trajectoryGenerator).start();
    }

    @Override
    public void execute() {
        if (!notifierStarted) {
            if (trajectoryGenerator == null || !trajectoryGenerator.isDone()) {
                // trajectories is not done yet
                generationLoopCount++;
            } else {

                // get the calculated trajectories
                try {
                    trajectories = trajectoryGenerator.get();
                    trajectoryGenerator = null;
                } catch (InterruptedException | ExecutionException e) {
                    e.printStackTrace();
                }

                // calculate the trajectory offset to the robot state
                Pose2d firstTrajectoryPose = trajectories.get(0).getInitialPose();
                offset = new Pose2d().plus(driveTrain.getRobotState().minus(firstTrajectoryPose));

                totalTrajectoryTime = trajectories.stream().mapToDouble(Trajectory::getTotalTimeSeconds).sum();

                if (debug) {
                    System.out.println("Finished Generating Trajectory in " +
                            generationTimeMs + "ms, and " + generationLoopCount + " loops.");
                    System.out.println("Computed Offset: " + offset);
                    System.out.println("==== BEGIN TRAJECTORY FOLLOWING ====");
                }

                notifierStarted = true;
                rioTime = Timer.getFPGATimestamp();
                calculationNotifier.startPeriodic(0.01);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return trajectoryTime >= totalTrajectoryTime;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.neutralOutput();
        logger.saveToFile();
        if (debug) {
            System.out.println("==== END TRAJECTORY FOLLOWING ====");
        }
    }
}
