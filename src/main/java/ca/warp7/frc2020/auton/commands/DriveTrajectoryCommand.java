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
import ca.warp7.frc2020.subsystems.DriveTrain;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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

    private final String name;
    private final PathFollower follower;
    private final TimedPath2d path;

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("trajectory");
    private final NetworkTableEntry e_t = table.getEntry("t");
    private final NetworkTableEntry e_x = table.getEntry("x");
    private final NetworkTableEntry e_y = table.getEntry("y");
    private final NetworkTableEntry e_theta = table.getEntry("theta");
    private final NetworkTableEntry e_linear = table.getEntry("linear");
    private final NetworkTableEntry e_angular = table.getEntry("angular");

    public DriveTrajectoryCommand(TimedPath2d path) {
        Objects.requireNonNull(path, "path cannot be null");
        this.follower = path.getFollower();
        this.name = path.getName() + "+" + follower.getClass().getSimpleName();
        this.path = path;
        addRequirements(driveTrain);
    }

    private double rioTime;
    private double trajectoryTime;
    private double totalTrajectoryTime;
    private boolean trajectoryStarted;
    private double generationTimeMs;
    private int generationLoopCount;

    private Pose2d offset;
    private List<Trajectory> trajectories;
    private FutureTask<List<Trajectory>> trajectoryGenerator;

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

        System.out.println("Trajectory Error:" + error);

        // Send signal to drive train
        driveTrain.setChassisVelocity(correctedVelocity.getLinear(), correctedVelocity.getAngular());

        // Write logs
        e_t.setDouble(trajectoryTime);
        e_x.setDouble(robotState.getTranslation().getX());
        e_y.setDouble(robotState.getTranslation().getY());
        e_theta.setDouble(robotState.getRotation().getDegrees());
        e_linear.setDouble(correctedVelocity.getLinear());
        e_angular.setDouble(correctedVelocity.getAngular());
    }

    private void tryStartTrajectory() {
        // only run if the trajectory is not started
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
            offset = driveTrain.getRobotState().relativeTo(firstTrajectoryPose);

            // find the  total trajectory time
            totalTrajectoryTime = 0;
            for (Trajectory trajectory : trajectories) {
                totalTrajectoryTime += trajectory.getTotalTimeSeconds();
            }

            System.out.println("Finished Generating Trajectory in " +
                    generationTimeMs + "ms, and " + generationLoopCount + " loops.");
            System.out.println("Computed Offset: " + offset);
            System.out.println("==== BEGIN TRAJECTORY FOLLOWING ====");

            trajectoryStarted = true;
            rioTime = Timer.getFPGATimestamp();
        }
    }

    @Override
    public void initialize() {
        System.out.println("Start Generating Trajectory: " + name);
        driveTrain.neutralOutput();
        if (trajectoryGenerator != null) {
            throw new IllegalStateException("Trajectory is already generated");
        }
        // create a new trajectory generator on another thread
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
        if (trajectoryStarted) {
            calculateTrajectory();
        } else {
            tryStartTrajectory();
        }
    }

    @Override
    public boolean isFinished() {
        return trajectoryTime > totalTrajectoryTime;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.neutralOutput();
        System.out.println("==== END TRAJECTORY FOLLOWING ====");
    }
}
