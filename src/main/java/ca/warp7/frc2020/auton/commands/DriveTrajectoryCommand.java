/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.auton.commands;

import ca.warp7.frc2020.lib.trajectory.PathFollower;
import ca.warp7.frc2020.lib.trajectory.TimedPath2d;
import ca.warp7.frc2020.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.List;
import java.util.Objects;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.FutureTask;

import static edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber;

public class DriveTrajectoryCommand extends CommandBase {
    private DriveTrain driveTrain = DriveTrain.getInstance();

    private final String name;
    private final PathFollower follower;
    private final TimedPath2d path;

    private double rioTime;
    private double trajectoryTime;
    private double totalTrajectoryTime;
    private boolean trajectoryStarted;
    private double generationTimeMs;
    private int generationLoopCount = 1;

    private Pose2d offset;
    private List<Trajectory> trajectories;
    private FutureTask<List<Trajectory>> trajectoryGenerator;

    public DriveTrajectoryCommand(TimedPath2d path) {
        this.path = path;
        Objects.requireNonNull(this.path, "path cannot be null");
        this.follower = path.getFollower();
        Objects.requireNonNull(this.follower, "follwer cannot be null");

        this.name = path.getName() + "+" + follower.getClass().getSimpleName();
        addRequirements(driveTrain);
    }

    private void calculateTrajectory() {
        var newTime = Timer.getFPGATimestamp();
        var dt = newTime - rioTime;
        rioTime = newTime;

        // First make sure that we know where we are
        var robotState = driveTrain.getRobotState();

        var robotRelativeToTrajectory = robotState.relativeTo(offset);

        // Add dt to the amount of time tracked
        trajectoryTime += dt;

        // Lookup the current trajectory in the list
        var trajectoryFinishedTime = 0;
        var currentTrajectory = trajectories.get(0);

//        for (var trajectory : trajectories) {
//            if ((trajectoryFinishedTime + trajectory.getTotalTimeSeconds()) >= trajectoryTime) {
//                currentTrajectory = trajectory;
//                break;
//            }
//            trajectoryFinishedTime += trajectory.getTotalTimeSeconds();
//        }

        // Calculate the relative time to the current trajectory
        var relativeTime = trajectoryTime - trajectoryFinishedTime;

        // Sample based on relative time
        var sample = currentTrajectory.sample(relativeTime);

        var targetPose = sample.poseMeters;

        // Compute the error
        var error = targetPose.minus(robotRelativeToTrajectory);

        // Correct for the error using the follower
        var correctedVelocity = follower.calculateTrajectory(currentTrajectory, sample, error);

        // Send signal to drive train
        driveTrain.setChassisVelocity(correctedVelocity.getLinear(), correctedVelocity.getAngular());

        // Write logs

        double v = sample.velocityMetersPerSecond;
        double w = v * sample.curvatureRadPerMeter;

        double linearCorrection = correctedVelocity.getLinear() - v;
        double angularCorrection = correctedVelocity.getAngular() - w;

        putNumber("Trajectory Time", trajectoryTime);

        putNumber("Target X (m)", targetPose.getTranslation().getX());
        putNumber("Target Y (m)", targetPose.getTranslation().getY());
        putNumber("Target Angle (deg)", targetPose.getRotation().getDegrees());

        putNumber("Error X (m)", error.getTranslation().getX());
        putNumber("Error Y (m)", error.getTranslation().getY());
        putNumber("Error Angle (deg)", error.getRotation().getDegrees());

        putNumber("Target Linear (m/s)", v);
        putNumber("Target Angular (deg/s)", Math.toDegrees(w));

        putNumber("Linear Correction(m/s)", linearCorrection);
        putNumber("Angular Correction (deg/s)", Math.toDegrees(angularCorrection));

        putNumber("Left PID Error (m/s)", driveTrain.getLeftPIDError());
        putNumber("Right PID Error (m/s)", driveTrain.getRightPIDError());
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
            var firstTrajectoryPose = trajectories.get(0).getInitialPose();
            offset = driveTrain.getRobotState().relativeTo(firstTrajectoryPose);

            // find the  total trajectory time
            totalTrajectoryTime = 0;
            for (var trajectory : trajectories) {
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
        Thread thread = new Thread(trajectoryGenerator);
        thread.setName("Trajectory Generator");
        thread.start();
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
