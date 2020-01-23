package ca.warp7.frc2020.auton.pathfollower;

import ca.warp7.frc2020.lib.trajectory.ChassisVelocity;
import ca.warp7.frc2020.lib.trajectory.PathFollower;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

/**
 * A follower that doesn't correct for error
 */
public class SimpleFollower implements PathFollower {
    @Override
    public ChassisVelocity calculateTrajectory(Trajectory trajectory, Trajectory.State state, Transform2d error) {
        double v = state.velocityMetersPerSecond;
        return new ChassisVelocity(v, v * state.curvatureRadPerMeter);
    }

}
