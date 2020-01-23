package ca.warp7.frc2020.auton.pathfollower;

import ca.warp7.frc2020.lib.trajectory.ChassisVelocity;
import ca.warp7.frc2020.lib.trajectory.PathFollower;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

public class PIDFollower implements PathFollower {

    private static final double kX = 5.0;
    private static final double kY = 5.0;
    private static final double kTheta = 1.0;


    @Override
    public ChassisVelocity calculateTrajectory(Trajectory trajectory, Trajectory.State state, Transform2d error) {
        double v = state.velocityMetersPerSecond;
        double w = v * state.curvatureRadPerMeter;

        double newLinear = v + kX * error.getTranslation().getX();
        double newAngular = w + kY * v * error.getTranslation().getY()
                + kTheta * error.getRotation().getRadians();
        return new ChassisVelocity(newLinear, newAngular);
    }

}