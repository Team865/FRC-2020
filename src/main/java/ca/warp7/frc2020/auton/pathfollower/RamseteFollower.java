package ca.warp7.frc2020.auton.pathfollower;

import ca.warp7.frc2020.lib.trajectory.ChassisVelocity;
import ca.warp7.frc2020.lib.trajectory.PathFollower;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

public class RamseteFollower implements PathFollower {

    private static final double kBeta = 2.0; // Correction coefficient, β > 0
    private static final double kZeta = 0.7; // Damping coefficient, 0 < ζ < 1

    @Override
    public ChassisVelocity calculateTrajectory(Trajectory trajectory, Trajectory.State state, Transform2d error) {
        double v = state.velocityMetersPerSecond;
        double w = v * state.curvatureRadPerMeter;

        return calculate(v, w, error);
    }

    /**
     * Calculate the ramsete follower
     * @param v velocity in metres per second
     * @param w angular velocity in radians per second
     * @param error the error to correct
     * @return the corrected velocity
     */
    public ChassisVelocity calculate(double v, double w, Transform2d error) {
        double k = 2.0 * kZeta * Math.sqrt(kBeta * v * v + w * w);

        double angularError = error.getRotation().getRadians();

        double sinRatio = Math.abs(angularError) < 1E9 ?
                1.0 - 1.0 / 6.0 * angularError * angularError
                : error.getRotation().getSin() / angularError;

        double linear = v * error.getRotation().getCos() + k * error.getTranslation().getX();
        double angular = w + k * angularError + v * kBeta * sinRatio * error.getTranslation().getY();

        return new ChassisVelocity(linear, angular);
    }
}
