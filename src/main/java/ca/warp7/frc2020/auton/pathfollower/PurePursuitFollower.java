package ca.warp7.frc2020.auton.pathfollower;

import ca.warp7.frc2020.lib.trajectory.ChassisVelocity;
import ca.warp7.frc2020.lib.trajectory.PathFollower;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

public class PurePursuitFollower implements PathFollower {

    private static final double kPathLookaheadTime = 0.4;  // seconds to look ahead along the path for steering
    private static final double kMinLookDist = 24.0;  // inches
    private static final double kLookaheadSearchDt = 0.01;
    private static final double kX = 5.0;


    Pose2d getLookahead(Trajectory trajectory, Trajectory.State state) {
        var lookaheadTime = state.timeSeconds + kPathLookaheadTime;
        var lookahead = trajectory.sample(lookaheadTime).poseMeters;
        var lookaheadTwist = state.poseMeters.log(lookahead);
        var lookaheadDistance = Math.hypot(lookaheadTwist.dx, lookaheadTwist.dy);

        while (lookaheadDistance < kMinLookDist &&
                (trajectory.getTotalTimeSeconds() - lookaheadTime) > kPathLookaheadTime) {

            lookaheadTime += kLookaheadSearchDt;
            lookahead = trajectory.sample(lookaheadTime).poseMeters;

            lookaheadTwist = state.poseMeters.log(lookahead);
            lookaheadDistance = Math.hypot(lookaheadTwist.dx, lookaheadTwist.dy);
        }

        if (lookaheadDistance < kMinLookDist) {
            lookahead = trajectory
                    .getStates().get(trajectory.getStates().size() - 1).poseMeters;
        }

        return lookahead;
    }

    @Override
    public ChassisVelocity calculateTrajectory(Trajectory trajectory, Trajectory.State state, Transform2d error) {

        var lookahead = getLookahead(trajectory, state);
        var initialToRobot = new Pose2d().plus(state.poseMeters.minus(new Pose2d().plus(error)));

        Pose2d robotToLookahead = new Pose2d().plus(lookahead.minus(initialToRobot));
        double y = robotToLookahead.getTranslation().getY();
        Twist2d log = new Pose2d().log(robotToLookahead);
        double l = Math.hypot(log.dx, log.dy);
        double curvature = (2 * y) / (l * l);

        double linear;
        double angular;
        linear = state.velocityMetersPerSecond + kX * error.getTranslation().getX();
        angular = curvature * state.velocityMetersPerSecond;
        return new ChassisVelocity(linear, angular);
    }

}