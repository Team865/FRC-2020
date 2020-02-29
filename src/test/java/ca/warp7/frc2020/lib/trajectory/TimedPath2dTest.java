package ca.warp7.frc2020.lib.trajectory;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class TimedPath2dTest {

    private static final TrajectoryConfig kConfig =
            new TrajectoryConfig(1.0, 1.0);

    @Test
    void testOnePointThrows() {
        var tp = new TimedPath2d("test", new Pose2d());
        assertThrows(NullPointerException.class, tp::asTrajectory);
        tp.setConfig(kConfig);
        assertThrows(IllegalArgumentException.class, tp::asTrajectory);
    }

    @Test
    void testSimple() {
        var tp = new TimedPath2d("test", new Pose2d()).addForward(3.0);
        assertEquals(new Pose2d(3.0, 0.0, new Rotation2d()), tp.getPoints().get(1).pose);
        tp.setConfig(kConfig);
        assertDoesNotThrow(tp::asTrajectory);
    }

    @Test
    public void testGenerateReversed() {
        final Pose2d kTrench1Reversed = new Pose2d(5.85, 3.38, Rotation2d.fromDegrees(0));
        final Pose2d kTrench3Reversed = new Pose2d(7.70, 3.38, Rotation2d.fromDegrees(0));

        var path = new TimedPath2d("", kTrench3Reversed)
                .addPoint(kTrench1Reversed)
                .setConfig(new TrajectoryConfig(2.2, 1.0))
                .setReversed(true);

        Assertions.assertDoesNotThrow(path::asTrajectory);
        Assertions.assertDoesNotThrow(path::asTrajectory);
    }
}
