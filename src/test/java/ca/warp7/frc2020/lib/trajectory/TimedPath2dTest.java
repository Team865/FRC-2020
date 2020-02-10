package ca.warp7.frc2020.lib.trajectory;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
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
        var tp = new TimedPath2d("test", new Pose2d());
        tp.addForward(3.0);
        assertEquals(new Pose2d(3.0, 0.0, new Rotation2d()), tp.getPoints().get(1).pose);
        tp.setConfig(kConfig);
        assertFalse(tp.asTrajectory().isEmpty());
    }
}
