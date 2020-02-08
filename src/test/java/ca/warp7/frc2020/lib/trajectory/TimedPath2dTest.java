package ca.warp7.frc2020.lib.trajectory;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class TimedPath2dTest {

    private static final TrajectoryConfig kConfig =
            new TrajectoryConfig(1.0, 1.0);

    @Test
    void testOnePointThrows() {
        var tp = new TimedPath2d("test", new Pose2d());
        Assertions.assertThrows(NullPointerException.class, tp::asTrajectory);
        tp.setConfig(kConfig);
        Assertions.assertThrows(IllegalArgumentException.class, tp::asTrajectory);
    }
}
