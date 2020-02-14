package ca.warp7.frc2020.auton.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

@SuppressWarnings("unused")
public class Limelight {

    private static Limelight instance;

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry tv = table.getEntry("tv");
    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");

    public static Limelight getInstance() {
        if (instance == null) instance = new Limelight();
        return instance;
    }

    public boolean hasValidTarget() {
        return tv.getDouble(0.0) != 0;
    }

    public double getHorizontalOffset() {
        return tx.getDouble(Double.NaN);
    }

    public double getVerticalOffset() {
        return ty.getDouble(Double.NaN);
    }


    // the center location of the power ports on the field
    private static final Pose2d kTargetToField = new Pose2d(0.0, 1.7, Rotation2d.fromDegrees(-180));

    // the centre height of the target
    // 81.25 in (bottom) and 17 in (height) -> 89.75in = 2.27965 m
    private static final double kTargetCentreHeight = 2.27965;

    // the height of the camera relative to the carpet in metres todo
    private static final double kCameraHeight = 0.0;

    // the transform from the camera lens to the centre of rotation of the robot todo
    private static final Transform2d kCameraToRobot =
            new Transform2d(new Translation2d(0.0, 0.0), new Rotation2d());

    // the angle that the camera is mounted relative to the horizontal in degrees.
    // up is positive todo
    private static final double kCameraMountingAngle = 0.0;

    // the relative height between the camera and the target
    private static final double kCameraToTargetHeight = kTargetCentreHeight - kCameraHeight;

    public double getCameraToTarget() {
        return kCameraToTargetHeight / Math.tan(getVerticalOffset() + kCameraMountingAngle);
    }

    /**
     * Create an estimated robot-to-target pose transform, based on
     * a robot to field estimation (provided by drive train)
     * http://docs.limelightvision.io/en/latest/cs_estimating_distance.html
     */
    public Pose2d getRobotToTarget(Pose2d robotToFieldEstimation) {

        boolean has_target = hasValidTarget();

        if (!has_target) {
            // return the best guess based on the drive train
            return kTargetToField.relativeTo(robotToFieldEstimation);
        }

        double x = getHorizontalOffset();
        double camera_to_target_dist = getCameraToTarget();

        Pose2d camera_to_field_estimation = kTargetToField
                .relativeTo(robotToFieldEstimation.plus(kCameraToRobot));
        Rotation2d camera_to_target_angle = camera_to_field_estimation
                .getRotation().plus(kTargetToField.getRotation());

        Rotation2d angle = camera_to_target_angle.plus(Rotation2d.fromDegrees(x));
        double relative_x = camera_to_target_dist * angle.getCos();
        double relative_y = camera_to_target_dist * angle.getSin();

        return new Pose2d(new Translation2d(relative_x, relative_y), angle)
                .relativeTo(new Pose2d().plus(kCameraToRobot));
    }
}
