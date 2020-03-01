package ca.warp7.frc2020.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

@SuppressWarnings("unused")
public class Limelight implements Subsystem {

    private static Limelight instance;

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry tv = table.getEntry("tv");
    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry tl = table.getEntry("tl");

    private double smoothHorizontalAngle = 0.0;
    private boolean smoothAngleExists = false;

    public static Limelight getInstance() {
        if (instance == null) instance = new Limelight();
        return instance;
    }

    private boolean prevEnabled = false;
    private double prevT;
    private double prevAngle;
    private DriveTrain driveTrain = DriveTrain.getInstance();

    @Override
    public void periodic() {
        boolean enabled = RobotState.isEnabled();
        if (enabled) {
            double t = Timer.getFPGATimestamp();
            double angle = driveTrain.getContinousAngleRadians() * 180 / Math.PI;
            if (prevEnabled) {
                if (this.hasValidTarget()) {
                    double angleChange = -1 * (prevAngle - angle);
                    double angularVelocity = angleChange / (prevT - t); // TODO div/0?
                    double latency = this.getLatencySeconds();

                    double targetAngle = this.getHorizontalAngle() + angularVelocity * latency;
                    if (smoothAngleExists) {
                        smoothHorizontalAngle += angleChange;
                        double smoothing = 0.9;
                        smoothHorizontalAngle = smoothHorizontalAngle * smoothing + targetAngle * (1 - smoothing);
                        smoothAngleExists = true;
                    } else
                        smoothHorizontalAngle = targetAngle;
                } else
                    smoothAngleExists = false;
            }

            prevT = t;
            prevAngle = angle;
        }
        prevEnabled = enabled;
    }

    public void setSmoothHorizontalAngle(Double smoothHorizontalAngle) {
        this.smoothHorizontalAngle = smoothHorizontalAngle;
    }

    public Double getSmoothHorizontalAngle() {
        return this.smoothHorizontalAngle;
    }

    public boolean hasValidTarget() {
        return tv.getDouble(0.0) != 0;
    }

    public double getHorizontalAngle() {
        return tx.getDouble(Double.NaN);
    }

    public double getVerticalAngle() {
        return ty.getDouble(Double.NaN);
    }

    public double getLatencySeconds() {
        return tl.getDouble(0.0) / 1000;
    }

    // the center location of the power ports on the field
    private static final Pose2d kTargetToField = new Pose2d(0.0, 1.7, Rotation2d.fromDegrees(-180));

    // the centre height of the target
    // 81.25 in (bottom) and 17 in (height) -> 89.75in = 2.27965 m
    private static final double kTargetCentreHeight = 2.26;

    // the height of the camera relative to the carpet in metres todo
    private static final double kCameraHeight = 34.0 * 0.0254;

    // the transform from the camera lens to the centre of rotation of the robot todo
    private static final Transform2d kCameraToRobot =
            new Transform2d(new Translation2d(0.0, 0.0), new Rotation2d());

    // the angle that the camera is mounted relative to the horizontal in degrees.
    // up is positive todo
    private static final double kCameraMountingAngle = 23.0;

    // the relative height between the camera and the target
    private static final double kCameraToTargetHeight = kTargetCentreHeight - kCameraHeight;

    public double getCameraToTarget() {
        return kCameraToTargetHeight / Math.tan(Math.toRadians(getVerticalAngle() + kCameraMountingAngle));
    }
}
