package ca.warp7.frc2020.auton.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

@SuppressWarnings("unused")
public class Limelight {

    private static Limelight instance;

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry tv = table.getEntry("tv");
    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry ta = table.getEntry("ta");
    private NetworkTableEntry ts = table.getEntry("ts");

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

    public double getTargetAreaInImage() {
        return ta.getDouble(0.0);
    }

    public Rotation2d getSkew() {
        return Rotation2d.fromDegrees(ts.getDouble(0.0));
    }
}
