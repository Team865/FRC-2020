package ca.warp7.frc2020.lib.control;

/**
 * Just a simple class to store some PID values
 */
public class PID {
    public double kP;
    public double kI;
    public double kD;
    public double kF;

    /** Allocates a PIDController with the given constants for Kp, Ki, and Kd.
     *
     * @param kP     The proportional coefficient.
     * @param kI     The integral coefficient.
     * @param kD     The derivative coefficient.
     * @param kF     The feed-forward coefficient.
     */
    public PID(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }
}
