package ca.warp7.frc2020.lib.control;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * Implements a PID control loop.
 *
 * This implementation has better control of resetting the integral gain,
 * compared with the WPILib version, but doesn't allow continuous input
 */
public class PIDController {
    public PID pid;

    public double maxOutput = Double.POSITIVE_INFINITY;
    public double minOutput = Double.NEGATIVE_INFINITY;
    public double maxIntegral = Double.POSITIVE_INFINITY;
    public double minIntegral = Double.NEGATIVE_INFINITY;
    public double errorEpsilon = 0.0;
    public double dErrorEpsilon = 0.0;
    public double minTimeInEpsilon = 0.0;

    public PIDController(PID pid) {
        this.pid = pid;
    }


    private double lastError = 0.0;
    private double sumError = 0.0;
    private double timeInEpsilon = 0.0;
    private double lastTime = 0.0;

    public double calculate(double error) {
        double time = Timer.getFPGATimestamp();
        double dt = time - lastTime;
        lastTime = time;

        // calculate the proportional gain
        double kP_gain = pid.kP * error;

        if (kP_gain <= minOutput || kP_gain >= maxOutput ||
                Math.abs(error) < errorEpsilon || kP_gain * sumError < 0) {
            // Reset sum when the integral gain is not useful
            sumError = 0;
        } else {
            // Numerically integrate sum of error
            sumError = MathUtil.clamp(sumError + error * dt, minIntegral, maxIntegral);
        }

        // calculate the integral gain
        double kI_gain = pid.kI * sumError;

        // Calculate change in error and the derivative gain
        double dError = (error - lastError) / dt;
        lastError = error;
        double kd_gain = pid.kD * dError;

        // Numerically integrate time in epsilon
        if (Math.abs(error) < errorEpsilon && Math.abs(dError) < dErrorEpsilon) {
            timeInEpsilon += dt;
        } else {
            timeInEpsilon = 0.0;
        }

        double output = kP_gain + kI_gain + kd_gain;
        return MathUtil.clamp(output, minOutput, maxOutput);
    }

    public double calculate(double setpoint, double actual) {
        // calculate the feed-forward gain
        double kF_gain = setpoint * pid.kF;

        double output = calculate(setpoint - actual) + kF_gain;
        return MathUtil.clamp(output, minOutput, maxOutput);
    }

    public double getTimeInEpsilon() {
        return timeInEpsilon;
    }

    public boolean isDone() {
        return timeInEpsilon >= minTimeInEpsilon;
    }
}
