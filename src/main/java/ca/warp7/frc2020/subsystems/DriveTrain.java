/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.subsystems;

import ca.warp7.frc2020.lib.control.PID;
import ca.warp7.frc2020.subsystems.drivetrain.DriveTrainVariant;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static ca.warp7.frc2020.Constants.*;

public final class DriveTrain implements Subsystem {

    private static DriveTrain instance;

    public static DriveTrain getInstance() {
        if (instance == null) {
            instance = new DriveTrain();
        }
        return instance;
    }

    private static DriveTrainVariant driveTrainVariant;

    public static void setVariant(DriveTrainVariant variant) {
        if (driveTrainVariant != null) {
            throw new IllegalStateException("Cannot set drive train variant");
        }
        driveTrainVariant = variant;
    }

    // private final Solenoid shifterSolenoid = new Solenoid(kDriveShifterID);
    private final AHRS navx = new AHRS(I2C.Port.kMXP, (byte) 100);

    private boolean isHighGear = false;
    private boolean isUsingNativeVelocityPID = false;

    // Used to calculate expected acceleration
    private double previousLinear = 0.0;
    private double previousAngular = 0.0;
    private double previousTime = 0.0;

    // Used to calculate robot state estimation
    private Pose2d robotState = new Pose2d(); // metres
    private Rotation2d previousYaw = new Rotation2d();
    private double previousLeftPosition = 0.0; // metres
    private double previousRightPosition = 0.0; // metres

    private DriveTrain() {
    }

    /**
     * @return isHighGear
     */
    public boolean isHighGear() {
        return isHighGear;
    }

    /**
     * Set the high gear state. If different than current state,
     * shifter is applied the new state, otherwise do nothing
     *
     * @param highGear whether to go high gear
     */
    public void setHighGear(boolean highGear) {
        if (highGear != isHighGear) {
            isHighGear = highGear;
            // shifterSolenoid.set(highGear);
        }
    }

    /**
     * Set whether to offload velocity commands to the motor controller so
     * that it can respond to errors at a faster rate. If false, velocity
     * commands are converted to percent output
     * @param usingNativeVelocityPID whether to run the native velocity PID
     */
    public void setUsingNativeVelocityPID(boolean usingNativeVelocityPID) {
        isUsingNativeVelocityPID = usingNativeVelocityPID;
    }

    /**
     * @return the transmission feed forward calculator for high or low gear
     */
    public SimpleMotorFeedforward getTransmission() {
        return isHighGear ? HighGear.kTransmission : LowGear.kTransmission;
    }

    /**
     * @return the direction reading of the gyro as a Rotation2D
     */
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(navx.getYaw());
    }

    /**
     * Reset the gyro's yaw to 0 (keep track of internal offset)
     */
    public void zeroYaw() {
        navx.zeroYaw();
        previousYaw = new Rotation2d();
    }

    /**
     * @return the conversion factor between motor rotation and distance of
     * wheel travel based on high or low gear
     */
    public double getMetresPerRotation() {
        return isHighGear ? HighGear.kMetresPerRotation : LowGear.kMetresPerRotation;
    }

    /**
     * @return the robot state estimation on the field
     */
    public Pose2d getRobotState() {
        return robotState;
    }

    /**
     * @return the encoder position of the left motors in m
     */
    public double getLeftPosition() {
        return driveTrainVariant.getLeftPositionRotations() *
                getMetresPerRotation();
    }

    /**
     * @return the encoder position of the right motors in m
     */
    public double getRightPosition() {
        return driveTrainVariant.getRightPositionRotations() *
                getMetresPerRotation();
    }

    /**
     * @return the encoder velocity of the left motors in m/s
     */
    public double getLeftVelocity() {
        return driveTrainVariant.getLeftVelocityRotationsPerSecond() *
                getMetresPerRotation();
    }

    /**
     * @return the encoder velocity of the right motors in m/s
     */
    public double getRightVelocity() {
        return driveTrainVariant.getRightVelocityRotationsPerSecond() *
                getMetresPerRotation();
    }

    /**
     * Update robot state estimation.
     * If gyro is connected, use the gyro for heading, otherwise
     * calculate the change in heading from the difference between
     * left and right wheels
     */
    public void updateRobotStateEstimation() {
        double leftPosition = getLeftPosition();
        double rightPosition = getRightPosition();
        double leftDelta = leftPosition - previousLeftPosition;
        double rightDelta = rightPosition - previousRightPosition;
        previousLeftPosition = leftPosition;
        previousRightPosition = rightPosition;

        Rotation2d angleDelta;
        if (navx.isConnected() && !navx.isCalibrating()) {
            Rotation2d currentYaw = getYaw();
            // Make sure that we have a previous yaw, otherwise we could
            // end up with the wrong delta if the gyro suddenly reconnects
            if (previousYaw != null) {
                angleDelta = currentYaw.minus(previousYaw);
            } else {
                angleDelta = new Rotation2d();
            }
            previousYaw = currentYaw;
        } else {
            System.out.println("WARNING the gyro is not connected");
            // Calculate the angle delta from wheel deltas
            angleDelta = new Rotation2d((rightDelta - leftDelta) / (2 * kWheelBaseRadius));
            previousYaw = null;
        }

        // Construct the pose exponential transform and add it to current state
        double distanceDelta = (leftDelta + rightDelta) / 2.0;
        Twist2d twist = new Twist2d(distanceDelta, 0.0, angleDelta.getRadians());
        robotState = robotState.exp(twist);
    }

    /**
     * Reset the robot state to the origin
     */
    public void resetRobotState() {
        if (navx.isConnected() && !navx.isCalibrating()) {
            previousYaw = getYaw();
            robotState = new Pose2d(new Translation2d(), previousYaw);
        } else {
            System.out.println("WARNING the gyro is not connected");
            previousYaw = null;
            robotState = new Pose2d();
        }
        resetEncoderPosition();
    }


    /**
     * Configured the on-board PID values for the Talons
     *
     * @param pid the PID values
     */
    public void configurePID(PID pid) {
        driveTrainVariant.configurePID(pid);
    }

    /**
     * Configures the open-loop ramp rate of throttle output.
     *
     * @param secondsFromNeutralToFull
     *            Minimum desired time to go from neutral to full throttle. A
     *            value of '0' will disable the ramp.
     */
    public void configureRampRate(double secondsFromNeutralToFull) {
        driveTrainVariant.configureRampRate(secondsFromNeutralToFull);
    }

    /**
     * Set the encoder position
     *
     * @param left the left position in metres
     * @param right the right position in metres
     */
    public void setEncoderPosition(double left, double right) {
        double metresPerRotation = getMetresPerRotation();
        driveTrainVariant.setEncoderPosition(
                left / metresPerRotation,
                right / metresPerRotation
        );
    }

    /**
     * Reset the encoder position to 0
     */
    public void resetEncoderPosition() {
        setEncoderPosition(0, 0);
    }

    /**
     * Disables the drive motors
     */
    public void neutralOutput() {
        driveTrainVariant.neutralOutput();
        previousLinear = 0.0;
        previousAngular = 0.0;
    }

    /**
     * Set the motor speed without any feedback
     *
     * @param left  the left speed between [-1, 1]
     * @param right the right speed between [-1, 1]
     */
    public void setPercentOutput(double left, double right) {
        driveTrainVariant.setPercentOutput(left, right);
    }

    /**
     * Set the position setpoint for individual sides of the drive train
     *
     * @param left the left position target, in metres
     * @param right the right position target, in metres
     */
    @SuppressWarnings("unused")
    public void setWheelPositionPID(double left, double right) {
        double metresPerRotation = getMetresPerRotation();
        driveTrainVariant.setPositionPID(
                left / metresPerRotation,
                right / metresPerRotation
        );
    }

    /**
     * Set the velocity setpoint and feed forward for individual wheels
     *
     * @param leftVelocity  desired left wheel velocity in metres per second
     * @param rightVelocity desired right wheel velocity in metres per second
     * @param leftVoltage   left wheel voltage feed forward in volts
     * @param rightVoltage  right wheel voltage feed forward in volts
     */
    public void setWheelVelocity(
            double leftVelocity,
            double rightVelocity,
            double leftVoltage,
            double rightVoltage
    ) {
        double metresPerRotation = getMetresPerRotation();
        double leftRotationsPerSecond = leftVelocity / metresPerRotation;
        double rightRotationsPerSecond = rightVelocity / metresPerRotation;

        if (isUsingNativeVelocityPID) {
            driveTrainVariant.setVelocityPID(
                    leftRotationsPerSecond, rightRotationsPerSecond,
                    leftVoltage, rightVoltage);
        } else {
            driveTrainVariant.setVoltage(leftVoltage, rightVoltage);
        }
    }

    /**
     * Set the velocity and acceleration setpoint for the entire robot.
     *
     * @param linearVelocity      desired linear velocity in metres per second
     * @param angularVelocity     desired angular velocity in radians per second
     * @param linearAcceleration  desired linear acceleration in metres per second squared
     * @param angularAcceleration desired angular acceleration in radians per second squared
     */
    public void setChassisKinematics(
            double linearVelocity,
            double angularVelocity,
            double linearAcceleration,
            double angularAcceleration
    ) {
        double leftVelocity = linearVelocity - angularVelocity * kWheelBaseRadius;
        double leftAcceleration = linearAcceleration - angularAcceleration * kWheelBaseRadius;

        double rightVelocity = linearVelocity + angularVelocity * kWheelBaseRadius;
        double rightAcceleration = linearAcceleration + angularAcceleration * kWheelBaseRadius;

        SimpleMotorFeedforward transmission = getTransmission();
        double leftVoltage = transmission.calculate(leftVelocity, leftAcceleration);
        double rightVoltage = transmission.calculate(rightVelocity, rightAcceleration);

        setWheelVelocity(leftVelocity, rightVelocity, leftVoltage, rightVoltage);
    }

    /**
     * Set the chess velocity setpoint for the entire robot
     * This method uses a previously stored value of velocity setpoint
     * to calculate acceleration
     *
     * @param linear  the linear velocity to adjust to in metres per second
     * @param angular the angular velocity to adjust to in radians per second
     */
    public void setChassisVelocity(
            double linear,
            double angular
    ) {
        double time = Timer.getFPGATimestamp();
        double dt = time - previousTime;
        previousTime = time;

        double linearAcceleration = (linear - previousLinear) / dt;
        double angularAcceleration = (angular - previousAngular) / dt;
        previousLinear = linear;
        previousAngular = angular;
        setChassisKinematics(linear, angular, linearAcceleration, angularAcceleration);
    }
}
