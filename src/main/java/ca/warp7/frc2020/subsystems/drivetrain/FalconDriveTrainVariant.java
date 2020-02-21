package ca.warp7.frc2020.subsystems.drivetrain;

import ca.warp7.frc2020.lib.control.PID;
import ca.warp7.frc2020.lib.motor.MotorControlHelper;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import static ca.warp7.frc2020.Constants.*;

public final class FalconDriveTrainVariant implements DriveTrainVariant {

    private static final double kTicksPerRotation = 2048.0;
    private static final double kVelocityMeasurementPeriod = 0.100;

    public static final double kCurrentLimitAfterActivation = 35.0;
    public static final double kCurrentLimitTriggerThreshold = 40.0;
    public static final double kCurrentLimitTriggerTime = 1.0;
    public static final StatorCurrentLimitConfiguration kDriveStatorCurrentLimit =
            new StatorCurrentLimitConfiguration(true, kCurrentLimitAfterActivation,
                    kCurrentLimitTriggerThreshold, kCurrentLimitTriggerTime);

    private final TalonFX driveLeftMasterFalcon =
            MotorControlHelper.createMasterTalonFX(kDriveLeftMasterID);
    private final TalonFX driveLeftFollowerFalcon =
            MotorControlHelper.assignFollowerTalonFX(
                    driveLeftMasterFalcon,
                    kDriveLeftFollowerID,
                    InvertType.FollowMaster
            );

    private final TalonFX driveRightMasterFalcon =
            MotorControlHelper.createMasterTalonFX(kDriveRightMasterID);
    private final TalonFX driveRightFollowerFalcon =
            MotorControlHelper.assignFollowerTalonFX(
                    driveRightMasterFalcon,
                    kDriveRightFollowerID,
                    InvertType.FollowMaster
            );

    public FalconDriveTrainVariant() {
        driveRightMasterFalcon.setInverted(true);
        driveLeftMasterFalcon.configStatorCurrentLimit(kDriveStatorCurrentLimit);
        driveRightMasterFalcon.configStatorCurrentLimit(kDriveStatorCurrentLimit);
    }

    @Override
    public void setVelocityPID(
            double leftVelocityRotationsPerSecond,
            double rightVelocityRotationsPerSecond,
            double leftVoltage,
            double rightVoltage) {
        driveLeftMasterFalcon.set(
                ControlMode.Velocity,
                leftVelocityRotationsPerSecond *
                        kTicksPerRotation * kVelocityMeasurementPeriod,
                DemandType.ArbitraryFeedForward,
                leftVoltage / kMaxVoltage
        );
        driveRightMasterFalcon.set(
                ControlMode.Velocity,
                rightVelocityRotationsPerSecond *
                        kTicksPerRotation * kVelocityMeasurementPeriod,
                DemandType.ArbitraryFeedForward,
                rightVoltage / kMaxVoltage
        );
    }

    @Override
    public void setPositionPID(double leftDistanceRotations, double rightDistanceRotations) {
        driveLeftMasterFalcon.set(
                ControlMode.Position,
                leftDistanceRotations * kTicksPerRotation
        );
        driveRightMasterFalcon.set(
                ControlMode.Position,
                rightDistanceRotations * kTicksPerRotation
        );
    }

    @Override
    public void configurePID(PID pid) {
        MotorControlHelper.configurePID(driveLeftMasterFalcon, pid);
        MotorControlHelper.configurePID(driveRightMasterFalcon, pid);
    }

    @Override
    public void configureRampRate(double secondsFromNeutralToFull) {
        driveLeftMasterFalcon.configOpenloopRamp(secondsFromNeutralToFull);
        driveRightMasterFalcon.configOpenloopRamp(secondsFromNeutralToFull);
    }

    @Override
    public void setEncoderPosition(double leftRotations, double rightRotations) {
        driveLeftMasterFalcon
                .setSelectedSensorPosition((int) (leftRotations * kTicksPerRotation));
        driveRightMasterFalcon
                .setSelectedSensorPosition((int) (rightRotations * kTicksPerRotation));
    }

    /*
    https://phoenix-documentation.readthedocs.io/en/latest/ch13_MC.html#neutral-mode

    Follower motor controllers have separate neutral modes than their masters,
    so you must choose both. Additionally, you may want to mix your neutral modes
    to achieve a partial electric brake when using multiple motors.
     */
    private void setNeutralMode(NeutralMode mode) {
        driveLeftMasterFalcon.setNeutralMode(mode);
        driveLeftFollowerFalcon.setNeutralMode(mode);

        driveRightMasterFalcon.setNeutralMode(mode);
        driveRightFollowerFalcon.setNeutralMode(mode);
    }

    @Override
    public void setBrake() {
        setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void setCoast() {
        setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public double getLeftPositionRotations() {
        return driveLeftMasterFalcon.getSelectedSensorPosition() / kTicksPerRotation;
    }

    @Override
    public double getRightPositionRotations() {
        return driveRightMasterFalcon.getSelectedSensorPosition() / kTicksPerRotation;
    }

    @Override
    public double getLeftVelocityRPS() {
        return driveLeftMasterFalcon.getSelectedSensorVelocity() /
                kTicksPerRotation / kVelocityMeasurementPeriod;
    }

    @Override
    public double getRightVelocityRPS() {
        return driveRightMasterFalcon.getSelectedSensorVelocity() /
                kTicksPerRotation / kVelocityMeasurementPeriod;
    }

    @Override
    public double getLeftVoltage() {
        return driveLeftMasterFalcon.getMotorOutputVoltage();
    }

    @Override
    public double getRightVoltage() {
        return driveRightMasterFalcon.getMotorOutputVoltage();
    }

    @Override
    public void neutralOutput() {
        driveLeftMasterFalcon.neutralOutput();
        driveRightMasterFalcon.neutralOutput();
    }

    @Override
    public void setPercentOutput(double leftPercent, double rightPercent) {
        driveLeftMasterFalcon.set(ControlMode.PercentOutput, leftPercent);
        driveRightMasterFalcon.set(ControlMode.PercentOutput, rightPercent);
    }

    @Override
    public double getLeftPIDErrorRotations() {
        var error = driveLeftMasterFalcon.getClosedLoopError() / kTicksPerRotation;
        switch (driveLeftMasterFalcon.getControlMode()) {
            case Position:
                return error;
            case Velocity:
                return error / kVelocityMeasurementPeriod;
            default:
                throw new UnsupportedOperationException();
        }
    }

    @Override
    public double getRightPIDErrorRotations() {
        var error = driveRightMasterFalcon.getClosedLoopError() / kTicksPerRotation;
        switch (driveRightMasterFalcon.getControlMode()) {
            case Position:
                return error;
            case Velocity:
                return error / kVelocityMeasurementPeriod;
            default:
                throw new UnsupportedOperationException();
        }
    }
}
