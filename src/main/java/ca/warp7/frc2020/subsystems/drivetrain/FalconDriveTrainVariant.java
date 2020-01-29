package ca.warp7.frc2020.subsystems.drivetrain;

import ca.warp7.frc2020.lib.control.PID;
import ca.warp7.frc2020.lib.motor.MotorControlHelper;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import static ca.warp7.frc2020.Constants.*;

public final class FalconDriveTrainVariant implements DriveTrainVariant {

    private final TalonFX driveLeftMasterFalcon = MotorControlHelper.createMasterTalonFX(kDriveLeftMasterID);
    private final TalonFX driveRightMasterFalcon = MotorControlHelper.createMasterTalonFX(kDriveRightMasterID);

    public FalconDriveTrainVariant() {
        driveRightMasterFalcon.setInverted(true);
        driveLeftMasterFalcon.configStatorCurrentLimit(kDriveStatorCurrentLimit, 50);
        driveRightMasterFalcon.configStatorCurrentLimit(kDriveStatorCurrentLimit, 50);
        MotorControlHelper.assignFollowerTalonFX(driveLeftMasterFalcon, kDriveLeftFollowerID);
        MotorControlHelper.assignFollowerTalonFX(driveRightMasterFalcon, kDriveRightFollowerID);
    }

    @Override
    public void setVelocityPID(
            double leftVelocityRotationsPerSecond,
            double rightVelocityRotationsPerSecond,
            double leftVoltage,
            double rightVoltage) {
        driveLeftMasterFalcon.set(
                ControlMode.Velocity,
                leftVelocityRotationsPerSecond * 2048 / 10.0,
                DemandType.ArbitraryFeedForward,
                leftVoltage / kMaxVoltage
        );
        driveRightMasterFalcon.set(
                ControlMode.Velocity,
                rightVelocityRotationsPerSecond * 2048 / 10.0,
                DemandType.ArbitraryFeedForward,
                rightVoltage / kMaxVoltage
        );
    }

    @Override
    public void setPositionPID(double leftDistanceRotations, double rightDistanceRotations) {
        driveLeftMasterFalcon.set(
                ControlMode.Position,
                leftDistanceRotations * 2048
        );
        driveRightMasterFalcon.set(
                ControlMode.Position,
                rightDistanceRotations * 2048
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
        driveLeftMasterFalcon.setSelectedSensorPosition((int) (leftRotations * 2048));
        driveRightMasterFalcon.setSelectedSensorPosition((int) (rightRotations * 2048));
    }

    @Override
    public double getLeftPositionRotations() {
        return driveLeftMasterFalcon.getSelectedSensorPosition() / 2048.0;
    }

    @Override
    public double getRightPositionRotations() {
        return driveRightMasterFalcon.getSelectedSensorPosition() / 2048.0;
    }

    @Override
    public double getLeftVelocityRotationsPerSecond() {
        return driveLeftMasterFalcon.getSelectedSensorVelocity() / 2048.0 * 10;
    }

    @Override
    public double getRightVelocityRotationsPerSecond() {
        return driveRightMasterFalcon.getSelectedSensorVelocity() / 2048.0 * 10;
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
    public void setVoltage(double leftVoltage, double rightVoltage) {
        setPercentOutput(leftVoltage / kMaxVoltage, rightVoltage / kMaxVoltage);
    }

    // Current Limiting

    public static final double kCurrentLimitAfterActivation = 35.0;
    public static final double kCurrentLimitTriggerThreshold = 40.0;
    public static final double kCurrentLimitTriggerTime = 1.0;
    public static final StatorCurrentLimitConfiguration kDriveStatorCurrentLimit =
            new StatorCurrentLimitConfiguration(true, kCurrentLimitAfterActivation,
                    kCurrentLimitTriggerThreshold, kCurrentLimitTriggerTime);
}
