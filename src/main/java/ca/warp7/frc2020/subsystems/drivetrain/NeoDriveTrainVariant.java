package ca.warp7.frc2020.subsystems.drivetrain;

import ca.warp7.frc2020.lib.control.PID;
import ca.warp7.frc2020.lib.motor.MotorControlHelper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import static ca.warp7.frc2020.Constants.*;

public final class NeoDriveTrainVariant implements DriveTrainVariant {

    private CANSparkMax driveLeftMasterNeo = MotorControlHelper
            .createMasterSparkMAX(kDriveLeftMasterID);
    private CANSparkMax driveRightMasterNeo = MotorControlHelper
            .createMasterSparkMAX(kDriveRightMasterID);

    public NeoDriveTrainVariant() {
        driveRightMasterNeo.setInverted(true);
        MotorControlHelper.assignFollowerSparkMAX(driveLeftMasterNeo, kDriveLeftFollowerID);
        MotorControlHelper.assignFollowerSparkMAX(driveRightMasterNeo, kDriveRightFollowerID);
    }

    @Override
    public void setVelocityPID(
            double leftVelocityRotationsPerSecond,
            double rightVelocityRotationsPerSecond,
            double leftVoltage,
            double rightVoltage) {
        driveLeftMasterNeo
                .getPIDController()
                .setReference(leftVelocityRotationsPerSecond * 60, ControlType.kVelocity,
                        0, leftVoltage);
        driveRightMasterNeo
                .getPIDController()
                .setReference(rightVelocityRotationsPerSecond * 60, ControlType.kVelocity,
                        0, rightVoltage);
    }

    @Override
    public void setPositionPID(double leftDistanceRotations, double rightDistanceRotations) {
        driveLeftMasterNeo.getPIDController()
                .setReference(leftDistanceRotations, ControlType.kPosition);
        driveRightMasterNeo.getPIDController()
                .setReference(rightDistanceRotations, ControlType.kPosition);
    }

    @Override
    public void configurePID(PID pid) {
        MotorControlHelper.configurePID(driveLeftMasterNeo, pid);
        MotorControlHelper.configurePID(driveRightMasterNeo, pid);
    }

    @Override
    public void configureRampRate(double secondsFromNeutralToFull) {
        driveLeftMasterNeo.setOpenLoopRampRate(secondsFromNeutralToFull);
        driveRightMasterNeo.setOpenLoopRampRate(secondsFromNeutralToFull);
    }


    double previousLeft = 0;
    double previousRight = 0;

    @Override
    public void setEncoderPosition(double leftRotations, double rightRotations) {
        previousLeft = leftRotations;
        previousRight = rightRotations;
        driveLeftMasterNeo.getEncoder().setPosition(leftRotations);
        driveRightMasterNeo.getEncoder().setPosition(rightRotations);
    }


    @Override
    public double getLeftDeltaRotation() {
        double left = driveLeftMasterNeo.getEncoder().getPosition();
        double delta = left - previousLeft;
        previousLeft = left;
        return delta;
    }

    @Override
    public double getRightDeltaRotation() {
        double right = driveRightMasterNeo.getEncoder().getPosition();
        double delta = right - previousRight;
        previousRight = right;
        return delta;
    }

    @Override
    public void neutralOutput() {
        driveLeftMasterNeo.stopMotor();
        driveRightMasterNeo.stopMotor();
    }

    @Override
    public void setPercentOutput(double leftPercent, double rightPercent) {
        driveLeftMasterNeo.set(leftPercent);
        driveRightMasterNeo.set(rightPercent);
    }

    @Override
    public void setVoltage(double leftVoltage, double rightVoltage) {
        driveLeftMasterNeo.setVoltage(leftVoltage);
        driveRightMasterNeo.setVoltage(rightVoltage);
    }
}
