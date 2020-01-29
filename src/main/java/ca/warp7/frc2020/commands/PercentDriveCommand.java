/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.commands;

import ca.warp7.frc2020.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class PercentDriveCommand extends CommandBase {

    private DriveTrain driveTrain = DriveTrain.getInstance();
    private DoubleSupplier xSpeedSupplier;
    private DoubleSupplier zRotationSupplier;
    private BooleanSupplier quickTurnSupplier;

    public PercentDriveCommand(
            DoubleSupplier xSpeedSupplier,
            DoubleSupplier zRotationSupplier,
            BooleanSupplier quickTurnSupplier) {
        this.xSpeedSupplier = xSpeedSupplier;
        this.zRotationSupplier = zRotationSupplier;
        this.quickTurnSupplier = quickTurnSupplier;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        // make sure we don't miss a loop before the next scheduler run
        execute();
    }

    private static final double m_quickStopThreshold = 0.2;
    private static final double m_quickStopAlpha = 0.1;
    private double m_quickStopAccumulator;

    @Override
    public void execute() {

        double xSpeed = xSpeedSupplier.getAsDouble();
        double zRotation = zRotationSupplier.getAsDouble();
        boolean isQuickTurn = quickTurnSupplier.getAsBoolean();

        /*----------------------------------------------------------------------------*/
        /* Copyright (c) 2008-2019 FIRST. All Rights Reserved.                        */
        /* Open Source Software - may be modified and shared by FRC teams. The code   */
        /* must be accompanied by the FIRST BSD license file in the root directory of */
        /* the project.                                                               */
        /*----------------------------------------------------------------------------*/

        double angularPower;
        boolean overPower;

        if (isQuickTurn) {
            if (Math.abs(xSpeed) < m_quickStopThreshold) {
                m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator
                        + m_quickStopAlpha * MathUtil.clamp(zRotation, -1.0, 1.0) * 2;
            }
            overPower = true;
            angularPower = zRotation;
        } else {
            overPower = false;
            angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

            if (m_quickStopAccumulator > 1) {
                m_quickStopAccumulator -= 1;
            } else if (m_quickStopAccumulator < -1) {
                m_quickStopAccumulator += 1;
            } else {
                m_quickStopAccumulator = 0.0;
            }
        }

        double leftMotorOutput = xSpeed + angularPower;
        double rightMotorOutput = xSpeed - angularPower;

        // If rotation is overpowered, reduce both outputs to within acceptable range
        if (overPower) {
            if (leftMotorOutput > 1.0) {
                rightMotorOutput -= leftMotorOutput - 1.0;
                leftMotorOutput = 1.0;
            } else if (rightMotorOutput > 1.0) {
                leftMotorOutput -= rightMotorOutput - 1.0;
                rightMotorOutput = 1.0;
            } else if (leftMotorOutput < -1.0) {
                rightMotorOutput -= leftMotorOutput + 1.0;
                leftMotorOutput = -1.0;
            } else if (rightMotorOutput < -1.0) {
                leftMotorOutput -= rightMotorOutput + 1.0;
                rightMotorOutput = -1.0;
            }
        }

        // Normalize the wheel speeds
        double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
        if (maxMagnitude > 1.0) {
            leftMotorOutput /= maxMagnitude;
            rightMotorOutput /= maxMagnitude;
        }

        driveTrain.setPercentOutput(leftMotorOutput, rightMotorOutput);
    }
}
