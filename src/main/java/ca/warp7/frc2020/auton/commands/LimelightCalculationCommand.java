/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.auton.commands;

import ca.warp7.frc2020.Constants;
import ca.warp7.frc2020.auton.vision.Limelight;
import ca.warp7.frc2020.lib.control.PIDController;
import ca.warp7.frc2020.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

import static ca.warp7.frc2020.Constants.kMaxVoltage;

public class LimelightCalculationCommand extends CommandBase {
    private DriveTrain driveTrain = DriveTrain.getInstance();
    private Limelight limelight = Limelight.getInstance();
    private double prevT;
    private double prevAngle;

    @Override
    public void initialize() {
        prevT = Timer.getFPGATimestamp();
        prevAngle = driveTrain.getContinousAngleRadians() * 180 / Math.PI;
    }

    @Override
    public void execute() {
        double t = Timer.getFPGATimestamp();
        double angle = driveTrain.getContinousAngleRadians() * 180 / Math.PI;

        Double smoothTargetAngle = limelight.getSmoothHorizontalAngle();

        if (!limelight.hasValidTarget()) {
            smoothTargetAngle = null;
        } else {
            double angleChange = -1 * (prevAngle - angle);
            double angularVelocity = angleChange / (prevT - t); // TODO div/0?
            double latency = limelight.getLatencySeconds();

            double targetAngle = limelight.getHorizontalAngle() + angularVelocity * latency;
            if (smoothTargetAngle != null) {

                smoothTargetAngle += angleChange;

                double smoothing = 0.9;
                smoothTargetAngle = smoothTargetAngle * smoothing + targetAngle * (1 - smoothing);
            } else
                smoothTargetAngle = targetAngle;
        }
        limelight.setSmoothHorizontalAngle(smoothTargetAngle);

        prevT = t;
        prevAngle = angle;
    }
}
