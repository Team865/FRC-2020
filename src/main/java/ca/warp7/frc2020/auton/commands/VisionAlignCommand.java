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

public class VisionAlignCommand extends CommandBase {
    private DriveTrain driveTrain = DriveTrain.getInstance();
    private Limelight limelight = Limelight.getInstance();
    private double prevT;
    private double prevAngle;

    private DoubleSupplier forwardSpeedSupplier;
    private PIDController pidController = new PIDController(Constants.kVisionAlignmentYawPID);

    public VisionAlignCommand(DoubleSupplier forwardSpeedSupplier) {
        this.forwardSpeedSupplier = forwardSpeedSupplier;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        prevT = Timer.getFPGATimestamp();
        prevAngle = driveTrain.getContinousAngleRadians();
    }

    @Override
    public void execute() {
        double speed = forwardSpeedSupplier.getAsDouble();
        double t = Timer.getFPGATimestamp();
        double angle = driveTrain.getContinousAngleRadians() * 180 / Math.PI;

        if (!limelight.hasValidTarget()) {
            driveTrain.setPercentOutput(speed, speed);
        } else {
            double angularVelocity = (prevAngle - angle) / (prevT - t);
            double latency = limelight.getLatencySeconds();

            double adjustedHorizontalAngle = limelight.getHorizontalAngle() + -1 * angularVelocity * latency;
            double ff = driveTrain.getTransmission().ks / kMaxVoltage;

            double correction = pidController.calculate(0, adjustedHorizontalAngle);
            double left = speed - correction;
            double right = speed + correction;

            driveTrain.setPercentOutput(
                    Math.copySign(ff, left) + left,
                    Math.copySign(ff, right) + right
            );
        }

        prevT = t;
    }
}
