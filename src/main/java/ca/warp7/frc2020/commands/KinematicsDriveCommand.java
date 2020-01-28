/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.commands;

import ca.warp7.frc2020.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static ca.warp7.frc2020.Constants.kWheelBaseRadius;

public class KinematicsDriveCommand extends CommandBase {

    private DriveTrain driveTrain = DriveTrain.getInstance();
    private DoubleSupplier xSpeedSupplier;
    private DoubleSupplier zRotationSupplier;
    private BooleanSupplier quickTurnSupplier;

    public KinematicsDriveCommand(
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

    @Override
    public void execute() {

        double xSpeed = xSpeedSupplier.getAsDouble();
        boolean isQuickTurn = quickTurnSupplier.getAsBoolean();

        // We multiply by -1 because left is positive in kinematic calculations
        double zRotation = -1 * zRotationSupplier.getAsDouble();

        double maxVelocity = getMaximumVelocity();
        double linear;
        double angular;

        if (isQuickTurn) {
            
            // Since the angular velocity is the independent variable,
            // linear must accommodate zRotation first by ensuring
            // that even if xSpeed is 1 or -1, the left and right
            // wheel speeds never exceeds maxVelocity given zRotation

            linear = xSpeed * maxVelocity * (1 - Math.abs(zRotation));

            // We want zRotation to map to the maximum angular velocity,
            // which is equal to maxVelocity / kWheelBaseRadius

            angular = zRotation * maxVelocity / kWheelBaseRadius;
        } else {

            // The linear velocity is the independent variable here.
            //
            // We first find that the magnitude of the desired curvature 
            // lies between 0 (going straight) and 1 / kWheelBaseRadius
            // (the maximum curvature that the robot can drive in before 
            // one side has to go in the opposite direction).
            //
            // We then map this magnitude to zRotation to get the curvature:
            // curvature = zRotation * (1 / kWheelBaseRadius)
            //
            // Now we find the maximum linear velocity that the robot can
            // reach given the curvature, which is:
            // maxVelocity / (1 + abs(curvature) * kWheelbaseRadius).
            //
            // Substituting curvature with the equation above, we get:
            // maxVelocity / (1 + abs(zRotation * (1 / kWheelBaseRadius)) * kWheelbaseRadius)
            // 
            // Since kWheelBaseRadius is positive, we can move it ouside
            // the absolute value and cancel it out
            //
            // Finally we map xSpeed between 0 and the maximum possible velocity

            linear = xSpeed * maxVelocity / (1 + Math.abs(zRotation));

            // Angular velocity is just abs(linear) * curvature. We can
            // substitute curvature with the equation above to calculate
            // the final angular velocity.

            angular = zRotation * Math.abs(linear) / kWheelBaseRadius;
        }

        driveTrain.setChassisVelocity(linear, angular);
    }


    private double getMaximumVelocity() {
        return driveTrain.isHighGear() ? 4.6 : 2.2;
    }
}
