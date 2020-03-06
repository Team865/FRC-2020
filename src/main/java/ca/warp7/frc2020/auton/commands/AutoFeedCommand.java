/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.auton.commands;

import ca.warp7.frc2020.Constants;
import ca.warp7.frc2020.subsystems.Feeder;
import ca.warp7.frc2020.subsystems.Flywheel;
import ca.warp7.frc2020.subsystems.Hopper;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class AutoFeedCommand extends CommandBase {
    private Feeder feeder = Feeder.getInstance();
    private Hopper hopper = Hopper.getInstance();
    private Flywheel flywheel = Flywheel.getInstance();

    private BooleanSupplier shootingSupplier;

    public AutoFeedCommand(BooleanSupplier shootingSupplier) {
        this.shootingSupplier = shootingSupplier;
        addRequirements(feeder, hopper);
    }

    @Override
    public void execute() {
        boolean shooting = shootingSupplier.getAsBoolean();
        if ((shooting && flywheel.isTargetReached(0.015)) || feeder.getBeamBreak()) {
            feeder.setSpeed(Constants.kFeedingSpeed);
            hopper.setSpeed(Constants.kHopperSpeed);
        } else {
            feeder.setSpeed(0.0);
            hopper.setSpeed(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        feeder.setSpeed(0.0);
        hopper.setSpeed(0.0);
    }
}