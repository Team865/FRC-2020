/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.commands;

import ca.warp7.frc2020.lib.Util;
import ca.warp7.frc2020.subsystems.Feeder;
import ca.warp7.frc2020.subsystems.Flywheel;
import ca.warp7.frc2020.subsystems.Hopper;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class FeedCommand extends CommandBase {
    private Feeder feeder = Feeder.getInstance();
    private Hopper hopper = Hopper.getInstance();
    private Flywheel flywheel = Flywheel.getInstance();

    private DoubleSupplier speedSupplier;

    public FeedCommand(DoubleSupplier speedSupplier) {
        this.speedSupplier = speedSupplier;
        addRequirements(feeder, hopper);
    }

    @Override
    public void execute() {
        double speed = speedSupplier.getAsDouble();
        if (speed < 0) {
            feeder.setSpeed(speed);
            hopper.setSpeed(speed);
        } else if (speed > 0 && Util.epsilonEquals(flywheel.getPercentError(), 0.0, 0.015) || feeder.getPhotoSensor()) {
            feeder.setSpeed(0.4);
            hopper.setSpeed(0.5);
        } else {
            feeder.setSpeed(0.0);
            hopper.setSpeed(0.0);
        }
    }
}