/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.commands;

import ca.warp7.frc2020.Constants;
import ca.warp7.frc2020.lib.control.MinTimeBoolean;
import ca.warp7.frc2020.subsystems.Elevator;
import ca.warp7.frc2020.subsystems.Hopper;
import ca.warp7.frc2020.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class PowerCellFeedCommand extends CommandBase {
    private Elevator elevator = Elevator.getInstance();
    private Hopper hopper = Hopper.getInstance();
    private MinTimeBoolean flywheelMinTime = new MinTimeBoolean(Constants.kFlywheelFeedInterval);

    private DoubleSupplier speedSupplier;

    public PowerCellFeedCommand(DoubleSupplier speedSupplier) {
        this.speedSupplier = speedSupplier;
        addRequirements(elevator, hopper);
    }

    @Override
    public void execute() {
        double speed = speedSupplier.getAsDouble();
        boolean photoSensorTriggered = elevator.getPhotoSensor();

        if (flywheelMinTime.update(photoSensorTriggered, Timer.getFPGATimestamp())) {
            elevator.setOuterSpeed(speed);
            elevator.setInnerSpeed(speed);
            hopper.setSpeed(speed);
        } else {
            elevator.setOuterSpeed(0.0);
            elevator.setInnerSpeed(0.0);
            hopper.setSpeed(speed / 2.0);
        }
    }
}
