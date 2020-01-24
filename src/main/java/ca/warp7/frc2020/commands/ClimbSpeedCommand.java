/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.commands;

import ca.warp7.frc2020.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class ClimbSpeedCommand extends CommandBase {
    private DoubleSupplier speedSupplier;
    private Climber climber = Climber.getInstance();

    public ClimbSpeedCommand(DoubleSupplier speedSupplier) {
        this.speedSupplier = speedSupplier;
    }

    @Override
    public void execute() {
        climber.set(speedSupplier.getAsDouble());
    }
}
