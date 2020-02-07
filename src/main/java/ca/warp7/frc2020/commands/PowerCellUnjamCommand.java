/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.commands;

import ca.warp7.frc2020.subsystems.Feeder;
import ca.warp7.frc2020.subsystems.Hopper;
import ca.warp7.frc2020.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class PowerCellUnjamCommand extends CommandBase {
    private Feeder feeder = Feeder.getInstance();
    private Hopper hopper = Hopper.getInstance();
    private Intake intake = Intake.getInstance();

    private DoubleSupplier speedSupplier;

    public PowerCellUnjamCommand(DoubleSupplier speedSupplier) {
        this.speedSupplier = speedSupplier;
        addRequirements(feeder, hopper, intake);
    }

    @Override
    public void execute() {
        if (intake.isExtended()) {
            double speed = speedSupplier.getAsDouble();
            feeder.setInnerSpeed(-speed);
            feeder.setOuterSpeed(-speed);
            hopper.setSpeed(-speed);
            intake.setSpeed(-speed);
        }
    }
}
