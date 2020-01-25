/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.commands;

import ca.warp7.frc2020.Constants;
import ca.warp7.frc2020.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;

public class IntakingCommand extends CommandBase {
    private BooleanSupplier intakingSupplier;
    private Intake intake = Intake.getInstance();

    public IntakingCommand(BooleanSupplier intakingSupplier) {
        this.intakingSupplier = intakingSupplier;
    }

    @Override
    public void execute() {
        boolean intaking = intakingSupplier.getAsBoolean();
        intake.setExtended(intaking);
        if (intaking) 
            intake.setSpeed(Constants.intakingSpeed);
        else
            intake.setSpeed(0.0);
    }
}
