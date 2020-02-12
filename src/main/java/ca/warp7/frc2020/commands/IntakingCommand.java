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

import java.util.function.DoubleSupplier;

public class IntakingCommand extends CommandBase {
    private DoubleSupplier intakingSupplier;
    private Intake intake = Intake.getInstance();

    public IntakingCommand(DoubleSupplier intakingSupplier) {
        this.intakingSupplier = intakingSupplier;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        double intakeSpeed = intakingSupplier.getAsDouble();
        boolean intaking = intakeSpeed != 0.0;
        intake.setExtended(intaking);
        intake.setSpeed(Constants.intakingSpeed);
    }
}
