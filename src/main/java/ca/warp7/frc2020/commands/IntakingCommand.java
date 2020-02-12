/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.commands;

import ca.warp7.frc2020.Constants;
import ca.warp7.frc2020.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class IntakingCommand extends CommandBase {
    private DoubleSupplier intakingSupplier;
    private Intake intake = Intake.getInstance();
    private double pTime = 0.0;

    public IntakingCommand(DoubleSupplier intakingSupplier) {
        this.intakingSupplier = intakingSupplier;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        pTime = Double.POSITIVE_INFINITY;
    }

    @Override
    public void execute() {
        double time = Timer.getFPGATimestamp();
        double intakeSpeed = intakingSupplier.getAsDouble();
        boolean intaking = intakeSpeed != 0.0;
        intake.setExtended(intaking);
        if (intakeSpeed != 0) {
            intake.setSpeed(0.75 * intakeSpeed);
            pTime = time;
        } else if (time - pTime < 1)
            intake.setSpeed(0.55);
    }
}
