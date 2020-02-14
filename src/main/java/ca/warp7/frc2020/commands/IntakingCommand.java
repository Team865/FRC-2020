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
    private double pTime = -1.0;

    public IntakingCommand(DoubleSupplier intakingSupplier) {
        this.intakingSupplier = intakingSupplier;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        pTime = -1.0;
    }

    @Override
    public void execute() {
        double time = Timer.getFPGATimestamp();
        double intakeSpeed = intakingSupplier.getAsDouble();
        boolean intaking = intakeSpeed != 0.0;
        intake.setExtended(intaking);
        if (intaking) {
            intake.setSpeed(0.75 * intakeSpeed);
            pTime = time;
        } else if (time - pTime < 0.75 && pTime >= 0)
            intake.setSpeed(0.55);
        else
            intake.setSpeed(0.0);
    }
}
