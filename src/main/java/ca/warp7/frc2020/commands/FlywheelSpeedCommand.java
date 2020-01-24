/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.commands;

import ca.warp7.frc2020.Constants;
import ca.warp7.frc2020.subsystems.Flywheel;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.IntSupplier;

public class FlywheelSpeedCommand extends CommandBase {
    private IntSupplier wantedFarShotRPM;
    private Flywheel flywheel = Flywheel.getInstance();

    public FlywheelSpeedCommand(IntSupplier wantedFarShotRPM) {
        this.wantedFarShotRPM = wantedFarShotRPM;
    }

    @Override
    public void execute() {

        double targetRPM;
        if (flywheel.getHood()) {
            targetRPM = wantedFarShotRPM.getAsInt();
        } else  {
            targetRPM = 3000;
        }
        double currentRPM = flywheel.getRPM();

        double newRPM;
        // do a P controller if target is above current, otherwise just set to targetRPM
        // (so that kP does not overshoot going down)
        if (targetRPM > currentRPM) {
            newRPM = Math.min(targetRPM, currentRPM + (targetRPM - currentRPM) * Constants.kFlywheelSpeedKp);
        } else {
            newRPM = targetRPM;
        }

        flywheel.setRPM((int) newRPM);
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setRPM(0);
    }
}
