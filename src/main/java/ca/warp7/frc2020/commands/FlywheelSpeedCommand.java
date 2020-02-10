/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.commands;

import ca.warp7.frc2020.subsystems.Flywheel;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import java.util.function.DoubleSupplier;

import static ca.warp7.frc2020.Constants.*;

public class FlywheelSpeedCommand extends CommandBase {
    private DoubleSupplier wantedFarShotRPS;
    private Flywheel flywheel = Flywheel.getInstance();

    public FlywheelSpeedCommand(DoubleSupplier wantedFarShotRPS) {
        this.wantedFarShotRPS = wantedFarShotRPS;
        addRequirements(flywheel);
    }

    @Override
    public void execute() {

        double targetRPS;
//        if (flywheel.getHood()) {
//            targetRPM = wantedFarShotRPM.getAsInt();
//        } else  {
//            targetRPM = 3000;
//        }

        targetRPS = wantedFarShotRPS.getAsDouble();
        double currentRPS = flywheel.getRotationsPerSecond();

        double voltage = (targetRPS + (targetRPS - currentRPS) * kFlywheelKp)
                * kFlywheelKv + kFlywheelKs;

        flywheel.setVoltage(MathUtil.clamp(voltage, 0, 12.0));
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setVoltage(0);
    }
}
