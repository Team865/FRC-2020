/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.commands;

import ca.warp7.frc2020.subsystems.Flywheel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import java.util.function.DoubleSupplier;

import static ca.warp7.frc2020.Constants.*;

public class FlywheelSpeedCommand extends CommandBase {
    private DoubleSupplier wantedRPS;
    private Flywheel flywheel = Flywheel.getInstance();
    public double prev = 0.0;
    public double pt = 0.0;

    public FlywheelSpeedCommand(DoubleSupplier wantedRPS) {
        this.wantedRPS = wantedRPS;
        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        pt = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {

        double targetRPS;

        targetRPS = wantedRPS.getAsDouble();
        flywheel.setTargetRPS(targetRPS);
        flywheel.calcOutput();
        double currentRPS = flywheel.getRPS();
        double t = Timer.getFPGATimestamp();
        SmartDashboard.putNumber("Acceleration", (currentRPS - prev) / (t - pt));
        prev = currentRPS;
        pt = t;
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setVoltage(0);
    }
}
