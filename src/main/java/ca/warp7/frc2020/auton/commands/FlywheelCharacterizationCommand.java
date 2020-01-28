/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.auton.commands;

import ca.warp7.frc2020.lib.LinearRegression;
import ca.warp7.frc2020.subsystems.Flywheel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FlywheelCharacterizationCommand extends CommandBase {

    private static final int kMeasurements = 50;
    private static final double kMaxVoltage = 12.0;
    private static final double kInterval = kMaxVoltage / kMeasurements;
    private static final double kTimePerIncrement = 0.5; // s

    private Flywheel flywheel = Flywheel.getInstance();
    private double[] voltages = new double[kMeasurements];
    private double[] velocities = new double[kMeasurements];
    private LinearRegression regression = new LinearRegression(voltages, velocities);
    private int index = 0;
    private double prevTime = 0.0;

    public FlywheelCharacterizationCommand() {
        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        prevTime = Timer.getFPGATimestamp();
        index = 0;
    }

    @Override
    public void execute() {

        double voltage = index * kInterval;
        flywheel.setVoltage(voltage);
        double curTime = Timer.getFPGATimestamp();

        if (curTime - prevTime >= kTimePerIncrement) {
            prevTime = curTime;
            voltages[index] = voltage;
            velocities[index] = flywheel.getRPM();
            regression.calculate(index);
            System.out.println(String.format("Ks=%.5f Kv=%.5f",
                    regression.intercept(), regression.slope()));
            index++;
        }

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return index >= kMeasurements;
    }
}
