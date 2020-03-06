/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.auton.commands;

import ca.warp7.frc2020.subsystems.Feeder;
import ca.warp7.frc2020.subsystems.Flywheel;
import ca.warp7.frc2020.subsystems.Hopper;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static ca.warp7.frc2020.Constants.*;

public class FeedAutoCommand extends CommandBase {
    private Flywheel flywheel = Flywheel.getInstance();
    private Feeder feeder = Feeder.getInstance();
    private Hopper hopper = Hopper.getInstance();

    public FeedAutoCommand() {
        addRequirements(feeder, hopper);
    }

    @Override
    public void execute() {
        double currentFlywheelSpeed = flywheel.getRPS();
        double error = kFlywheelDefaultCloseRPS - currentFlywheelSpeed;
        SmartDashboard.putNumber("Velocity", currentFlywheelSpeed);
        SmartDashboard.putNumber("Error", error);
        if (Math.abs(error) <= -1) {
            feeder.setSpeed(kFeedingSpeed);
            hopper.setSpeed(kHopperSpeed);
        } else {
            feeder.setSpeed(0);
            hopper.setSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
