/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.auton.commands;

import ca.warp7.frc2020.Constants;
import ca.warp7.frc2020.subsystems.Feeder;
import ca.warp7.frc2020.subsystems.Flywheel;
import ca.warp7.frc2020.subsystems.Hopper;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootBallsCloseCommand extends CommandBase {
    private Flywheel flywheel = Flywheel.getInstance();
    private Feeder feeder = Feeder.getInstance();
    private Hopper hopper = Hopper.getInstance();
    private final int n;
    //private double prevT;
    //private double initT;
    private int cellsCount = 0;
    private boolean previousState;

    public ShootBallsCloseCommand(int n) {
        this.n = n;
        addRequirements(flywheel, feeder, hopper);
    }

    @Override
    public void initialize() {
        //initT = Timer.getFPGATimestamp();
        //prevT = Timer.getFPGATimestamp();
        previousState = feeder.getBeamBreak();

        flywheel.setHoodCloseShot(true);
    }

    @Override
    public void execute() {
        boolean currentState;
        SmartDashboard.putNumber("Cells shot", cellsCount);
        //double time = Timer.getFPGATimestamp();
        flywheel.setTargetRPS(Flywheel.getOptimalCloseShotRPS());
        flywheel.calcOutput();

        currentState = feeder.getBeamBreak();

        if (!previousState && currentState) {
            cellsCount++;
        }

        previousState = currentState;


        if (flywheel.isTargetReached(0.015)) {
            feeder.setSpeed(Constants.kFeedingSpeed);
            hopper.setSpeed(Constants.kHopperSpeed);
        } else {
            feeder.setSpeed(0);
            hopper.setSpeed(0);
        }
        //prevT = time;
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setTargetRPS(0);
        feeder.setSpeed(0);
        hopper.setSpeed(0);
        cellsCount = 0;
    }

    @Override
    public boolean isFinished() {
        return cellsCount >= n;
        //return prevT - initT > 1 + n; // TODO make not bad (detect how many balls have been shot)
    }
}
