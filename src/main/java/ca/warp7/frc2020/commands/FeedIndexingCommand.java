/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.commands;

import ca.warp7.frc2020.Constants;
import ca.warp7.frc2020.subsystems.Feeder;
import ca.warp7.frc2020.subsystems.Hopper;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FeedIndexingCommand extends CommandBase {
    private Feeder feeder = Feeder.getInstance();
    private Hopper hopper = Hopper.getInstance();
    private boolean notVisible = true;

    private double visibleTimes = 0;
    private double prevT;
    private double time;
  public FeedIndexingCommand() {
      addRequirements(feeder, hopper);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    time = Timer.getFPGATimestamp();
    
    if(feeder.getBeamBreak()){
        feeder.setSpeed(Constants.kIndexSpeed);
    }

    if(!feeder.getBeamBreak()){
        prevT  = Timer.getFPGATimestamp();
        if(time - prevT <= 1.0){
            //This runs the outer 775 backwards, which moves the ball down
            SmartDashboard.putNumber("Time error", time - prevT);
            feeder.indexPowerCell();
        }
    } 

}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      feeder.setSpeed(0);
  }
}
