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
    
    private double visibleTimes = 0;
    private double prevT;
    private double time;
    private double indexTime = 1.0;
  public FeedIndexingCommand() {
      addRequirements(feeder, hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    visibleTimes = 0;
    prevT  = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean notVisible = true;
    time = Timer.getFPGATimestamp();

    SmartDashboard.putNumber("Visible times", visibleTimes);
    SmartDashboard.putNumber("Time error", time - prevT);
    
    if(!feeder.getBeamBreak()){
        visibleTimes++;
        notVisible = false;
    } 

    if(notVisible){
        feeder.setSpeed(Constants.kIndexSpeed);
    }

    if(notVisible == false && time - prevT <= indexTime){
        feeder.indexPowerCell();;
    } else if (time - prevT > indexTime){
        notVisible = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      feeder.setSpeed(0);
  }
}
