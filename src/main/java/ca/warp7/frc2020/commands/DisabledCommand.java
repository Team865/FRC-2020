/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.commands;

import ca.warp7.frc2020.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DisabledCommand extends CommandBase {

    private DriveTrain driveTrain = DriveTrain.getInstance();
    
    @Override
    public void initialize() {
        driveTrain.setCoast();
        SmartDashboard.putNumber("rps", 0.0);
        SmartDashboard.putNumber("target", 0.0);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
