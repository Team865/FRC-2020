/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.commands;

import ca.warp7.frc2020.subsystems.ControlPanelSpinner;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import ca.warp7.frc2020.subsystems.ControlPanelSpinner.SpinnerColour;

import java.util.function.DoubleSupplier;

public class ControlPanelCommand extends CommandBase {
    
    private int colourCount = 0;
    private SpinnerColour currentColour = SpinnerColour.UNKNOWN;
    private SpinnerColour nextColour = SpinnerColour.UNKNOWN;
    
    private ControlPanelSpinner spinner = ControlPanelSpinner.getInstance();
    private DoubleSupplier speedSupplier;
    
    public ControlPanelCommand(DoubleSupplier speedSupplier) {
        this.speedSupplier = speedSupplier;
        addRequirements(spinner);
    }
    
    @Override
    public void execute() {
        
        spinner.setSpeed(speedSupplier.getAsDouble());
        String gameData = DriverStation.getInstance().getGameSpecificMessage();
        
        // ===PRINTING INFO ONTO THE DASHBOARD FOR THE DRIVER===
        
        // todo put the FMS control panel data on the network tables/shuffleboard
        if(gameData.length() > 0)
        {
            //The colour we have to land on is gameData.charAt(0)
            switch (gameData.charAt(0))
            {
                case 'B' :
                SmartDashboard.putString("targetColour", "Blue");
                break;
                case 'G' :
                SmartDashboard.putString("targetColour", "Green");
                break;
                case 'R' :
                SmartDashboard.putString("targetColour", "Red");
                break;
                case 'Y' :
                SmartDashboard.putString("targetColour", "Yellow");
                break;
                default :
                SmartDashboard.putString("targetColour", "Data Corrupted");
                break;
            }
        } else {
            SmartDashboard.putString("targetColour", "No Data");
            //Code for no data received yet
        }
        
        SpinnerColour colourSensorReading = spinner.getCurrentSpinnerColor();
        
        //If the colour sensed is difrent from the current colour, wait 5 cycles. If it stayed the same
        //change the current colour to the senced colour.
        if(currentColour != colourSensorReading){
            if(colourSensorReading == nextColour && colourCount <= 5){
                currentColour = nextColour;
            }
            else if(colourSensorReading == nextColour){
                colourCount++;
            }
            else{
                colourCount = 0;
                nextColour = colourSensorReading;
            }
        }

        SmartDashboard.putString("spinner", currentColour.toString());        
    }
}
