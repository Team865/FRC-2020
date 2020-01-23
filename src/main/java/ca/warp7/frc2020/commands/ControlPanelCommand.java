/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.commands;

import ca.warp7.frc2020.subsystems.ControlPanelSpinner;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class ControlPanelCommand extends CommandBase {

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

        // todo put the FMS control panel data on the network tables/shuffleboard
        if(gameData.length() > 0)
        {
            switch (gameData.charAt(0))
            {
                case 'B' :
                    //Blue case code
                    break;
                case 'G' :
                    //Green case code
                    break;
                case 'R' :
                    //Red case code
                    break;
                case 'Y' :
                    //Yellow case code
                    break;
                default :
                    //This is corrupt data
                    break;
            }
        } else {
            System.out.println("No Data Yet");
            //Code for no data received yet
        }

    }
}
