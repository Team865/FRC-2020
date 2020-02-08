/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.subsystems;

import ca.warp7.frc2020.Constants;
import ca.warp7.frc2020.lib.motor.MotorControlHelper;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class Hopper implements Subsystem {
    private static Hopper instance;

    public static Hopper getInstance() {
        if (instance == null) instance = new Hopper();
        return instance;
    }

    private CANSparkMax hopperMiniNeo = MotorControlHelper
            .createMasterSparkMAX(Constants.kHopperID);

    private Hopper() {
        hopperMiniNeo.setInverted(true);
    }

    public void setSpeed(double speed) {
        hopperMiniNeo.set(speed);
    }
}
