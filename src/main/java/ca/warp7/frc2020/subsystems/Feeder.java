/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.subsystems;

import ca.warp7.frc2020.Constants;
import ca.warp7.frc2020.lib.motor.MotorControlHelper;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static ca.warp7.frc2020.Constants.kPhotoSensorID;

public final class Feeder implements Subsystem {
    private static Feeder instance;

    public static Feeder getInstance() {
        if (instance == null) instance = new Feeder();
        return instance;
    }

    private DigitalInput photoSensor = new DigitalInput(kPhotoSensorID);

    private VictorSPX feederOuter775 = MotorControlHelper.createMasterVictorSPX(Constants.kFeederOuterID);
    private VictorSPX feederInner775 = MotorControlHelper.createMasterVictorSPX(Constants.kFeederInnerID);

    private Feeder() {
        feederInner775.setInverted(true);
        feederOuter775.setInverted(true);
    }

    public void setSpeed(double speed) {
        feederOuter775.set(ControlMode.PercentOutput, speed);
        feederInner775.set(ControlMode.PercentOutput, speed);
    }

    public boolean getPhotoSensor() {
        return photoSensor.get();
    }
}
