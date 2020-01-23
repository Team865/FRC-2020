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

public final class Elevator implements Subsystem {
    private static Elevator instance;

    public static Elevator getInstance() {
        if (instance == null) instance = new Elevator();
        return instance;
    }

    private DigitalInput photoSensor = new DigitalInput(kPhotoSensorID);

    private VictorSPX elevatorOuter775 = MotorControlHelper.createMasterVictorSPX(Constants.kElevatorOuterID);
    private VictorSPX elevatorInner775 = MotorControlHelper.createMasterVictorSPX(Constants.kElevatorInnerID);

    private Elevator() {
    }

    public void setOuterSpeed(double speed) {
        elevatorOuter775.set(ControlMode.PercentOutput, speed);
    }

    public void setInnerSpeed(double speed) {
        elevatorInner775.set(ControlMode.PercentOutput, speed);
    }

    public boolean getPhotoSensor() {
        return photoSensor.get();
    }
}
