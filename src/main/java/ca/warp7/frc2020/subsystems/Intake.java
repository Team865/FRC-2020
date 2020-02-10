/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.subsystems;

import ca.warp7.frc2020.Constants;
import ca.warp7.frc2020.lib.motor.MotorControlHelper;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static ca.warp7.frc2020.Constants.*;

public final class Intake implements Subsystem {
    private static Intake instance;

    public static Intake getInstance() {
        if (instance == null) instance = new Intake();
        return instance;
    }

    private VictorSPX intakeBagMotor = MotorControlHelper
            .createMasterVictorSPX(Constants.kIntakeID);
    private Solenoid intakeExtensionPiston1 = new Solenoid(kIntakeExtensionLeftID);
    private Solenoid intakeExtensionPiston2 = new Solenoid(kIntakeExtensionRightID);

    public void setSpeed(double speed) {
//        intakeBagMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setExtended(boolean extended) {
        intakeExtensionPiston1.set(extended);
        intakeExtensionPiston2.set(extended);
    }

    public boolean isExtended() {
        return intakeExtensionPiston1.get();
    }

    public void toggle() {
        setExtended(!intakeExtensionPiston1.get());
    }
}
