/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.subsystems;

import ca.warp7.frc2020.lib.LazySolenoid;
import ca.warp7.frc2020.lib.motor.MotorControlHelper;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static ca.warp7.frc2020.Constants.*;

public final class Intake implements Subsystem {
    private static Intake instance;

    public static Intake getInstance() {
        if (instance == null) instance = new Intake();
        return instance;
    }

    private VictorSPX intakeBagMotor = MotorControlHelper
            .createMasterVictorSPX(kIntakeID);

    private LazySolenoid intakeExtensionPiston =
            new LazySolenoid(kIntakeExtensionID, kEnableSolenoids);

    public void setSpeed(double speed) {
        intakeBagMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setExtended(boolean extended) {
        intakeExtensionPiston.set(extended);
    }

    public boolean isExtended() {
        return intakeExtensionPiston.get();
    }

    public void toggle() {
        setExtended(!intakeExtensionPiston.get());
    }
}
