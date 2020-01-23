/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.subsystems;

import ca.warp7.frc2020.lib.motor.MotorControlHelper;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static ca.warp7.frc2020.Constants.*;

public final class Flywheel implements Subsystem {
    private static Flywheel instance;

    public static Flywheel getInstance() {
        if (instance == null) instance = new Flywheel();
        return instance;
    }

    private CANSparkMax flywheelMasterNeo = MotorControlHelper.createMasterSparkMAX(kFlywheelShooterMasterID);
    private Solenoid flywheelHoodPiston = new Solenoid(kFlywheelHoodActuatorID);

    private Flywheel() {
        MotorControlHelper.assignFollowerSparkMAX(flywheelMasterNeo, kFlywheelShooterFollowerID);
    }

    public void setRPM(int rpm) {
        flywheelMasterNeo.set(-1 * rpm / 2.0 / 5676.0);
    }

    public double getRPM() {
        return -1 * flywheelMasterNeo.getEncoder().getVelocity() * 2;
    }

    public void setHood(boolean up) {
        flywheelHoodPiston.set(up);
    }

    public boolean getHood() {
        return flywheelHoodPiston.get();
    }

    public void toggleHood() {
        setHood(!getHood());
    }
}
