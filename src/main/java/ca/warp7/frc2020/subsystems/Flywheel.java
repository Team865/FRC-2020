/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.subsystems;

import ca.warp7.frc2020.lib.motor.MotorControlHelper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static ca.warp7.frc2020.Constants.*;

public final class Flywheel implements Subsystem {
    private static Flywheel instance;
    private double targetRPS;

    public static Flywheel getInstance() {
        if (instance == null) instance = new Flywheel();
        return instance;
    }

    private CANSparkMax flywheelMasterNeo = MotorControlHelper.createMasterSparkMAX(kFlywheelShooterMasterID);

//    private LazySolenoid flywheelHoodPiston =
//            new LazySolenoid(kFlywheelHoodActuatorID, kEnableSolenoids);

    private Flywheel() {
        flywheelMasterNeo.setIdleMode(IdleMode.kCoast);
        flywheelMasterNeo.setOpenLoopRampRate(3.0);
        flywheelMasterNeo.enableVoltageCompensation(12);
        MotorControlHelper.assignFollowerSparkMAX(flywheelMasterNeo, kFlywheelShooterFollowerID, true);
    }

    public double getRPS() {
        return flywheelMasterNeo.getEncoder().getVelocity() / kFlywheelGearRatio / 60;
    }

    public void setTargetRPS(double target) {
        this.targetRPS = target;
    }


    public double getError() {
        return targetRPS - getRPS();
    }

    public void calcOutput() {
        this.setVoltage((targetRPS + getError() * kFlywheelKp) * kFlywheelKv + kFlywheelKs);
    }

    public void setVoltage(double voltage) {
        flywheelMasterNeo.setVoltage(voltage);
    }

    public void setHoodCloseShot(boolean hoodCloseShot) {
//        flywheelHoodPiston.set(up);
    }

    public boolean isHoodCloseShot() {
        return true;
//        return flywheelHoodPiston.get();
    }

    public void toggleHood() {
//        setHoodCloseShot(!isHoodCloseShot());
    }
}
