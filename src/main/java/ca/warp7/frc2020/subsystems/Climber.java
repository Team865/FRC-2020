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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class Climber implements Subsystem {
    private static Climber instance;

    public static Climber getInstance() {
        if (instance == null) instance = new Climber();
        return instance;
    }

    private VictorSPX climberMaster775 = MotorControlHelper
            .createMasterVictorSPX(Constants.kClimberMasterID);

    private Solenoid climberPancakeCylinder = new Solenoid(Constants.kClimberLockActuatorID);

    public Climber() {
        MotorControlHelper.assignFollowerVictorSPX(climberMaster775, Constants.kClimberFollowerID);
    }

    public void setSpeed(double speed) {
        climberMaster775.set(ControlMode.PercentOutput, speed);
    }

    public void lock() {
        climberPancakeCylinder.set(true);
    }

    public boolean isLocked() {
        return climberPancakeCylinder.get();
    }
    
    public void toggleLock() {
        climberPancakeCylinder.set(!isLocked());
    }
}
