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
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static ca.warp7.frc2020.Constants.*;

public final class Climber implements Subsystem {
    private static Climber instance;

    public static Climber getInstance() {
        if (instance == null) instance = new Climber();
        return instance;
    }

    private VictorSPX climberMaster775 = MotorControlHelper
            .createMasterVictorSPX(kClimberMasterID);

    private LazySolenoid climberPancakeCylinder =
            new LazySolenoid(kClimberLockActuatorID, kEnableSolenoids);

    public Climber() {
        climberMaster775.setInverted(true);
        MotorControlHelper.assignFollowerVictorSPX(
                climberMaster775,
                kClimberFollowerID,
                InvertType.OpposeMaster
        );
    }

    public void setSpeed(double speed) {
        climberMaster775.set(ControlMode.PercentOutput, speed);
    }

    public boolean isLocked() {
        return !climberPancakeCylinder.get();
    }

    public void toggleLock() {
        climberPancakeCylinder.toggle();
    }
}
