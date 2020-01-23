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

public final class Intake implements Subsystem {
    private static Intake instance;

    public static Intake getInstance() {
        if (instance == null) instance = new Intake();
        return instance;
    }

    private CANSparkMax intakeMiniNeo = MotorControlHelper.createMasterSparkMAX(kIntakeID);
    private Solenoid intakeExtensionPistons = new Solenoid(kIntakeExtensionID);

    public void setSpeed(double speed) {
        intakeMiniNeo.set(speed);
    }

    public void extend(){
        intakeExtensionPistons.set(true);
    }

    public boolean isExtended() {
        return intakeExtensionPistons.get();
    }

    public void toggleFourBarExtension() {
        intakeExtensionPistons.set(!intakeExtensionPistons.get());
    }
}
