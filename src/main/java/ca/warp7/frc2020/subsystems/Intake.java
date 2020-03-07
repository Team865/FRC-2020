/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.subsystems;

import ca.warp7.frc2020.lib.LazySolenoid;
import ca.warp7.frc2020.lib.motor.MotorControlHelper;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static ca.warp7.frc2020.Constants.*;

public final class Intake implements Subsystem {
    private static Intake instance;

    public static Intake getInstance() {
        if (instance == null) instance = new Intake();
        return instance;
    }

    private CANSparkMax intakeNeo550 = MotorControlHelper
            .createMasterSparkMAX(kIntakeID);

    private LazySolenoid intakeExtensionPiston =
            new LazySolenoid(kIntakeExtensionID, kEnableSolenoids);

    private Intake() {
        intakeNeo550.setInverted(true);
    }

    public void setSpeed(double speed) {
        SmartDashboard.putNumber("Intake Speed", speed);
        SmartDashboard.putBoolean("Intake isReversed", speed < 0);
        intakeNeo550.set(speed);
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
