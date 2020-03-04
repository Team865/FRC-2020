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
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static ca.warp7.frc2020.Constants.*;
import static ca.warp7.frc2020.Constants.getOptimaInnerGoalRPS;

public final class Flywheel implements Subsystem {
    private static Flywheel instance;
    private double targetRPS;

    public static Flywheel getInstance() {
        if (instance == null) instance = new Flywheel();
        return instance;
    }

    private CANSparkMax flywheelMasterNeo = MotorControlHelper.createMasterSparkMAX(kFlywheelShooterMasterID);

    private LazySolenoid flywheelHoodPiston =
            new LazySolenoid(kFlywheelHoodActuatorID, kEnableSolenoids);

    private Flywheel() {
        flywheelMasterNeo.setIdleMode(IdleMode.kCoast);
        flywheelMasterNeo.setOpenLoopRampRate(1.2);
        flywheelMasterNeo.enableVoltageCompensation(12.0);
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

    public double getPercentError() {
        if (targetRPS != 0)
            return getError() / targetRPS;
        else
            return 0;
    }

    public static double calculateOptimalCloseShotRPS(double metersFromGoal) {
//        return getOptimaInnerGoalRPS(metersFromGoal);
        double r = 0.5; // meters // the distance to ramp between the normal shot and the outer shot
        if (metersFromGoal <= kMaxInnerGoalDist - r) {
            // if you can hit threes, you don't need to adjust the RPS
            return getOptimaInnerGoalRPS(metersFromGoal);
        } else if (metersFromGoal >= kMaxInnerGoalDist) {
            // if you can't hit threes from that distance (because they would hit the top of the power port)
            // you need to aim for the outer goal instead
            return getOptimaInnerGoalRPS(metersFromGoal - kInnerToOuterGoalAdjustment);
        } else {
            // interpolate
            return getOptimaInnerGoalRPS(kInnerToOuterGoalAdjustment *
                    (1 + (metersFromGoal - kMaxInnerGoalDist) / r));
        }
    }

    public static double getOptimalCloseShotRPS() {
        Limelight limelight = Limelight.getInstance();
        if (limelight.hasValidTarget()) {
            double d = limelight.getCameraToTarget();
            return calculateOptimalCloseShotRPS(d);
        } else return kFlywheelDefaultCloseRPS;
    }

    public void calcOutput() {
        if (targetRPS == 0.0)
            this.setVoltage(0.0);
        else
            this.setVoltage((targetRPS + getError() * kFlywheelKp) * kFlywheelKv +
                    kFlywheelKs * Math.signum(targetRPS));
    }

    private void setVoltage(double voltage) {
        flywheelMasterNeo.set(voltage / 12);
    }

    public void setHoodCloseShot(boolean hoodCloseShot) {
        flywheelHoodPiston.set(hoodCloseShot);
    }
}
