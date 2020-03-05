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
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static ca.warp7.frc2020.Constants.kBeamBreakID;

public final class Feeder implements Subsystem {
    private static Feeder instance;
    private boolean previousState = false;
    private boolean prevEnabled = false;
    private int cellsCount = 0;

    public static Feeder getInstance() {
        if (instance == null) instance = new Feeder();
        return instance;
    }

    private DigitalInput beamBreak = new DigitalInput(kBeamBreakID);

    private VictorSPX feederOuter775 = MotorControlHelper.createMasterVictorSPX(Constants.kFeederOuterID);
    private VictorSPX feederInner775 = MotorControlHelper.createMasterVictorSPX(Constants.kFeederInnerID);

    private Feeder() {

        feederOuter775.setInverted(true);
        feederInner775.enableVoltageCompensation(false);
        feederOuter775.enableVoltageCompensation(false);
        feederInner775.configOpenloopRamp(0.1);
        feederOuter775.configOpenloopRamp(0.1);
    }

    public void setSpeed(double speed) {
        feederOuter775.set(ControlMode.PercentOutput, speed);
        feederInner775.set(ControlMode.PercentOutput, speed);
    }

    public boolean getBeamBreak() {
        return beamBreak.get();
    }

    public int getCellsCount() {
        return cellsCount;
    }


    @Override
    public void periodic() {
        boolean isEnabled = RobotState.isEnabled();
        if (isEnabled) {
            if (prevEnabled) {
                SmartDashboard.putNumber("Cells shot", getCellsCount());
                boolean currentState = this.getBeamBreak();

                if (!previousState && currentState) {
                    cellsCount++;
                }
                previousState = currentState;
            } else {
                cellsCount = 0;
            }
        }

        prevEnabled = isEnabled;
    }
}
