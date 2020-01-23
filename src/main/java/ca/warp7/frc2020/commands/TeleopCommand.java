/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.commands;

import ca.warp7.frc2020.Constants;
import ca.warp7.frc2020.auton.commands.VisionAlignCommand;
import ca.warp7.frc2020.lib.Util;
import ca.warp7.frc2020.lib.XboxController;
import ca.warp7.frc2020.lib.control.LatchedBoolean;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * This class is responsible for scheduling the proper commands while operator controlled
 */
@SuppressWarnings("FieldCanBeLocal")
public class TeleopCommand extends CommandBase {


    private Command curvatureDriveCommand = Constants.kUseKinematicsDrive ?
            new KinematicsDriveCommand(this::getXSpeed, this::getZRotation, this::isQuickTurn) :
            new PercentDriveCommand(this::getXSpeed, this::getZRotation, this::isQuickTurn);

    private Command visionAlignCommand = new VisionAlignCommand(this::getVisionAlignSpeed);

    private Command controlPanelDisplay = new ControlPanelCommand(this::getControlPanelSpinnerSpeed);
    private Command feedCommand = new PowerCellFeedCommand(this::getFeedingSpeed);
    private Command unjamCommand = new PowerCellUnjamCommand(this::getUnjamSpeed);
    private Command flywheelSpeedCommand = new FlywheelSpeedCommand(this::getWantedRPM);

    private Command robotStateEstimationCommand = SingleFunctionCommand.getRobotStateEstimation();
    private Command setLowGearDriveCommand = SingleFunctionCommand.getSetDriveLowGear();
    private Command setHighGearDriveCommand = SingleFunctionCommand.getSetDriveHighGear();

    private Command lockHangingClimberCommand = SingleFunctionCommand.getLockHangingClimber();
    private Command intakeExtensionToggleCommand = SingleFunctionCommand.getIntakeExtensionToggle();
    private Command flywheelHoodToggleCommand = SingleFunctionCommand.getFlywheelHoodToggle();

    private LatchedBoolean kFeedThresholdLatch = new LatchedBoolean();
    private LatchedBoolean kUnjamThresholdLatch = new LatchedBoolean();

    private XboxController driver = new XboxController(0);
    private XboxController operator = new XboxController(1);

    private int flywheelWantedRPM = 0;
    private double controlPanelSpinnerSpeed = 0;

    private int getWantedRPM() {
        return flywheelWantedRPM;
    }

    public double getControlPanelSpinnerSpeed() {
        return controlPanelSpinnerSpeed;
    }

    private double getXSpeed() {
        return Util.applyDeadband(-driver.leftY, 0.2);
    }

    private double getVisionAlignSpeed() {
        return getXSpeed() / 2.0;
    }

    private double getZRotation() {
        return Util.applyDeadband(driver.rightX, 0.15);
    }

    private double getFeedingSpeed() {
        return Util.applyDeadband(driver.leftTrigger, 0.3);
    }

    private double getUnjamSpeed() {
        return Util.applyDeadband(driver.rightTrigger, 0.3);
    }

    private boolean isQuickTurn() {
        return driver.leftBumper.isHeldDown();
    }

    @Override
    public void initialize() {
        setLowGearDriveCommand.schedule();
        curvatureDriveCommand.schedule();
        controlPanelDisplay.schedule();
        flywheelSpeedCommand.schedule();
        robotStateEstimationCommand.schedule();
    }

    @Override
    public void execute() {
        driver.collectControllerData();
        operator.collectControllerData();

        if (driver.rightBumper.isPressed()) {
            setHighGearDriveCommand.schedule();
        } else if (driver.rightBumper.isReleased()) {
            setLowGearDriveCommand.schedule();
        }

        if (driver.aButton.isPressed()) {
            curvatureDriveCommand.cancel();
            visionAlignCommand.schedule();
        } else if (driver.aButton.isReleased()) {
            visionAlignCommand.cancel();
            curvatureDriveCommand.schedule();
        }

        if (driver.bButton.isPressed()) {
            intakeExtensionToggleCommand.schedule();
        }

        if (kUnjamThresholdLatch.update(driver.leftTrigger > 0.3)) {
            unjamCommand.cancel();
            feedCommand.schedule();
        }
        if (kFeedThresholdLatch.update(driver.leftTrigger > 0.3)) {
            unjamCommand.cancel();
            feedCommand.schedule();
        }

        if (operator.bButton.isPressed()) {
            flywheelHoodToggleCommand.schedule();
        }
        if (operator.startButton.isPressed()) {
            lockHangingClimberCommand.schedule();
        }
    }
}
