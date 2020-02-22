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
import ca.warp7.frc2020.subsystems.Flywheel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is responsible for scheduling the proper commands while operator
 * controlled
 */
public class TeleopCommand extends CommandBase {
    private Command curvatureDriveCommand = Constants.kUseKinematicsDrive ?
            new KinematicsDriveCommand(this::getXSpeed, this::getZRotation, this::isQuickTurn) :
            new PercentDriveCommand(this::getXSpeed, this::getZRotation, this::isQuickTurn);

        private Command visionAlignCommand = new VisionAlignCommand(this::getVisionAlignSpeed);

    //    private Command controlPanelDisplay = new ControlPanelCommand(this::getControlPanelSpinnerSpeed);
    private Command feedCommand = new FeedCommand(this::getFeedSpeed);
    private Command intakingCommand = new IntakingCommand(this::getIntakeSpeed);
    private Command flywheelSpeedCommand = new FlywheelSpeedCommand(this::getWantedFlywheelRPS);

    private Command climbSpeedOptionalCommand = Constants.isPracticeRobot() ?
            new InstantCommand() :
            new ClimbSpeedCommand(this::getClimbSpeed);

    private Command resetRobotStateCommand = SingleFunctionCommand.getResetRobotState();
    private Command robotStateEstimationCommand = SingleFunctionCommand.getRobotStateEstimation();
    private Command reportStateCommand = SingleFunctionCommand.getReportRobotState();
    private Command setLowGearDriveCommand = SingleFunctionCommand.getSetDriveLowGear();
    private Command setHighGearDriveCommand = SingleFunctionCommand.getSetDriveHighGear();
    private Command zeroYawCommand = SingleFunctionCommand.getZeroYaw();
    private Command limelightGetPoseCommand = SingleFunctionCommand.getLimelightGetPoseCommand();

    private Command lockHangingClimberCommand = SingleFunctionCommand.getClimbLockToggle();
    private Command flywheelHoodToggleCommand = SingleFunctionCommand.getFlywheelHoodToggle();

    private XboxController driver = new XboxController(0);
    private XboxController operator = new XboxController(1);

    private boolean isIntaking = false;
    private boolean isReversed = false;

    private double farShotAdjustment = 0.0;
    private double closeShotAdjustment = 0.0;
    private boolean isClose = false;
    private boolean isPriming = false;

    private double getWantedFlywheelRPS() {
        if (isPriming)
            return isClose ? Constants.flywheelDefaultCloseRPS + closeShotAdjustment
                    : Constants.flywheelFarRPS + farShotAdjustment;
        return 0;
    }


//    public double getControlPanelSpinnerSpeed() {
//        return operator.leftTrigger;
//    }

    public double getIntakeSpeed() {
        if (isIntaking)
            return Util.applyDeadband(driver.leftTrigger, 0.2) * (isReversed ? -1 : 1);
        return 0.0;
    }

//    private double getWantedFlywheelRPS() {
//        return Constants.flywheelDefaultCloseRPS + closeShotAdjustment;
//    }

    private double getXSpeed() {
        return Util.applyDeadband(-driver.leftY, 0.2);
    }

    private double getZRotation() {
        return Util.applyDeadband(driver.rightX, 0.15);
    }

    private boolean isQuickTurn() {
        return driver.leftBumper.isHeldDown();
    }

    private double getVisionAlignSpeed() {
        return getXSpeed() / 2.0;
    }

    private double getFeedSpeed() {
        return Util.applyDeadband(driver.rightTrigger, 0.2) * (isReversed ? -1 : 1);
    }


    private double getClimbSpeed() {
        return Util.applyDeadband(operator.rightY, 0.3);
    }

    @Override
    public void initialize() {
        farShotAdjustment = 0.0;
        closeShotAdjustment = 0.0;

        setLowGearDriveCommand.schedule();
        curvatureDriveCommand.schedule();
        flywheelSpeedCommand.schedule();
        feedCommand.schedule();
        // controlPanelDisplay.schedule();
        climbSpeedOptionalCommand.schedule();
        intakingCommand.schedule();
        resetRobotStateCommand.schedule();
        robotStateEstimationCommand.schedule();
        zeroYawCommand.schedule();
        limelightGetPoseCommand.schedule();
    }

    @Override
    public void execute() {
        driver.collectControllerData();
        operator.collectControllerData();

        // Driver

        if (driver.rightBumper.isPressed())
            setHighGearDriveCommand.schedule();
        else if (driver.rightBumper.isReleased())
            setLowGearDriveCommand.schedule();

        if (isIntaking)
            isIntaking = driver.leftTrigger > 0.2;
        else
            isIntaking = driver.leftTrigger > 0.25;

        isReversed = driver.yButton.isHeldDown();

        if (driver.aButton.isPressed())
            visionAlignCommand.schedule();
        else if (driver.aButton.isReleased())
            visionAlignCommand.cancel();

        // Operator
        if (operator.rightBumper.isDown()) {
            isPriming = true;
            isClose = false;
            if (operator.rightBumper.isPressed() && Flywheel.getInstance().isHoodCloseShot())
                flywheelHoodToggleCommand.schedule();
        } else if (operator.leftBumper.isDown()) {
            isPriming = true;
            isClose = true;
            if (operator.leftBumper.isPressed() && !Flywheel.getInstance().isHoodCloseShot())
                flywheelHoodToggleCommand.schedule();
        } else
            isPriming = false;

        if (operator.aButton.isPressed()) {
            farShotAdjustment += 0.5;
        }
        if (operator.bButton.isPressed()) {
            farShotAdjustment -= 0.5;
        }
        if (operator.xButton.isPressed()) {
            closeShotAdjustment += 0.5;
        }
        if (operator.yButton.isPressed()) {
            closeShotAdjustment -= 0.5;
        }
        if (operator.backButton.isPressed())
            lockHangingClimberCommand.schedule();
    }
}
