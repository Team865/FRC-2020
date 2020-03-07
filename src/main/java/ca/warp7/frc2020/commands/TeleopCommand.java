/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.commands;

import ca.warp7.frc2020.Constants;
import ca.warp7.frc2020.auton.commands.RobotStateCommand;
import ca.warp7.frc2020.auton.commands.VisionAlignCommand;
import ca.warp7.frc2020.lib.Util;
import ca.warp7.frc2020.lib.XboxController;
import ca.warp7.frc2020.subsystems.Flywheel;
import ca.warp7.frc2020.subsystems.Limelight;
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
    private Command intakeExtensionCommand = SingleFunctionCommand.getIntakeExtensionToggle();
    private Command flywheelSpeedCommand = new FlywheelSpeedCommand(this::getWantedFlywheelRPS);

    private Command climbSpeedOptionalCommand = Constants.isPracticeRobot() ?
            new InstantCommand() :
            new ClimbSpeedCommand(this::getClimbSpeed);

    private Command resetRobotStateCommand = new RobotStateCommand();

    private Command setLowGearDriveCommand = SingleFunctionCommand.getSetDriveLowGear();
    private Command setHighGearDriveCommand = SingleFunctionCommand.getSetDriveHighGear();
    private Command zeroYawCommand = SingleFunctionCommand.getZeroYaw();
    private Command brakeCommand = SingleFunctionCommand.getSetDriveBrakeMode();

    //  private Command flywheelHoodToggleCommand = SingleFunctionCommand.getFlywheelHoodToggle();


    private Command climbLockToggleOptionalCommand = Constants.isPracticeRobot() ?
            new InstantCommand() :
            SingleFunctionCommand.getClimbLockToggle();
    private Command flywheelSetHoodCloseCommand = SingleFunctionCommand.getFlywheelSetHoodCloseCommand();
    private Command flywheelSetHoodFarCommand = SingleFunctionCommand.getFlywheelSetHoodFarCommand();

    private XboxController driver = new XboxController(0);
    private XboxController operator = new XboxController(1);

    private boolean isIntaking = false;
    private boolean isReversed = false;

    private double farShotAdjustment = 0.0;
    private double closeShotAdjustment = 0.0;
    private boolean isClose = false;
    private boolean isPriming = false;

    private double getWantedFlywheelRPS() {
        if (isPriming) {
            if (!isClose) {
                return Constants.kFlywheelFarRPS + farShotAdjustment;
            } else {
                return Flywheel.getOptimalCloseShotRPS() + closeShotAdjustment;
            }
        }
        return 0;
    }


//    public double getControlPanelSpinnerSpeed() {
//        return operator.rightX;
//    }

    public double getIntakeSpeed() {
        if (isIntaking)
            return Util.applyDeadband(driver.leftTrigger, 0.2) * (isReversed ? -1 : 1);
        return 0.0;
    }

    private double getXSpeed() {
        return Util.applyDeadband(driver.leftY, 0.2);
    }

    private double getZRotation() {
        double zRotation = Util.applyDeadband(driver.rightX, 0.15);
        if (driver.backButton.isDown()) zRotation *= 0.5;
        if (isQuickTurn() || driver.leftY < 0) {
            return zRotation;
        } else {
            return -1 * zRotation;
        }
    }

    private boolean isQuickTurn() {
        return driver.leftBumper.isHeldDown();
    }

    private double getVisionAlignSpeed() {
        return getXSpeed() / 2.0;
    }

    private double getFeedSpeed() {
        return 0.7 * Util.applyDeadband(driver.rightTrigger, 0.3) * (isReversed ? -1 : 1);
    }


    private double getClimbSpeed() {
        return Util.applyDeadband(operator.leftY, 0.3);
    }

    @Override
    public void initialize() {
        farShotAdjustment = 0.0;
        closeShotAdjustment = 0.0;

        zeroYawCommand.schedule();
        resetRobotStateCommand.schedule();

        setLowGearDriveCommand.schedule();
        curvatureDriveCommand.schedule();
        flywheelSpeedCommand.schedule();
        feedCommand.schedule();
        // controlPanelDisplay.schedule();
        climbSpeedOptionalCommand.schedule();
        intakingCommand.schedule();
        brakeCommand.schedule();
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

        if (!isIntaking) {
            isIntaking = driver.leftTrigger > 0.22;
        } else {
            isIntaking = driver.leftTrigger > 0.2;
        }

        if (driver.xButton.isPressed() || driver.xButton.isReleased()) intakeExtensionCommand.schedule();

        isReversed = driver.yButton.isHeldDown();

        if (driver.aButton.isPressed()) {
            visionAlignCommand.schedule();
        } else if (driver.aButton.isReleased()) {
            visionAlignCommand.cancel();
            curvatureDriveCommand.schedule();
        }

        // Operator

        if (operator.leftTrigger > 0.2) {
            isPriming = true;
            isClose = true;
            flywheelSetHoodCloseCommand.schedule();
            Limelight.getInstance().setPipeline(0);
        } else if (operator.rightTrigger > 0.3) {
            isPriming = true;
            isClose = false;
            flywheelSetHoodFarCommand.schedule();
            Limelight.getInstance().setPipeline(1);
        } else
            isPriming = false;

        if (operator.leftBumper.isPressed()) {
            if (isClose) closeShotAdjustment -= 0.5;
            else farShotAdjustment -= 0.5;
        }
        if (operator.rightBumper.isPressed()) {
            if (isClose) closeShotAdjustment += 0.5;
            else farShotAdjustment += 0.5;
        }
        if (operator.xButton.isPressed()) {
            if (isClose) closeShotAdjustment -= 5;
            else farShotAdjustment -= 5;
        }
        if (operator.yButton.isPressed()) {
            if (isClose) closeShotAdjustment += 5;
            else farShotAdjustment += 5;
        }

        if (operator.startButton.isPressed())
            climbLockToggleOptionalCommand.schedule();
    }
}
