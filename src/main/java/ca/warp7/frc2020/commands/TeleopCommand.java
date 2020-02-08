/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.commands;

import ca.warp7.frc2020.Constants;
import ca.warp7.frc2020.lib.Util;
import ca.warp7.frc2020.lib.XboxController;
import ca.warp7.frc2020.lib.control.LatchedBoolean;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * This class is responsible for scheduling the proper commands while operator
 * controlled
 */
public class TeleopCommand extends CommandBase {


    private Command curvatureDriveCommand = Constants.kUseKinematicsDrive ?
            new KinematicsDriveCommand(this::getXSpeed, this::getZRotation, this::isQuickTurn) :
            new PercentDriveCommand(this::getXSpeed, this::getZRotation, this::isQuickTurn);

//    private Command visionAlignCommand = new VisionAlignCommand(this::getVisionAlignSpeed);
//
//    private Command controlPanelDisplay = new ControlPanelCommand(this::getControlPanelSpinnerSpeed);
    private Command forwardFeedCommand = new ForwardFeedCommand(this::getForwardFeedSpeed);
    private Command reverseFeedCommand = new ReverseFeedCommand(this::getReverseFeedSpeed);
//    private Command intakingCommand = new IntakingCommand(this::getIsIntaking);
    private Command flywheelSpeedCommand = new FlywheelSpeedCommand(this::getWantedFlywheelRPS);
//    private Command climbSpeedCommand = new ClimbSpeedCommand(this::getClimbSpeed);
//
//    private Command robotStateEstimationCommand = SingleFunctionCommand.getRobotStateEstimation();*/
    private Command setLowGearDriveCommand = SingleFunctionCommand.getSetDriveLowGear();
//     private Command setHighGearDriveCommand = SingleFunctionCommand.getSetDriveHighGear();
//
//     private Command lockHangingClimberCommand = SingleFunctionCommand.getClimbLockToggle();
//     private Command flywheelHoodToggleCommand = SingleFunctionCommand.getFlywheelHoodToggle();
//
     private LatchedBoolean forwardFeedLatch = new LatchedBoolean();
     private LatchedBoolean reverseFeedLatch = new LatchedBoolean();

    private XboxController driver = new XboxController(0);
    private XboxController operator = new XboxController(1);

//    private boolean isIntaking = false;

//    private int farShotAdjustment = 0;
//    private int closeShotAdjustment = 0;
//    private boolean isClose = false;
//    private boolean isPriming = false;
//
//    private int getWantedFlywheelRPM() {
//        if (isPriming)
//            return isClose ? Constants.flywheelDefaultCloseRPM + closeShotAdjustment
//                    : Constants.flywheelFarRPM + farShotAdjustment;
//        return 0;
//    }
//
//    public double getControlPanelSpinnerSpeed() {
//        return operator.leftTrigger;
//    }
//
//    public boolean getIsIntaking() {
//        return isIntaking;
//    }

    double speed = 88.0;

    private double getWantedFlywheelRPS() {
        return speed;
    }

    private double getXSpeed() {
        return Util.applyDeadband(-driver.leftY, 0.2);
    }

    private double getZRotation() {
        return Util.applyDeadband(driver.rightX, 0.15);
    }

    private boolean isQuickTurn() {
        return driver.leftBumper.isHeldDown();
    }

//    private double getVisionAlignSpeed() {
//        return getXSpeed() / 2.0;
//    }

    private double getForwardFeedSpeed() {
        return Util.applyDeadband(driver.rightTrigger, 0.2);
    }

    private double getReverseFeedSpeed() {
        return Util.applyDeadband(driver.leftTrigger, 0.2);
    }

//    private double getClimbSpeed() {
//        return Util.applyDeadband(operator.rightY, 0.5);
//    }

    @Override
    public void initialize() {
        setLowGearDriveCommand.schedule();
        curvatureDriveCommand.schedule();
        flywheelSpeedCommand.schedule();
        //forwardFeedCommand.schedule();
        // controlPanelDisplay.schedule();
        // climbSpeedCommand.schedule();
        // intakingCommand.schedule();
        // robotStateEstimationCommand.schedule();
    }

    @Override
    public void execute() {
        driver.collectControllerData();
        operator.collectControllerData();

        // Driver

//        if (driver.rightBumper.isPressed())
//             setHighGearDriveCommand.schedule();
//         else if (driver.rightBumper.isReleased())
//             setLowGearDriveCommand.schedule();
//
//         if (isIntaking)
//             isIntaking = driver.leftTrigger > 0.25;
//         else
//             isIntaking = driver.leftTrigger > 0.3;
//
//         if (driver.aButton.isPressed())
//             visionAlignCommand.schedule();
//         else if (driver.aButton.isReleased())
//             visionAlignCommand.cancel();
//
//         if (feedThresholdLatch.update(driver.leftTrigger > 0.3)) {
//             feedCommand.schedule();
//         } else if (driver.leftTrigger < 0.25)
//             feedCommand.cancel();

        if (reverseFeedLatch.update(driver.leftTrigger > 0.2)) {
            forwardFeedCommand.cancel();
            reverseFeedCommand.schedule();
        } else if (forwardFeedLatch.update(driver.rightTrigger > 0.2)) {
            reverseFeedCommand.cancel();
            forwardFeedCommand.schedule();
        }

        // Operator
//        if (operator.rightBumper.isDown()) {
//             isPriming = true;
//             isClose = false;
//             if (operator.rightBumper.isPressed() && Flywheel.getInstance().getHood())
//                 flywheelHoodToggleCommand.schedule();
//         } else if (operator.leftBumper.isDown()) {
//             isPriming = true;
//             isClose = true;
//             if (operator.leftBumper.isPressed() && !Flywheel.getInstance().getHood())
//                 flywheelHoodToggleCommand.schedule();
//         } else
//             isPriming = false;
//
//         if (operator.aButton.isPressed())
//             farShotAdjustment += 200;
//         if (operator.bButton.isPressed())
//             farShotAdjustment -= 200;
//         if (operator.xButton.isPressed())
//             closeShotAdjustment += 200;
//         if (operator.yButton.isPressed())
//             closeShotAdjustment -= 200;
//
//         if (operator.backButton.isPressed())
//             lockHangingClimberCommand.schedule();
    }
}
