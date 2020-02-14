/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.auton.commands;

import ca.warp7.frc2020.subsystems.DriveTrain;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * This command is used with the FRC Characterization tools
 * from https://github.com/wpilibsuite/frc-characterization
 */
public class DriveCharacterizationCommand extends CommandBase {
    private DriveTrain driveTrain = DriveTrain.getInstance();

    private NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();

    NetworkTableEntry autoSpeedEntry = ntInstance.getEntry("/robot/autospeed");
    NetworkTableEntry telemetryEntry = ntInstance.getEntry("/robot/telemetry");
    NetworkTableEntry rotateEntry = ntInstance.getEntry("/robot/rotate");

    Number[] numberArray = new Number[10];

    public DriveCharacterizationCommand() {
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        // stop the drive train at the start of characterization
        driveTrain.neutralOutput();

        System.out.println("====STARTING CHARACTERIZATION====");
        System.out.println("Changing NetworkTables Speed");

        // Set the update rate instead of using flush because of a ntcore bug
        // -> probably don't want to do this on a robot in competition
        ntInstance.setUpdateRate(0.010);
    }

    @Override
    public void execute() {
        // Retrieve values to send back before telling the motors to do something
        double now = Timer.getFPGATimestamp();

        double leftPosition = driveTrain.getLeftPosition();
        double leftVelocity = driveTrain.getLeftVelocity();

        double rightPosition = driveTrain.getRightPosition();
        double rightVelocity = driveTrain.getRightVelocity();

        double batteryVoltage = RobotController.getBatteryVoltage();

        double leftVoltage = driveTrain.getLeftVoltage();
        double rightVoltage = driveTrain.getRightVoltage();

        double angle = driveTrain.getContinousAngleRadians();

        // Retrieve the commanded speed from NetworkTables
        double autospeed = autoSpeedEntry.getDouble(0);

        // command motors to do things
        double leftSpeed = (rotateEntry.getBoolean(false) ? -1 : 1) * autospeed;

        driveTrain.setPercentOutput(leftSpeed, autospeed);

        // send telemetry data array back to NT
        numberArray[0] = now;
        numberArray[1] = batteryVoltage;
        numberArray[2] = autospeed;
        numberArray[3] = leftVoltage;
        numberArray[4] = rightVoltage;
        numberArray[5] = leftPosition;
        numberArray[6] = rightPosition;
        numberArray[7] = leftVelocity;
        numberArray[8] = rightVelocity;
        numberArray[9] = angle;

        telemetryEntry.setNumberArray(numberArray);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.neutralOutput();
    }
}
