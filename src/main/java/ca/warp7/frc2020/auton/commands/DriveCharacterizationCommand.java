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
 *
 * We need a custom command because FRC Characterization does
 * not support encoders from the Falcons.
 *
 * The command uses NetworkTables to interact with the FRC-
 * characterization GUI
 */
public class DriveCharacterizationCommand extends CommandBase {
    private DriveTrain driveTrain = DriveTrain.getInstance();

    private NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();

    NetworkTableEntry autoSpeedEntry = ntInstance.getEntry("/robot/autospeed");
    NetworkTableEntry telemetryEntry = ntInstance.getEntry("/robot/telemetry");
    NetworkTableEntry rotateEntry = ntInstance.getEntry("/robot/rotate");

    double priorSpeed = 0;
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
        double leftRate = driveTrain.getLeftVelocity();

        double rightPosition = driveTrain.getRightPosition();
        double rightRate = driveTrain.getRightVelocity();

        double battery = RobotController.getBatteryVoltage();
        double motorVolts = battery * Math.abs(priorSpeed);

        // Retrieve the commanded speed from NetworkTables
        double autospeed = autoSpeedEntry.getDouble(0);
        priorSpeed = autospeed;

        // command motors to do things
        double leftSpeed = (rotateEntry.getBoolean(false) ? -1 : 1) * autospeed;

        driveTrain.setPercentOutput(leftSpeed, autospeed);

        // send telemetry data array back to NT
        numberArray[0] = now;
        numberArray[1] = battery;
        numberArray[2] = autospeed;
        numberArray[3] = motorVolts;
        numberArray[4] = motorVolts;
        numberArray[5] = leftPosition;
        numberArray[6] = rightPosition;
        numberArray[7] = leftRate;
        numberArray[8] = rightRate;
        numberArray[9] = driveTrain.getYaw().getRadians();

        telemetryEntry.setNumberArray(numberArray);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.neutralOutput();
    }
}
