/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020;

import ca.warp7.frc2020.lib.NetworkUtil;
import ca.warp7.frc2020.lib.control.PID;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Configuration

    public static final boolean kEnableSolenoids = true;
    public static final boolean kDebugCommandScheduler = false;
    public static final boolean kUseKinematicsDrive = false;
    public static final boolean kUseNotifierForMainLoop = false;

    // CAN IDs

    public static final int kDriveLeftMasterID = 31;
    public static final int kDriveLeftFollowerID = 32;

    public static final int kDriveRightMasterID = 34;
    public static final int kDriveRightFollowerID = 33;

    public static final int kFlywheelShooterMasterID = 20;
    public static final int kFlywheelShooterFollowerID = 22;

    public static final int kHopperID = 24;
    public static final int kIntakeID = 26;

    public static final int kClimberMasterID = 6;
    public static final int kClimberFollowerID = 7;

    public static final int kFeederOuterID = 1;
    public static final int kFeederInnerID = 2;

    public static final int kControlPanelManipulatorID = -1;


    // PCM IDs

    public static final int kFlywheelHoodActuatorID = 4;
    public static final int kIntakeExtensionID = 5;
    public static final int kClimberLockActuatorID = 6;
    public static final int kDriveShifterID = 7;

    // DIO IDs

    public static final int kPhotoSensorID = 0;

    // Drive Train Tuning

    public static final double kLowGearRampRate = 0.15;
    public static final double kHighGearRampRate = 0.3;

    public static final PID kAutonLowGearVelocityPID =
            new PID(0.0, 0.0, 0.0, 0.0);
    public static final PID kTeleopLowGearVelocityPID =
            new PID(0.0, 0.0, 0.0, 0.0);
    public static final PID kTeleopHighGearVelocityPID =
            new PID(0.0, 0.0, 0.0, 0.0);
    public static final PID kVisionAlignmentYawPID =
            new PID(0.025, 0.0, 0.002, 0.0);

    // Flywheel Tuning

    public static final double flywheelDefaultCloseRPS = 57.0;
    public static final double flywheelFarRPS = 100.0;
    public static final double kFlywheelKp = 1.94;
    public static final double kFlywheelKs = 0.0911;
    public static final double kFlywheelKv = (0.0644 + 0.063) / 2;
    public static final double kFlywheelKa = (0.0401 + 0.0483) / 2;
    public static final double kFlywheelGearRatio = 1.0 / 2.0; // 0.5

    public static final double maxInnerGoalDist = 7; // meters
    public static final double innerToOuterGoalAdjustment = 0.7; //meters // the difference in where you are aiming

    public static double optimaInnerGoalRPS(double metersFromGoal) {
        return 2.79 * Math.pow(metersFromGoal - 4.10, 2) + 55.01;
    }

    // Intake Constants

    public static final double kIntakingSpeed = 0.3; // percent

    //Feeder Constants

    public static final double kFeedingSpeed = 0.6; //percent

    //Hopper constants

    public static final double kHopperSpeed = 0.6; //percent

    // Drive Train Constants

    public static final double kWheelBaseRadius = 0.35; // metres
    public static final double kDriveWheelRadius = 0.0760858711932102; // metres
    public static final double kMaxVoltage = 12.0; // volts

    public static class LowGear {
        public static final double kGearRatio = 42.0 / 10.0 * 60.0 / 14.0; // 18.0

        public static final double kMetresPerRotation =
                (2 * Math.PI * kDriveWheelRadius) / kGearRatio; // ticks/m

        public static final SimpleMotorFeedforward kTransmission =
                new SimpleMotorFeedforward(0.0534, 4.180, 0.429);

    }

    public static class HighGear {
        public static final double kGearRatio = 42.0 / 10.0 * 50.0 / 24.0; // 8.75

        public static final double kMetresPerRotation =
                (2 * Math.PI * kDriveWheelRadius) / kGearRatio; // m/rotation

        public static final SimpleMotorFeedforward kTransmission =
                new SimpleMotorFeedforward(1.0, 12.0, 0.4);
    }

    private static class PracticeRobotDetector {
        private static final String kPracticeRobotAddress = "00-80-2F-27-06-8E";
        private static final boolean kIsPracticeRobot = NetworkUtil
                .getMACAddress().equals(kPracticeRobotAddress);
    }

    public static boolean isPracticeRobot() {
        return PracticeRobotDetector.kIsPracticeRobot;
    }
}
