/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020;

import ca.warp7.frc2020.auton.AutonomousCommand;
import ca.warp7.frc2020.commands.TeleopClimbCommand;
import ca.warp7.frc2020.subsystems.*;
import ca.warp7.frc2020.subsystems.drivetrain.FalconDriveTrainVariant;
import ca.warp7.frc2020.commands.DisabledCommand;
import ca.warp7.frc2020.commands.TeleopCommand;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public final class Robot extends TimedRobot {

    private CommandScheduler scheduler;

    private Command disabledCommand;
    private Command teleopCommand;
    private Command autonomousCommand;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        System.out.println("Hello me is robit!");

        // Print out software version
        SmartDashboard.putString("Build Data", BuildConfig.kDeployTime);
        SmartDashboard.putString("Deploy User", BuildConfig.kDeployUser);
        SmartDashboard.putString("Git Revision", BuildConfig.kGitRevision);

        DriveTrain.setVariant(new FalconDriveTrainVariant());

        scheduler = CommandScheduler.getInstance();

        if (Constants.kDebugCommandScheduler) {
            scheduler.onCommandInitialize(c -> System.out.println("Initializing " + c.getName()));
            scheduler.onCommandInterrupt(c -> System.out.println("Interrupting " + c.getName()));
            scheduler.onCommandFinish(c -> System.out.println("Finishing " + c.getName()));
        }

        // Register subsystems
        scheduler.registerSubsystem(
                DriveTrain.getInstance(),
                Infrastructure.getInstance(),
                Flywheel.getInstance(),
                Feeder.getInstance(),
                Hopper.getInstance(),
                Intake.getInstance()
        );

        if (!Constants.isPracticeRobot()) {
            scheduler.registerSubsystem(Climber.getInstance());
        }

        // Create commands
        disabledCommand = new DisabledCommand();
        teleopCommand = new TeleopCommand();
        autonomousCommand = new AutonomousCommand();

        if (Constants.kUseNotifierForMainLoop) {
            new Notifier(scheduler::run).startPeriodic(0.02);
        }
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     * <p>
     * This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        if (!Constants.kUseNotifierForMainLoop) {
            scheduler.run();
        }
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        scheduler.cancelAll();
        scheduler.schedule(disabledCommand);
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        scheduler.cancelAll();
        scheduler.schedule(autonomousCommand);
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        scheduler.cancelAll();
        scheduler.schedule(teleopCommand);
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        scheduler.cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
