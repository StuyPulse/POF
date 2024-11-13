package com.stuypulse.robot;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.stuypulse.robot.commands.AutoDrive;
import com.stuypulse.robot.commands.leds.LEDReset;
import com.stuypulse.robot.commands.vision.VisionReloadWhiteList;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private RobotContainer robot;
    private CommandScheduler scheduler;
    private Command auto;

    /*************************/
    /*** ROBOT SCHEDULEING ***/
    /*************************/

    @Override
    public void robotInit() {
        Pathfinding.setPathfinder(new LocalADStar());

        DataLogManager.start();

        scheduler = CommandScheduler.getInstance();

        robot = new RobotContainer();

        scheduler.schedule(PathfindingCommand.warmupCommand());

        if (Robot.isReal()) CameraServer.startAutomaticCapture().setVideoMode(PixelFormat.kMJPEG, 80, 60, 30);

        SmartDashboard.putString("Robot State", "DISABLED");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    public static boolean isBlue() {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
    }

    /*********************/
    /*** DISABLED MODE ***/
    /*********************/

    @Override
    public void disabledInit() {
        SmartDashboard.putString("Robot State", "DISABLED");
    }

    @Override
    public void disabledPeriodic() {
        // reload whitelist in case of alliance change
        CommandScheduler.getInstance().schedule(new VisionReloadWhiteList());
    }

    /***********************/
    /*** AUTONOMOUS MODE ***/
    /***********************/  

    @Override
    public void autonomousInit() {
        auto = robot.getAutonomousCommand();

        if (auto != null) {
            auto.schedule();
        }

        scheduler.schedule(new LEDReset());

        SmartDashboard.putString("Robot State", "AUTON");
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    /*******************/
    /*** TELEOP MODE ***/
    /*******************/

    @Override
    public void teleopInit() {
        if (auto != null) {
            auto.cancel();
        }
        // scheduler.schedule(new AutoDrive());
        SmartDashboard.putString("Robot State", "TELEOP");
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    /*****************/
    /*** TEST MODE ***/
    /*****************/

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
