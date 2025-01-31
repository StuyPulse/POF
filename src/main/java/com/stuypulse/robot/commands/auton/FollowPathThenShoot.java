package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmToSpeaker;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class FollowPathThenShoot extends SequentialCommandGroup{

    private final double totalPathTime;
    
    public FollowPathThenShoot(PathPlannerPath path, boolean isLastShot) {
        try {
            totalPathTime = path.generateTrajectory(new ChassisSpeeds(), path.getPathPoses().get(0).getRotation(), RobotConfig.fromGUISettings()).getTotalTimeSeconds();
            addCommands(
                new ParallelCommandGroup(
                    SwerveDrive.getInstance().followPathCommand(path),
                    new WaitCommand(totalPathTime > 1.5 ? totalPathTime - 1.0 : 0)
                        // wait for handoff
                        .andThen(new WaitUntilCommand(() -> Shooter.getInstance().hasNote()).withTimeout(2.0).onlyIf(() -> Intake.getInstance().hasNote()))
                ),
                isLastShot ? ShootRoutine.fromAnywhereLastShot()
                        : ShootRoutine.fromAnywhere().onlyIf(() -> Shooter.getInstance().hasNote())
            );
        }
        catch (Exception e) {
            throw new IllegalStateException("RobotConfig.fromGUISettings() did not work");
        }
    }
}
