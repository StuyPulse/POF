package com.stuypulse.robot.commands.auton.BF_Series;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.auton.FollowPathThenShoot;
import com.stuypulse.robot.commands.auton.ShootRoutine;
import com.stuypulse.robot.commands.intake.IntakeSetAcquire;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SixPieceBFCAD extends SequentialCommandGroup {
    
    public SixPieceBFCAD(PathPlannerPath... paths) {

        addCommands(
            // Preload
            ShootRoutine.fromSubwoofer(),
            new ArmToFeed(),

            // Get B + Shoot B from there
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[0]),
            ShootRoutine.fromAnywhere(),
            new ArmToFeed(),

            // Get F + Shoot F
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[1]),
            new FollowPathThenShoot(paths[2], false),
            new ArmToFeed(),

            // Get C + Shoot C
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[3]),
            new FollowPathThenShoot(paths[4], false),
            new ArmToFeed(),

            // Get A + Shoot A
            new IntakeSetAcquire(),
            new FollowPathThenShoot(paths[5], false),
            new ArmToFeed(),

            // Get D to Shoot D
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[6]),
            new FollowPathThenShoot(paths[7], true),
            new ArmToFeed()
        );

    }

}
