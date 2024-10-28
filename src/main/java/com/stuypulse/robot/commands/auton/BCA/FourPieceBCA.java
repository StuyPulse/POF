package com.stuypulse.robot.commands.auton.BCA;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmSetState;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.arm.ArmToSubwooferShot;
import com.stuypulse.robot.commands.auton.FollowPathThenShoot;
import com.stuypulse.robot.commands.auton.ShootRoutine;
import com.stuypulse.robot.commands.intake.IntakeSetAcquire;
import com.stuypulse.robot.commands.shooter.ShooterFeederShoot;
import com.stuypulse.robot.commands.shooter.ShooterFeederStop;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FourPieceBCA extends SequentialCommandGroup {
    public FourPieceBCA(PathPlannerPath... paths) {
        addCommands(
            // Preload Shot
            ShootRoutine.fromSubwoofer(),
            new ArmToFeed(),

            // Drive to B + Shoot B
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[0]),
            new FollowPathThenShoot(paths[1], false),
            new ArmToFeed(),

            // Drive to C + Shoot C
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[2]),
            new FollowPathThenShoot(paths[3], false),
            new ArmToFeed(),

            // Drive to A + Shoot A
            new IntakeSetAcquire(),
            new FollowPathThenShoot(paths[4], true),
            new ArmToFeed()
        );
    }

}
