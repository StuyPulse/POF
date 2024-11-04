package com.stuypulse.robot.commands.auton.HGF;

import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.auton.FollowPathThenShoot;
import com.stuypulse.robot.commands.auton.ShootRoutine;
import com.stuypulse.robot.commands.intake.IntakeSetAcquire;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FourPieceChoreoHGF extends SequentialCommandGroup {
    
     public FourPieceChoreoHGF(ChoreoTrajectory... traj) {
        addCommands(
            // Preload Shot
            ShootRoutine.fromSubwoofer(),
            new ArmToFeed(),

            // Drive to H + Shoot H
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(traj[0]),
            //new FollowPathThenShoot(traj[1], false),
            new ArmToFeed(),

            // Drive to G + Shoot G
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(traj[2]),
            //new FollowPathThenShoot(traj[3], false),
            new ArmToFeed(),

            // Drive to F + Shoot F
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(traj[4]),
            //new FollowPathThenShoot(traj[5], true),
            new ArmToFeed()
        );
    }

}