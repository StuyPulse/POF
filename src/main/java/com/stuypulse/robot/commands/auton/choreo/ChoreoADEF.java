package com.stuypulse.robot.commands.auton.choreo;

import com.choreo.lib.ChoreoTrajectory;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.auton.FollowTrajThenShoot;
import com.stuypulse.robot.commands.auton.ShootRoutine;
import com.stuypulse.robot.commands.intake.IntakeSetAcquire;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ChoreoADEF extends SequentialCommandGroup {
    
    public ChoreoADEF(ChoreoTrajectory... traj) {
        
        addCommands(
            // Preload Shot
            ShootRoutine.fromAnywhere(),
            new ArmToFeed(),

            // Drive to A + Shoot A
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(traj[0]),
            ShootRoutine.fromAnywhere(),
            new ArmToFeed(),

            // Drive to D + Shoot D
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(traj[1]),
            new FollowTrajThenShoot(traj[2], false),
            new ArmToFeed(),

            // Drive to E + Shoot E
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(traj[3]),
            new FollowTrajThenShoot(traj[4], false),
            new ArmToFeed(),

            // Drive to F + Shoot F
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(traj[5]),
            new FollowTrajThenShoot(traj[6], true),
            new ArmToFeed()
        );
    }

}
