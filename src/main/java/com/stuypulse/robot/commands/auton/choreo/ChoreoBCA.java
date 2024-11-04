package com.stuypulse.robot.commands.auton.choreo;

import com.choreo.lib.ChoreoTrajectory;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.auton.FollowTrajThenShoot;
import com.stuypulse.robot.commands.auton.ShootRoutine;
import com.stuypulse.robot.commands.intake.IntakeSetAcquire;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class ChoreoBCA extends SequentialCommandGroup {
    
    public ChoreoBCA(ChoreoTrajectory... traj) {
        addCommands(
            // Preload Shot
            ShootRoutine.fromSubwoofer(),
            new ArmToFeed(),
            
            // Drive to B + Shoot B
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(traj[0]),
            new FollowTrajThenShoot(traj[1], false),
            new ArmToFeed(),

            // Drive to C + Shoot C
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(traj[2]),
            new FollowTrajThenShoot(traj[3], false),
            new ArmToFeed(),

            // Drive to A + Shoot A
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(traj[4]),
            new FollowTrajThenShoot(traj[5], true),
            new ArmToFeed()

        );
    }

}
