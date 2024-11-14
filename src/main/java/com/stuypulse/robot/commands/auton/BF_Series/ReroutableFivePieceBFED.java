package com.stuypulse.robot.commands.auton.BF_Series;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.auton.FollowPathThenShoot;
import com.stuypulse.robot.commands.auton.ShootRoutine;
import com.stuypulse.robot.commands.intake.IntakeSetAcquire;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ReroutableFivePieceBFED extends SequentialCommandGroup {
    
    public ReroutableFivePieceBFED(PathPlannerPath... paths) {

        PathPlannerPath F_To_E_Reroute = paths[7];
        PathPlannerPath E_To_D_Reroute = paths[8];

        addCommands(
            
            // Preload Shot
            ShootRoutine.fromAnywhere(),
            new ArmToFeed(),

            // Drive to B + Shoot B
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[0]),
            ShootRoutine.fromAnywhere(),
            new ArmToFeed(),

            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[1]),
            new WaitCommand(3),
            
            new ConditionalCommand(

                // If F is successfully intaken
                new SequentialCommandGroup(
                    
                    // Shoot F
                    new FollowPathThenShoot(paths[2], false),
                    new ArmToFeed(),
    
                    // Drive to E + Shoot E
                    new IntakeSetAcquire(),
                    SwerveDrive.getInstance().followPathCommand(paths[3]),
                    new FollowPathThenShoot(paths[4], false),
                    new ArmToFeed(),
    
                    // Drive to D + Shoot D
                    new IntakeSetAcquire(),
                    SwerveDrive.getInstance().followPathCommand(paths[5]),
                    new FollowPathThenShoot(paths[6], true),
                    new ArmToFeed()),
                
                // If F is not intaken
                new SequentialCommandGroup(
                    // Redirect from F to E
                    new IntakeSetAcquire(),
                    SwerveDrive.getInstance().followPathCommand(F_To_E_Reroute),
                    new WaitCommand(0.5),

                    new ConditionalCommand(
                    
                    // If E is successfully intaken
                    new SequentialCommandGroup(
                        // Shoot E
                        new FollowPathThenShoot(paths[4], false),

                        // E Shoot to D
                        new IntakeSetAcquire(),
                        SwerveDrive.getInstance().followPathCommand(paths[5]),

                        // Shoot D
                        new FollowPathThenShoot(paths[6],true)

                    ), 
                    
                    // If E is not intaken
                    new SequentialCommandGroup(
                        new IntakeSetAcquire(),
                        SwerveDrive.getInstance().followPathCommand(E_To_D_Reroute),
                        new FollowPathThenShoot(paths[6], true)
                    ), 
                    
                    Intake.getInstance()::hasNote

                )
                ),

                // Runs F to E redirection if hasNote is false at F
                Intake.getInstance()::hasNote
                )
        );
    }

}
