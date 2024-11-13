package com.stuypulse.robot.commands.auton.ADEF;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.auton.FollowPathThenShoot;
import com.stuypulse.robot.commands.auton.ShootRoutine;
import com.stuypulse.robot.commands.intake.IntakeSetAcquire;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ReroutableFivePieceADEF extends SequentialCommandGroup {
    
    public ReroutableFivePieceADEF(PathPlannerPath... paths) {

        PathPlannerPath D_To_E_Reroute = paths[6];
        PathPlannerPath E_To_F_Reroute = paths[7];

        addCommands(
            
            // Preload Shot
            ShootRoutine.fromAnywhere(),
            new ArmToFeed(),

            // Drive to A + Shoot A
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[0]),
            ShootRoutine.fromAnywhere(),
            new ArmToFeed(),

            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[1]),
            
            new ConditionalCommand(

                // If D is successfully intaken
                new SequentialCommandGroup(
                    
                    new FollowPathThenShoot(paths[2], false),
                    new ArmToFeed(),
    
                    // Drive to E + Shoot E
                    new IntakeSetAcquire(),
                    SwerveDrive.getInstance().followPathCommand(paths[3]),
                    new FollowPathThenShoot(paths[4], false),
                    new ArmToFeed(),
    
                    // Drive to F + Shoot F
                    new IntakeSetAcquire(),
                    SwerveDrive.getInstance().followPathCommand(paths[5]),
                    new FollowPathThenShoot(paths[6], true),
                    new ArmToFeed()),
                
                // If D is not intaken
                new SequentialCommandGroup(
                    // Redirect from D to E
                    new IntakeSetAcquire(),
                    SwerveDrive.getInstance().followPathCommand(D_To_E_Reroute),

                    // If E is successfully intaken
                    new ConditionalCommand(

                        new SequentialCommandGroup(
                            // Shoot E
                            new FollowPathThenShoot(paths[4], false),
                            new ArmToFeed(),
    
                            // Drive to F + Shoot F
                            new IntakeSetAcquire(),
                            SwerveDrive.getInstance().followPathCommand(paths[5]),
                            new FollowPathThenShoot(paths[6], true),
                            new ArmToFeed()
                        ),

                        // If E is not intaken
                        new SequentialCommandGroup(
                            // Redirect from E to F
                            new IntakeSetAcquire(),
                            SwerveDrive.getInstance().followPathCommand(E_To_F_Reroute),

                            // Shoot F
                            new FollowPathThenShoot(paths[6], true),
                            new ArmToFeed()
                        ),

                        // Runs E to F redirection if hasNote is false at F
                        Intake.getInstance()::hasNote
                    )
                ),

                // Runs D to E redirection if hasNote is false at D
                Intake.getInstance()::hasNote 
                
            )

        );
    }

}
