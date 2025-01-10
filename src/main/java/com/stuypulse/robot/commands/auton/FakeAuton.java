package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.arm.ArmToSpeaker;
import com.stuypulse.robot.commands.arm.ArmToSubwooferShot;
import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.intake.IntakeSetAcquire;
import com.stuypulse.robot.commands.shooter.ShooterFeederShoot;
import com.stuypulse.robot.commands.shooter.ShooterFeederStop;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FakeAuton extends SequentialCommandGroup {
    public FakeAuton(PathPlannerPath... paths) {
        addCommands(
            // Preload Shot
            new ArmToSubwooferShot(),
            new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                .alongWith(new ShooterWaitForTarget().withTimeout(1)),
            new ShooterFeederShoot(),
            new WaitCommand(1),
            new ShooterFeederStop(),
            new ArmToFeed(),

            new WaitCommand(2.5),

            // Drive to H + Shoot H
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[0]),

            new WaitCommand(1),
            
            SwerveDrive.getInstance().followPathCommand(paths[1]),
            new ArmToSpeaker(),
            new ParallelCommandGroup(
                new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET),
                new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)
            ),
            new ShooterFeederShoot(),
            new WaitCommand(2),
            new ShooterFeederStop(),
            new ArmToFeed()
        );
    }
}
