package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.commands.arm.ArmToSpeaker;
import com.stuypulse.robot.commands.arm.ArmToSubwooferShot;
import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.commands.shooter.ShooterFeederShoot;
import com.stuypulse.robot.commands.shooter.ShooterFeederStop;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.commands.swerve.SwerveDriveAlignToSpeaker;
import com.stuypulse.robot.constants.LEDInstructions;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public abstract class ShootRoutine {

    public static Command fromSubwoofer() {
        return new SequentialCommandGroup(
            new ArmToSubwooferShot(),
            new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                .alongWith(new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)),
            new ShooterFeederShoot(),
            new WaitCommand(0.4), // give time for note to leave shooter (should implement a way to check if the note is shot)
            new ShooterFeederStop()
        );
    }

    public static Command fromAnywhere() {
        return new SequentialCommandGroup(
            new ArmToSpeaker(),
            new ParallelCommandGroup(
                new SwerveDriveAlignToSpeaker(),
                new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET),
                new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)
            ),
            new ShooterFeederShoot(),
            new WaitCommand(0.4),
            new ShooterFeederStop()
        );
    }
}