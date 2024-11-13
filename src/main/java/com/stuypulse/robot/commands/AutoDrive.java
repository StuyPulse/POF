package com.stuypulse.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.auton.ShootRoutine;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveToNote;
import com.stuypulse.robot.commands.swerve.SwervePathFind;
import com.stuypulse.robot.constants.Ports.Swerve;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.notes.NoteVision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class AutoDrive extends RepeatCommand{

    public AutoDrive() {
        super(
            SwervePathFind.toPickup()
                // .until(() -> NoteVision.getInstance().hasNoteData() && NoteVision.getInstance().noteIsUsable())
                // should make it randomly move around instead of waiting to see a note
                // .andThen(new WaitUntilCommand(() -> NoteVision.getInstance().hasNoteData() && NoteVision.getInstance().noteIsUsable()))
                // .andThen(new SwerveDriveDriveToNote())
                // .repeatedly().until(() -> Intake.getInstance().hasNote())
                .andThen(SwervePathFind.toShoot())
                // .andThen(new WaitUntilCommand(() -> Shooter.getInstance().hasNote())
                //     .onlyIf(() -> Intake.getInstance().hasNote()))
                .andThen(ShootRoutine.fromAnywhere())
                .andThen(new ArmToFeed())
        );
    }
}
