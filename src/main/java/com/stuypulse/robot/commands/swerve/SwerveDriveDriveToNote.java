package com.stuypulse.robot.commands.swerve;

import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.util.StopWatch;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.notes.NoteVision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveDriveToNote extends Command {

    private final SwerveDrive swerve;
    private final NoteVision noteVision;

    private final SwerveRequest.RobotCentric robotCentricDrive;

    // used to represent how long the robot hasnt seen the note
    private final StopWatch stopWatch;

    private final AngleController angleController;

    private Rotation2d lastAngleToNoteRobotRelative;

    public SwerveDriveDriveToNote() {
        swerve = SwerveDrive.getInstance();
        noteVision = NoteVision.getInstance();

        robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(Settings.Swerve.MAX_LINEAR_VELOCITY * Settings.Driver.Drive.DEADBAND.get())
            .withRotationalDeadband(Settings.Swerve.MAX_ANGULAR_VELOCITY * Settings.Driver.Turn.DEADBAND.get())
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

        stopWatch = new StopWatch();

        angleController = new AnglePIDController(Motion.THETA.kP, Motion.THETA.kI, Motion.THETA.kD)
            .setOutputFilter(x -> -x);

        lastAngleToNoteRobotRelative = new Rotation2d();

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        if (noteVision.hasNoteData()) {
            lastAngleToNoteRobotRelative = noteVision.getRotationToNote();
            stopWatch.reset();
        }
        swerve.setControl(robotCentricDrive
            .withVelocityX(2.0 * Math.cos(lastAngleToNoteRobotRelative.getRadians()))
            .withVelocityY(2.0 * Math.sin(lastAngleToNoteRobotRelative.getRadians()))
            .withRotationalRate(angleController.update(
                Angle.fromDegrees(0),
                Angle.fromRotation2d(lastAngleToNoteRobotRelative))
            )
        );
    }

    @Override
    public boolean isFinished() {
        return Intake.getInstance().hasNote() || (!noteVision.hasNoteData() && stopWatch.getTime() > 1.0);
    }

    @Override
    public void end(boolean interrupted) {
        // stop just in case
        swerve.setControl(robotCentricDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }
}
