package com.stuypulse.robot.commands.swerve.noteAssist;

import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;
import com.stuypulse.stuylib.util.AngleVelocity;
import com.stuypulse.stuylib.util.StopWatch;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Driver.Turn;
import com.stuypulse.robot.constants.Settings.Swerve.Assist;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.NoteVision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveDriveNoteAssist extends Command {

    private enum Mode {
        NORMAL,
        ASSIST
    }

    private final SwerveDrive swerve;
    private final NoteVision noteVision;

    private final SwerveRequest.RobotCentric robotCentricDrive;

    // used to represent how long the robot hasnt seen the note
    private final StopWatch stopWatch;

    private final Gamepad driver;

    private Mode mode;

    private final VStream speed;
    private final IStream driverTurn;
    private final IStream assistAngleVelocity;
    private final AngleController angleController;

    private Rotation2d lastAngleToNoteRobotRelative;

    public SwerveDriveDriveNoteAssist(Gamepad driver) {
        swerve = SwerveDrive.getInstance();
        noteVision = NoteVision.getInstance();

        robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(Settings.Swerve.MAX_LINEAR_VELOCITY * Settings.Driver.Drive.DEADBAND.get())
            .withRotationalDeadband(Settings.Swerve.MAX_ANGULAR_VELOCITY * Settings.Driver.Turn.DEADBAND.get())
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

        stopWatch = new StopWatch();

        speed = VStream.create(driver::getLeftStick)
            .filtered(
                new VDeadZone(Drive.DEADBAND),
                x -> x.clamp(1),
                x -> x.pow(Drive.POWER.get()),
                x -> x.mul(Drive.MAX_TELEOP_SPEED.get()),
                new VRateLimit(Drive.MAX_TELEOP_ACCEL),
                new VLowPassFilter(Drive.RC));

        driverTurn = IStream.create(driver::getRightX)
            .filtered(
                x -> SLMath.deadband(x, Turn.DEADBAND.get()),
                x -> SLMath.spow(x, Turn.POWER.get()),
                x -> x * Turn.MAX_TELEOP_TURN_SPEED.get(),
                new LowPassFilter(Turn.RC));

        angleController = new AnglePIDController(Motion.THETA.kP, Motion.THETA.kI, Motion.THETA.kD)
            .setOutputFilter(x -> -x);

        lastAngleToNoteRobotRelative = new Rotation2d();

        AngleVelocity derivative = new AngleVelocity();
        assistAngleVelocity = IStream.create(() -> derivative.get(Angle.fromRotation2d(mode == Mode.ASSIST ? lastAngleToNoteRobotRelative : new Rotation2d())))
            .filtered(new LowPassFilter(Assist.ANGLE_DERIV_RC))
            // make angleVelocity contribute less once distance is less than REDUCED_FF_DIST
            // so that angular velocity doesn't oscillate
            .filtered(x -> x * Math.min(1, (mode == Mode.ASSIST ? noteVision.getRobotRelativeNotePose().getNorm() : 0) / Assist.REDUCED_FF_DIST))
            .filtered(x -> -x);

        this.driver = driver;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        if (noteVision.hasNoteData()) {
            Translation2d notePose = noteVision.getRobotRelativeNotePose();
            if (notePose.getNorm() < Settings.NoteDetection.INTAKE_THRESHOLD_DISTANCE && Math.abs(notePose.getAngle().getDegrees()) < Settings.NoteDetection.MAX_ANGLE) {
                mode = Mode.ASSIST;
                stopWatch.reset();
                lastAngleToNoteRobotRelative = notePose.getAngle();
            }
            else {
                mode = Mode.NORMAL;
            }
        }
        else if (stopWatch.getTime() > 1.0) {
            mode = Mode.NORMAL;
        }

        switch (mode) {
            case NORMAL:
                swerve.drive(speed.get(), driverTurn.get());
                break;
            case ASSIST:
                swerve.setControl(robotCentricDrive
                    .withVelocityX(speed.get().magnitude() * Math.cos(lastAngleToNoteRobotRelative.getRadians()))
                    .withVelocityY(speed.get().magnitude() * Math.sin(lastAngleToNoteRobotRelative.getRadians()))
                    .withRotationalRate(assistAngleVelocity.get() + angleController.update(
                        Angle.fromDegrees(0),
                        Angle.fromRotation2d(lastAngleToNoteRobotRelative)))
                );
                break;
            default:
                break;
        }
    }
}
