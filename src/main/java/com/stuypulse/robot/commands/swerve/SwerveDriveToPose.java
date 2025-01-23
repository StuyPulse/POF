package com.stuypulse.robot.commands.swerve;

import java.util.function.Supplier;

import com.pathplanner.lib.config.PIDConstants;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Alignment.Theta;
import com.stuypulse.robot.constants.Settings.Alignment.XY;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.robot.util.MirrorRotation2d;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveToPose extends Command {
    
     public static SwerveDriveToPose speakerRelative(double angleToSpeaker, double distanceToSpeaker) {
        MirrorRotation2d angle = MirrorRotation2d.fromBlue(
            Rotation2d.fromDegrees(SLMath.clamp(
                angleToSpeaker, 80)));

        double distance = SLMath.clamp(distanceToSpeaker, 1, 5);

        return new SwerveDriveToPose(() -> {
            return new Pose2d(
                Field.getAllianceSpeakerPose().getTranslation()
                    .plus(new Translation2d(distance, angle.get())),
                angle.get());
            }
        );
    }

    private final SwerveDrive swerve;

    private final HolonomicController controller;
    private final Supplier<Pose2d> poseSupplier;
    private final BStream isAligned;
    private final IStream velocityError;

    private final FieldObject2d targetPose2d;

    private double xTolerance;
    private double yTolerance;
    private double thetaTolerance;
    private double velocityTolerance;
    private double maxSpeed;

    private Pose2d targetPose;

    public SwerveDriveToPose(Pose2d targetPose) {
        this(() -> targetPose);
    }

    public SwerveDriveToPose(Supplier<Pose2d> poseSupplier) {
        swerve = SwerveDrive.getInstance();

        this.poseSupplier = poseSupplier;

        targetPose2d = swerve.getField().getObject("Target Pose");
        maxSpeed = 4.9;

        controller = new HolonomicController(
            new PIDController(XY.kP, XY.kI, XY.kD),
            new PIDController(XY.kP, XY.kI, XY.kD),
            new AnglePIDController(Theta.kP, Theta.kI, Theta.kD));

        isAligned = BStream.create(this::isAligned)
            .filtered(new BDebounceRC.Both(Alignment.DEBOUNCE_TIME));

        velocityError = IStream.create(() -> {
            ChassisSpeeds speeds = controller.getError();

            return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).getNorm();
        })
            .filtered(new LowPassFilter(0.05))
            .filtered(x -> Math.abs(x));

        xTolerance = Alignment.X_TOLERANCE.get();
        yTolerance = Alignment.Y_TOLERANCE.get();
        thetaTolerance = Alignment.THETA_TOLERANCE.get();
        velocityTolerance = 0.15;

        addRequirements(swerve);
    }

    public SwerveDriveToPose withTranslationConstants(PIDConstants pid) {
        controller.setTranslationConstants(pid.kP, pid.kI, pid.kD);
        return this;
    }
    
    public SwerveDriveToPose withRotationConstants(PIDConstants pid) {
        controller.setRotationConstants(pid.kP, pid.kI, pid.kD);
        return this;
    }

    public SwerveDriveToPose withTranslationConstants(double p, double i, double d) {
        controller.setTranslationConstants(p, i, d);
        return this;
    }
    
    public SwerveDriveToPose withRotationConstants(double p, double i, double d) {
        controller.setRotationConstants(p, i, d);
        return this;
    }

    public SwerveDriveToPose withTolerance(Number x, Number y, Number theta) {
        xTolerance = x.doubleValue();
        yTolerance = y.doubleValue();
        thetaTolerance = theta.doubleValue();
        return this;
    }

    public SwerveDriveToPose withMaxSpeed(double speed) {
        maxSpeed = speed;
        return this;
    }

    @Override
    public void initialize() {
        targetPose = poseSupplier.get();
        SmartDashboard.putBoolean("AutonAlignment", true);
    }

    private boolean isAligned() {
        return controller.isDone(xTolerance, yTolerance, thetaTolerance)
            && velocityError.get() < velocityTolerance;
    }

    @Override
    public void execute() {
        targetPose2d.setPose(targetPose);
        controller.update(targetPose, swerve.getPose());

        Vector2D speed = new Vector2D(controller.getOutput().vxMetersPerSecond, controller.getOutput().vyMetersPerSecond)
            .clamp(maxSpeed);
        double rotation = SLMath.clamp(controller.getOutput().omegaRadiansPerSecond, Motion.MAX_ANGULAR_VELOCITY.get());
        
        SmartDashboard.putNumber("Alignment/Translation Target Speed", speed.distance());

        if (Math.abs(rotation) < Swerve.ALIGN_OMEGA_DEADBAND.get())
            rotation = 0;

        ChassisSpeeds clamped = new ChassisSpeeds(
            speed.x, speed.y, rotation);
        
        swerve.setChassisSpeeds(clamped);
    }

    @Override
    public boolean isFinished() {
        return isAligned.get();
    }

    @Override
    public void end(boolean interrupted) {
        Field.clearFieldObject(targetPose2d);
        SmartDashboard.putBoolean("AutonAlignment", false);
    }

}
