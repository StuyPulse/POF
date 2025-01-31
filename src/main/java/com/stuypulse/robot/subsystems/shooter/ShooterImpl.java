package com.stuypulse.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.ShooterLobFerryInterpolation;
import com.stuypulse.robot.util.ShooterLowFerryInterpolation;
import com.stuypulse.robot.util.ShooterSpeeds;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterImpl extends Shooter {
    
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final SparkMax feederMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private final DigitalInput feederBeam;

    private final Controller leftController;
    private final Controller rightController;

    private final BStream hasNote;

    private final SmartNumber leftTargetRPM;
    private final SmartNumber rightTargetRPM;

    protected ShooterImpl() {
        leftMotor = new SparkMax(Ports.Shooter.LEFT_MOTOR, MotorType.kBrushless);
        Motors.Shooter.LeftMotor.MOTOR.encoder.apply(Motors.Shooter.LeftMotor.ENCODER);
        leftMotor.configure(Motors.Shooter.LeftMotor.MOTOR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightMotor = new SparkMax(Ports.Shooter.RIGHT_MOTOR, MotorType.kBrushless);
        Motors.Shooter.RightMotor.MOTOR.encoder.apply(Motors.Shooter.RightMotor.ENCODER);
        rightMotor.configure(Motors.Shooter.RightMotor.MOTOR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        feederMotor = new SparkMax(Ports.Shooter.FEEDER_MOTOR, MotorType.kBrushless);
        feederMotor.configure(Motors.Shooter.Feeder.MOTOR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        feederBeam = new DigitalInput(Ports.Shooter.RECIEVER_IR);

        leftController = new MotorFeedforward(Settings.Shooter.LEFT.FF.kS, Settings.Shooter.LEFT.FF.kV, Settings.Shooter.LEFT.FF.kA).velocity()
            .add(new PIDController(Settings.Shooter.LEFT.PID.kP, Settings.Shooter.LEFT.PID.kI, Settings.Shooter.LEFT.PID.kD));

        rightController = new MotorFeedforward(Settings.Shooter.RIGHT.FF.kS, Settings.Shooter.RIGHT.FF.kV, Settings.Shooter.RIGHT.FF.kA).velocity()
            .add(new PIDController(Settings.Shooter.RIGHT.PID.kP, Settings.Shooter.RIGHT.PID.kI, Settings.Shooter.RIGHT.PID.kD));
        
        hasNote = BStream.create(feederBeam).not()
            .filtered(new BDebounce.Falling(Settings.Shooter.HAS_NOTE_FALLING_DEBOUNCE))
            .filtered(new BDebounce.Rising(Settings.Shooter.HAS_NOTE_RISING_DEBOUNCE));

        leftTargetRPM = new SmartNumber("Shooter/Left Target RPM", getSpeakerShotSpeeds().getLeftRPM());
        rightTargetRPM = new SmartNumber("Shooter/Right Target RPM", getSpeakerShotSpeeds().getRightRPM());
    }

    private double getLeftShooterRPM() {
        return leftEncoder.getVelocity();
    }

    private double getRightShooterRPM() {
        return rightEncoder.getVelocity();
    }

    @Override
    public boolean atTargetSpeeds() {
        return Math.abs(getLeftShooterRPM() - leftTargetRPM.get()) < Settings.Shooter.TARGET_RPM_THRESHOLD 
            && Math.abs(getRightShooterRPM() - rightTargetRPM.get()) < Settings.Shooter.TARGET_RPM_THRESHOLD;
    }

    private void setTargetSpeeds(ShooterSpeeds speeds) {
        this.leftTargetRPM.set(speeds.getLeftRPM());
        this.rightTargetRPM.set(speeds.getRightRPM());
    }

    private void setFeederBasedOnState() {
        switch (getFeederState()) {
            case INTAKING:
                feederMotor.set(+Settings.Shooter.FEEDER_INTAKE_SPEED);
                break;
            case DEACQUIRING:
                feederMotor.set(-Settings.Shooter.FEEDER_DEAQUIRE_SPEED);
                break;
            case SHOOTING:
                feederMotor.set(Settings.Shooter.FEEDER_SHOOT_SPEED);
                break;
            case STOP:
                feederMotor.set(0);
                break;
            default:
                feederMotor.set(0);
                break;
        }
    }

    private void setFlywheelTargetsBasedOnState() {
        double manualFerryDistance = Units.metersToInches(Field.getManualFerryPosition().getDistance(Field.getAmpCornerPose()));
        switch (getFlywheelState()) {
            case SPEAKER:
                setTargetSpeeds(getSpeakerShotSpeeds());
                break;
            case LOW_FERRY:
                setTargetSpeeds(getLowFerrySpeeds());
                break;
            case LOW_FERRY_MANUAL:
                setTargetSpeeds(new ShooterSpeeds(ShooterLowFerryInterpolation.getRPM(manualFerryDistance)));
                break;
            case LOB_FERRY:
                setTargetSpeeds(getLobFerrySpeeds());
                break;
            case LOB_FERRY_MANUAL:
                setTargetSpeeds(new ShooterSpeeds(ShooterLobFerryInterpolation.getRPM(manualFerryDistance)));
                break;
            case STOP:
                setTargetSpeeds(new ShooterSpeeds());
                break;
            default:
                setTargetSpeeds(new ShooterSpeeds());
                break;
        }
    }

    private ShooterSpeeds getSpeakerShotSpeeds() {
        Pose2d speakerPose = Field.getAllianceSpeakerPose();
        Pose2d robotPose = SwerveDrive.getInstance().getPose();
        double distanceToSpeaker = robotPose.minus(speakerPose).getTranslation().getNorm() - Settings.LENGTH / 2;
        // return new ShooterSpeeds(
        //     4000 + SLMath.clamp(distanceToSpeaker - 1.5, 0, Double.MAX_VALUE) * 600,
        //     500
        // );
        if (distanceToSpeaker <= 1.5) {
            return new ShooterSpeeds(4000, 500);
        }
        else {
            return new ShooterSpeeds(5500, 500);
        }
    }

    @Override
    public boolean hasNote() {
        return hasNote.get();
    }

    private ShooterSpeeds getLowFerrySpeeds() {
        Translation2d ferryZone = Robot.isBlue()
            ? new Translation2d(0, Field.WIDTH - 1.5)
            : new Translation2d(0, 1.5);
        
        double distanceToFerryInInches = Units.metersToInches(SwerveDrive.getInstance().getPose().getTranslation().getDistance(ferryZone));
        
        double targetRPM = ShooterLobFerryInterpolation.getRPM(distanceToFerryInInches);
        return new ShooterSpeeds(targetRPM, 500);
    }

    private ShooterSpeeds getLobFerrySpeeds() {
        Translation2d ferryZone = Robot.isBlue()
            ? new Translation2d(0, Field.WIDTH - 1.5)
            : new Translation2d(0, 1.5);
        
        double distanceToFerryInInches = Units.metersToInches(SwerveDrive.getInstance().getPose().getTranslation().getDistance(ferryZone));

        double targetRPM = ShooterLowFerryInterpolation.getRPM(distanceToFerryInInches);
        return new ShooterSpeeds(targetRPM + 500, 500);
    }

    @Override
    public void periodic () {
        super.periodic();

        setFeederBasedOnState();
        setFlywheelTargetsBasedOnState();

        leftController.update(leftTargetRPM.get(), getLeftShooterRPM());
        leftMotor.set(leftController.getOutput());

        rightController.update(rightTargetRPM.get(), getRightShooterRPM());
        rightMotor.set(rightController.getOutput());

        SmartDashboard.putNumber("Shooter/Feeder Speed", feederMotor.get());

        SmartDashboard.putNumber("Shooter/Left Voltage", leftMotor.getBusVoltage());
        SmartDashboard.putNumber("Shooter/Right Voltage", rightMotor.getBusVoltage());

        SmartDashboard.putBoolean("Shooter/Has Note", hasNote());

        SmartDashboard.putNumber("Shooter/Left RPM", getLeftShooterRPM());
        SmartDashboard.putNumber("Shooter/Right RPM", getRightShooterRPM());

        SmartDashboard.putNumber("Shooter/Left Voltage", leftMotor.getBusVoltage() * leftMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter/Right Voltage", rightMotor.getBusVoltage() * rightMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter/Feeder Voltage", feederMotor.getBusVoltage() * feederMotor.getAppliedOutput());

        SmartDashboard.putNumber("Shooter/Left Current", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/Right Current", rightMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/Feeder Current", feederMotor.getOutputCurrent());
    }

}