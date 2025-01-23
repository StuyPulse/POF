package com.stuypulse.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.stuypulse.robot.constants.Ports;

public class IntakeImpl extends Intake {

    private final SparkMax funnelMotorLeft;
    private final SparkMax funnelMotorRight;
    private final SparkMax intakeMotor;

    private final SparkMaxConfig leftConfig;
    private final SparkMaxConfig rightConfig;
    private final SparkMaxConfig intakeConfig;

    private final DigitalInput IRSensor;

    private final BStream hasNote;

    public IntakeImpl() {
        super();
        funnelMotorLeft = new SparkMax(Ports.Intake.FUNNEL_LEFT, MotorType.kBrushless);
        funnelMotorRight = new SparkMax(Ports.Intake.FUNNEL_RIGHT, MotorType.kBrushless);        
        intakeMotor = new SparkMax(Ports.Intake.INTAKE_MOTOR, MotorType.kBrushless);

        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();
        intakeConfig = new SparkMaxConfig();

        funnelMotorLeft.configure(leftConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        funnelMotorRight.configure(rightConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        intakeMotor.configure(intakeConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        IRSensor = new DigitalInput(Ports.Intake.IRSensor);

        hasNote = BStream.create(IRSensor).not()
            .filtered(new BDebounce.Both(Settings.Intake.IR_DEBOUNCE));
    }

    private void acquire() {
        intakeMotor.set(+Settings.Intake.INTAKE_ACQUIRE_SPEED);
        funnelMotorLeft.set(+Settings.Intake.FUNNEL_ACQUIRE);
        funnelMotorRight.set(+Settings.Intake.FUNNEL_ACQUIRE);
    }

    private void shoot() {
        intakeMotor.set(Settings.Intake.INTAKE_SHOOT_SPEED);
        funnelMotorLeft.stopMotor();
        funnelMotorRight.stopMotor();
    }

    private void deacquire() {
        intakeMotor.set(-Settings.Intake.INTAKE_DEACQUIRE_SPEED);
        funnelMotorLeft.set(-Settings.Intake.FUNNEL_DEACQUIRE);
        funnelMotorRight.set(-Settings.Intake.FUNNEL_DEACQUIRE);
    }

    private void stop() {
        intakeMotor.stopMotor();
        funnelMotorLeft.stopMotor();
        funnelMotorRight.stopMotor();
    }

    private void feed() {
        if (feedingTimer.getTime() < 0.75) {
            intakeMotor.set(Settings.Intake.INTAKE_FEED_SPEED);
            funnelMotorLeft.stopMotor();
            funnelMotorRight.stopMotor();
        }
        else if (feedingTimer.getTime() > 1.5) {
            feedingTimer.reset();
        }
        else {
            intakeMotor.set(1.0);
            funnelMotorLeft.stopMotor();
            funnelMotorRight.stopMotor();
        }
    }

    @Override
    public boolean hasNote() {
        return hasNote.get();
    }

    @Override
    public void periodic() {
        super.periodic();

        switch (getState()) {
            case ACQUIRING:
                acquire();
                break;
            case DEACQUIRING:
                deacquire();
                break;
            case FEEDING:
                feed();
                break;
            case SHOOTING:
                shoot();
                break;
            case STOP:
                stop();
                break;
            default:
                stop();
                break;
        }

        SmartDashboard.putBoolean("Intake/Has Note", hasNote());

        SmartDashboard.putNumber("Intake/Left Funnel Current", funnelMotorLeft.getOutputCurrent());
        SmartDashboard.putNumber("Intake/Left Funnel Current", funnelMotorRight.getOutputCurrent());
    }

}