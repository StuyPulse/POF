package com.stuypulse.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.stuypulse.robot.constants.Ports;

public class IntakeImpl extends Intake {

    private final CANSparkMax funnelMotorLeft;
    private final CANSparkMax funnelMotorRight;
    private final CANSparkMax intakeMotor;

    private final DigitalInput IRSensor;

    private final BStream hasNote;

    public IntakeImpl() {
        super();
        funnelMotorLeft = new CANSparkMax(Ports.Intake.FUNNEL_LEFT, MotorType.kBrushless);
        funnelMotorRight = new CANSparkMax(Ports.Intake.FUNNEL_RIGHT, MotorType.kBrushless);        
        intakeMotor = new CANSparkMax(Ports.Intake.INTAKE_MOTOR, MotorType.kBrushless);

        Motors.Intake.LEFT_FUNNEL_MOTOR_CONFIG.configure(funnelMotorLeft);
        Motors.Intake.RIGHT_FUNNEL_MOTOR_CONFIG.configure(funnelMotorRight);
        Motors.Intake.INTAKE_MOTOR_CONFIG.configure(intakeMotor);

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
            intakeMotor.set(Settings.Intake.INTAKE_DEACQUIRE_SPEED);
            funnelMotorLeft.stopMotor();
            funnelMotorRight.stopMotor();
        }
        else if (feedingTimer.getTime() > 1.5) {
            feedingTimer.reset();
        }
        else {
            intakeMotor.set(Settings.Intake.INTAKE_DEACQUIRE_SPEED);
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