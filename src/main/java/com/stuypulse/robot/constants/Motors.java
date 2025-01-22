package com.stuypulse.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.SparkMax;
import com.revrobotics.SparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.stuypulse.robot.constants.Motors.TalonSRXConfig.CANSparkConfig;
import com.revrobotics.CANSparkBase;

/*-
 * File containing all of the configurations that different motors require.
 *
 * Such configurations include:
 *  - If it is Inverted
 *  - The Idle Mode of the Motor
 *  - The Current Limit
 *  - The Open Loop Ramp Rate
 */
public interface Motors {

    public enum StatusFrame {
        APPLIED_OUTPUT_FAULTS,
        MOTOR_VEL_VOLTS_AMPS,
        MOTOR_POSITION,
        ANALOG_SENSOR,
        ALTERNATE_ENCODER,
        ABS_ENCODER_POSIITION,
        ABS_ENCODER_VELOCITY
    }

    public static void disableStatusFrames(CANSparkBase motor, StatusFrame... ids) {
        final int kDisableStatusFrame = 500;

        for (StatusFrame id : ids) {
            motor.setPeriodicFramePeriod(PeriodicFrame.fromId(id.ordinal()), kDisableStatusFrame);
        }
    }

    /** Classes to store all of the values a motor needs */

    public interface Arm {
        CANSparkConfig LEFT_MOTOR = new CANSparkConfig(false, IdleMode.kBrake, 40, 0.35, true); 
        CANSparkConfig RIGHT_MOTOR = new CANSparkConfig(true, IdleMode.kBrake, 40, 0.35, true); 
    }

    public interface Intake {
        CANSparkConfig LEFT_FUNNEL_MOTOR_CONFIG = new CANSparkConfig(false, IdleMode.kCoast, 60, 0.35, false);
        CANSparkConfig RIGHT_FUNNEL_MOTOR_CONFIG = new CANSparkConfig(true, IdleMode.kCoast, 60, 0.35, false);
        CANSparkConfig INTAKE_MOTOR_CONFIG = new CANSparkConfig(true, IdleMode.kCoast, 60, 0.25, false);
    }

    public interface Shooter {
        CANSparkConfig LEFT_SHOOTER = new CANSparkConfig(true, IdleMode.kCoast, 40, 0.5, false);
        CANSparkConfig RIGHT_SHOOTER = new CANSparkConfig(false, IdleMode.kCoast, 40, 0.5, false);
        CANSparkConfig FEEDER_MOTOR = new CANSparkConfig(true, IdleMode.kBrake, 40, 0.25, false);
    }
  
    /* Configurations */
    
    public static class TalonSRXConfig {
        public final boolean INVERTED;
        public final NeutralMode NEUTRAL_MODE;
        public final int PEAK_CURRENT_LIMIT_AMPS;
        public final double OPEN_LOOP_RAMP_RATE;

        public TalonSRXConfig(
                boolean inverted,
                NeutralMode neutralMode,
                int peakCurrentLimitAmps,
                double openLoopRampRate) {
            this.INVERTED = inverted;
            this.NEUTRAL_MODE = neutralMode;
            this.PEAK_CURRENT_LIMIT_AMPS = peakCurrentLimitAmps;
            this.OPEN_LOOP_RAMP_RATE = openLoopRampRate;
        }

        public TalonSRXConfig(boolean inverted, NeutralMode neutralMode, int peakCurrentLimitAmps) {
            this(inverted, neutralMode, peakCurrentLimitAmps, 0.0);
        }

    public static void disableStatusFrames(CANSparkBase motor, StatusFrame... ids) {
        final int kDisableStatusFrame = 500;

        for (StatusFrame id : ids) {
            motor.setPeriodicFramePeriod(PeriodicFrame.fromId(id.ordinal()), kDisableStatusFrame);
        }
    }

    public static class CANSparkConfig {
        public final boolean INVERTED;
        public final IdleMode IDLE_MODE;
        public final int CURRENT_LIMIT_AMPS;
        public final double OPEN_LOOP_RAMP_RATE;
        public final boolean ENABLE_VOLTAGE_COMPENSATION;

        public CANSparkConfig(
                boolean inverted,
                IdleMode idleMode,
                int currentLimitAmps,
                double openLoopRampRate,
                boolean enableVoltageCompensation) {
            this.INVERTED = inverted;
            this.IDLE_MODE = idleMode;
            this.CURRENT_LIMIT_AMPS = currentLimitAmps;
            this.OPEN_LOOP_RAMP_RATE = openLoopRampRate;
            this.ENABLE_VOLTAGE_COMPENSATION = enableVoltageCompensation;
        }

        public CANSparkConfig(boolean inverted, IdleMode idleMode, int currentLimitAmps, boolean enableVoltageCompensation) {
            this(inverted, idleMode, currentLimitAmps, 0.0, enableVoltageCompensation);
        }

        public CANSparkConfig(boolean inverted, IdleMode idleMode, boolean enableVoltageCompensation) {
            this(inverted, idleMode, 80, enableVoltageCompensation);
        }

        public void configure(CANSparkBase motor) {
            motor.setInverted(INVERTED);
            motor.setIdleMode(IDLE_MODE);
            motor.setSmartCurrentLimit(CURRENT_LIMIT_AMPS);
            motor.setOpenLoopRampRate(OPEN_LOOP_RAMP_RATE);
            if (ENABLE_VOLTAGE_COMPENSATION) {
                motor.enableVoltageCompensation(12);
            }
            motor.burnFlash();
        }

        public void configureAsFollower(CANSparkMax motor, CANSparkMax follows) {
            motor.setInverted(INVERTED);
            motor.setIdleMode(IDLE_MODE);
            motor.setSmartCurrentLimit(CURRENT_LIMIT_AMPS);
            motor.setOpenLoopRampRate(OPEN_LOOP_RAMP_RATE);
            if (ENABLE_VOLTAGE_COMPENSATION) {
                motor.enableVoltageCompensation(12);
            }
            motor.follow(follows);
            motor.burnFlash();
        }
    }
}
}
