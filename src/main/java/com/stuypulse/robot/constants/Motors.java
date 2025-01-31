package com.stuypulse.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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

    /** Classes to store all of the values a motor needs */

    public interface Arm {
        public interface LeftMotor {
            SparkBaseConfig MOTOR = new SparkMaxConfig().smartCurrentLimit(40).openLoopRampRate(0.35).inverted(false).idleMode(IdleMode.kBrake);
            EncoderConfig ENCODER = new EncoderConfig().positionConversionFactor(Settings.Arm.Encoder.GEAR_RATIO).velocityConversionFactor(Settings.Arm.Encoder.GEAR_RATIO);
        }
        public interface RightMotor {
            SparkBaseConfig MOTOR = new SparkMaxConfig().smartCurrentLimit(40).openLoopRampRate(0.35).inverted(true).idleMode(IdleMode.kBrake);
        }
    }

    public interface Intake {
        public interface LeftFunnel {
            SparkBaseConfig MOTOR = new SparkMaxConfig().smartCurrentLimit(60).openLoopRampRate(0.35).inverted(false).idleMode(IdleMode.kCoast);
        }
        public interface RightFunnel {
            SparkBaseConfig MOTOR = new SparkMaxConfig().smartCurrentLimit(60).openLoopRampRate(0.35).inverted(true).idleMode(IdleMode.kCoast);
        }
        public interface Rollers {
            SparkBaseConfig MOTOR = new SparkMaxConfig().smartCurrentLimit(60).openLoopRampRate(0.25).inverted(true).idleMode(IdleMode.kCoast);
        }
    }

    public interface Shooter {
        public interface LeftMotor {
            SparkBaseConfig MOTOR = new SparkMaxConfig().smartCurrentLimit(40).openLoopRampRate(0.5).inverted(true).idleMode(IdleMode.kCoast);
            EncoderConfig ENCODER = new EncoderConfig().velocityConversionFactor(1.2);
        }
        public interface RightMotor {
            SparkBaseConfig MOTOR = new SparkMaxConfig().smartCurrentLimit(40).openLoopRampRate(0.5).inverted(false).idleMode(IdleMode.kCoast);
            EncoderConfig ENCODER = new EncoderConfig().velocityConversionFactor(1.0);
        }
        public interface Feeder {
            SparkBaseConfig MOTOR = new SparkMaxConfig().smartCurrentLimit(40).openLoopRampRate(0.25).inverted(false).idleMode(IdleMode.kBrake);
        }
    }
}
