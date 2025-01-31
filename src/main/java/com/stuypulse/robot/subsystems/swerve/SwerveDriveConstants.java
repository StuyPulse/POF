package com.stuypulse.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstantsFactory;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

public class SwerveDriveConstants {

    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(Settings.Swerve.Turn.kP.get()).withKI(Settings.Swerve.Turn.kI.get()).withKD(Settings.Swerve.Turn.kD.get())
        .withKS(Settings.Swerve.Turn.kS.get()).withKV(Settings.Swerve.Turn.kV.get()).withKA(Settings.Swerve.Turn.kA.get());
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(Settings.Swerve.Drive.kP.get()).withKI(Settings.Swerve.Drive.kI.get()).withKD(Settings.Swerve.Drive.kD.get())
        .withKS(Settings.Swerve.Drive.kS.get()).withKV(Settings.Swerve.Drive.kV.get()).withKA(Settings.Swerve.Drive.kA.get());

    // Initial configs for the drive and steer motors and the CANcoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                // Swerve azimuth does not require much torque output, so we can set a relatively low
                // stator current limit to help avoid brownouts without impacting performance.
                .withStatorCurrentLimit(60)
                .withStatorCurrentLimitEnable(true)
        );
    
    private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();

    // Configs for the Pigeon 2; 
    // leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    public static final LegacySwerveDrivetrainConstants DrivetrainConstants = new LegacySwerveDrivetrainConstants()
            .withCANbusName(Settings.Swerve.CAN_BUS_NAME)
            .withPigeon2Id(Ports.Gyro.PIGEON2)
            .withPigeon2Configs(pigeonConfigs);

    private static final LegacySwerveModuleConstantsFactory ConstantCreator = new LegacySwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(Settings.Swerve.Drive.GEAR_RATIO)
            .withSteerMotorGearRatio(Settings.Swerve.Turn.GEAR_RATIO)
            .withWheelRadius(Settings.Swerve.Drive.WHEEL_DIAMETER / 2)
            .withSlipCurrent(Settings.Swerve.SLIP_CURRENT)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
            .withSpeedAt12VoltsMps(Settings.Swerve.SPEED_AT_12_VOLTS)
            .withSteerInertia(Settings.Swerve.Simulation.TURN_INERTIA)
            .withDriveInertia(Settings.Swerve.Simulation.DRIVE_INERTIA)
            .withSteerFrictionVoltage(Settings.Swerve.Simulation.TURN_FRICTION_VOLTAGE)
            .withDriveFrictionVoltage(Settings.Swerve.Simulation.DRIVE_FRICTION_VOLTAGE)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(3.5714285714285716)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs);
            // .withCANcoderInitialConfigs(cancoderInitialConfigs);

    public static final LegacySwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
        Ports.Swerve.FrontLeft.TURN, 
        Ports.Swerve.FrontLeft.DRIVE, 
        Ports.Swerve.FrontLeft.ENCODER, 
        Settings.Swerve.FrontLeft.ABSOLUTE_OFFSET.getRotations(), 
        Settings.Swerve.FrontLeft.MODULE_OFFSET.getX(), 
        Settings.Swerve.FrontLeft.MODULE_OFFSET.getY(), 
        Settings.Swerve.FrontLeft.DRIVE_INVERTED
    );

    public static final LegacySwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
        Ports.Swerve.FrontRight.TURN, 
        Ports.Swerve.FrontRight.DRIVE, 
        Ports.Swerve.FrontRight.ENCODER, 
        Settings.Swerve.FrontRight.ABSOLUTE_OFFSET.getRotations(), 
        Settings.Swerve.FrontRight.MODULE_OFFSET.getX(), 
        Settings.Swerve.FrontRight.MODULE_OFFSET.getY(), 
        Settings.Swerve.FrontRight.DRIVE_INVERTED
    );

    public static final LegacySwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
        Ports.Swerve.BackLeft.TURN, 
        Ports.Swerve.BackLeft.DRIVE, 
        Ports.Swerve.BackLeft.ENCODER, 
        Settings.Swerve.BackLeft.ABSOLUTE_OFFSET.getRotations(), 
        Settings.Swerve.BackLeft.MODULE_OFFSET.getX(), 
        Settings.Swerve.BackLeft.MODULE_OFFSET.getY(), 
        Settings.Swerve.BackLeft.DRIVE_INVERTED
    );
    
    public static final LegacySwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
        Ports.Swerve.BackRight.TURN, 
        Ports.Swerve.BackRight.DRIVE, 
        Ports.Swerve.BackRight.ENCODER, 
        Settings.Swerve.BackRight.ABSOLUTE_OFFSET.getRotations(), 
        Settings.Swerve.BackRight.MODULE_OFFSET.getX(), 
        Settings.Swerve.BackRight.MODULE_OFFSET.getY(), 
        Settings.Swerve.BackRight.DRIVE_INVERTED
    );

    public static final double UpdateOdometryFrequency = 50 /* hz */;
}