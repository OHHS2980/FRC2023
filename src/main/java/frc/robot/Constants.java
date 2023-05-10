package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
public final class Constants {

    public static final class ArmConstants{
        public static final int kArmMotorPort = 9;
        public static final int kArmMotorSlavePort = 10;
        public static final int kLinearMotorPort = 11;
        public static final int kWristMotorPort = 12;

        public static final int kArmSliderPort = 1;
        public static final int kLinearSliderPort = 4;
        public static final int kWristSliderPort = 3;

        public static final int kHighButtonPort = 2;
        public static final int kMidButtonPort = 6;
        public static final int kLowButtonPort = 10;

        public static final int kCubeConeToggleButtonPort = 1;
        public static final int kIntakePlaceToggleButtonPort = 4;
        public static final int kFrontBackToggleButtonPort = 3;
        public static final int kArmResetButtonPort = 2;
        public static final int kResetPosButtonPort = 6;
        public static final int kManualOverrideButtonPort = 5;
    }

    public static final class IntakeConstants{
        public static final int kCubeIntakeMotorPort = 14;
        public static final int kConeIntakeMotorPort = 13;

        public static final int kIntakeTriggerPort = 1;

        
        public static final int kIntakeSliderPort = 6;
        
        public static final int kCubeConeToggleButtonPort = 1;
    }

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 8.31;
        public static final double kTurningMotorGearRatio = 1 / 18.0f;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.25;
        public static final double kITurning = 1e-3;
        public static final double kDTurning = 0;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(18);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(24.5);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2)
        );

        public static final int kFrontLeftDriveMotorPort = 2;
        public static final int kBackLeftDriveMotorPort = 6;
        public static final int kFrontRightDriveMotorPort = 4;
        public static final int kBackRightDriveMotorPort = 8;

        public static final int kFrontLeftTurningMotorPort = 1;
        public static final int kBackLeftTurningMotorPort = 5;
        public static final int kFrontRightTurningMotorPort = 3;
        public static final int kBackRightTurningMotorPort = 7;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true; 
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.641718;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 4.164359;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 3.374253 + Math.PI;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 4.661981 + Math.PI;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 150;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = (25 * 2 * Math.PI);

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;//3.5
        public static final double kTeleDriveMaxAccelerationForwardUnitsPerSecond = 50;
        public static final double kTeleDriveMaxAccelerationSideUnitsPerSecond = 40;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 40;
    }

    public static final class OIConstants {
        public static final int kDriveControllerPort = 0;
        public static final int kTurnControllerPort = 1;
        public static final int kOperatorControllerPort = 2;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 0;
        
        public static final int kDriveSpeedAxis = 2;
        public static final int kTurnSpeedAxis = 2;

        public static final int kDriverFieldOrientedButtonPort = 8;
        public static final int kResetGyroButtonPort = 7;
        public static final int kConeIntakeButtonPort = 4;
        public static final int kCubeIntakeButtonPort = 5;

        public static final int kAngleHoldFWDButtonPort = 3;
        public static final int kAngleHoldBKWDButtonPort = 2;
        public static final int kAngleHoldLFTButtonPort = 4;
        public static final int kAngleHoldRGTButtonPort = 5;

        public static final double kDeadband = 0.2;
    } 

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 8;
        public static final double kMaxAngularSpeedRadiansPerSecond = 4;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final double kAutonomousMode = 2;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);

        public static final double kOnChargeStationDegree = 13.0;
        
        public static final double kLevelDegree = 6.0;
    }

} 