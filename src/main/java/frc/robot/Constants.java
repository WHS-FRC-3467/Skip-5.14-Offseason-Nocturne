// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class CanConstants {
        public static final int k_INTAKE_LEFT_CAN_ID = 19;
        public static final int k_INTAKE_RIGHT_CAN_ID = 20;
        public static final int k_STAGE_CAN_ID = 17;
    }

    public static class DIOConstants {
        public static final int k_INTAKE_BEAM_BREAK = 0;
        public static final int k_ARM_ENCODER_ID = 1;

    }

    public static class RobotConstants {
        public static final boolean kIsTuningMode = true;
        public static final boolean k_shouldUseLimelight = true;
    }

    public static class IntakeConstants {
        public static final double k_INTAKE_FWD_SPEED = 0.8;
        public static final double k_INTAKE_REV_SPEED = -0.8;
    }

    public static class ShooterConstants {
        // This enum is used to keep track of what the shooter is and should be doing.
        // Example: ShooterState currentShooterState = ShooterState.SPOOLING;
        public enum ShooterState {
            STOP,
            SPOOLING,
            REVVING,
            READY,
            REVERSE
        }
        public static final double k_DEFAULT_FWD_VELOCITY = 0.8;
        public static final double k_SHOOTER_REV_VELOCITY = -0.8;
        public static final double k_SHOOTER_VELOCITY_TOLERANCE = 5;
    }

    public static class StageConstants {
        public static final double k_STAGE_VELOCITY = 0.8;
        public static final double k_STAGE_REV_VELOCITY = -0.8;        
        public static final int k_STAGE_VELOCITY_TOLERANCE = 10;
    }

    public static class ArmConstants {

        public static final double kSVolts = 0.5;
        public static final double kGVolts = 0;
        public static final double kVVoltSecondPerRad = 60;
        public static final double kAVoltSecondSquaredPerRad = 100;

        public static final double k_ARM_KP = 5;
        public static final double k_ARM_KI = 0;
        public static final double k_ARM_KD = 1;

        public static final double kMaxVelocityRadPerSecond = 200;
        public static final double kMaxAccelerationRadPerSecSquared = 200;

        public static final double k_ARM_ENCODER_OFFSET_RADIANS = 0;

        public enum ArmState {
            STOWED,
            SUBWOOFER,
            PODIUM,
            WING,
            AMP,
            CLIMB,
            HARMONY,
            AIMING,
            MOVING,
            FEED
        }

    }

    public static class PhotonVisionConstants {

        // Constants such as camera and target height stored. Change per robot and goal!
        public final static double LEFT_CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
        public final static double RIGHT_CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
        public final static double TARGET_HEIGHT_METERS = Units.feetToMeters(5);

        // Angle between horizontal and the camera.
        public final static double LEFT_CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
        public final static double RIGHT_CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
    }
}
