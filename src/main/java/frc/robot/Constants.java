// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
        public static final int kOperatorControllerPort = 1;
        public static final double triggerThreshold = 0.4;

    }

    public static class CanConstants {
        /* PDH Power Ports
        0. Front Left Drive
        1. Front Left Steer
        2. Left Shooter
        3. Back Left Steer
        4. Back Left Drive
        5. Mini Power Module
        6. Left Arm Motor
        7. N/A
        8. Intake
        9. N/A
        10. N/A
        11. Right Arm
        12. Candle
        13. N/A
        14. Back Right Steer
        15. Back Right Drive
        16. Front Right Drive
        17. Front Right Steer
        18. Right Shooter Motor
        19. Stage Motor
        20. Future Limelight
        21. VRM
        22. RIO
        23. N/A
        30. Blower Motor
         * 
         * 
         */

        // Drivebase CAN IDs are 1 -> 13
        // See generated/TunerConstants.java
        // Shooter CAN IDs
        public static final int ID_ShooterLeft = 15;
        public static final int ID_ShooterRight = 17;

        // Intake CAN IDs
        public static final int k_INTAKE_LEFT_CAN_ID = 19;
        public static final int k_INTAKE_RIGHT_CAN_ID = 21;

        //Stage CAN IDs
        public static final int k_STAGE_CAN_ID = 23;

        // Arm CAN IDs
        public static final int ID_ArmLeader = 25;
        public static final int ID_ArmFollower = 26;    
        public static final int LED_CANDLE = 27;
    }

    public static class DIOConstants {
        public static final int k_INTAKE_BEAM_BREAK = 1;
        public static final int k_ARM_ENCODER_ID = 0;

    }

    public static class RobotConstants {
        public static final boolean kIsTuningMode = true;
        public static final boolean kIsDriveTuningMode = true;
        public static final boolean kIsArmTuningMode = true; // Only make this true for arm testing thorugh shuffleboard
        public static final boolean kIsIntakeTuningMode = true;
        public static final boolean kIsStageTuningMode = true;
        public static final boolean kIsShooterTuningMode = true;
        public static final boolean k_shouldUseLimelight = true;
    }

    public static class IntakeConstants {
        public static final double k_INTAKE_FWD_SPEED = -0.6;
        public static final double k_INTAKE_REV_SPEED = 0.3;
    }

    public static class ShooterConstants {

        public static final double k_DEFAULT_FWD_VELOCITY = 15;
        public static final double k_SHOOTER_REV_VELOCITY = -8;
        public static final double k_SHOOTER_VELOCITY_TOLERANCE = 5;

        public static final double kSVolts = 0.1;
        public static final double kVVoltSecondsPerRotation = 0.125;
        public static final double kP = 0.3;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public static class StageConstants {
        public static final double k_STAGE_VELOCITY = 0.3;
        public static final double kFeedToShooterSpeed = 1.0;
        public static final double kFeedToAmpSpeed = 0.7;

        public static final double kFeedToShooterTime = 0.5;
        public static final double kFeedToAmpTime = 1.0;

        public static final double k_STAGE_REV_VELOCITY = -0.8;        
        public static final int k_STAGE_VELOCITY_TOLERANCE = 50;
    }

    public static class ArmConstants {

        public static final double kSVolts = 0.5; // Static Gain (volts)
        public static final double kGVolts = 0.4; // Gravity Gain (volts)
        public static final double kVVoltSecondPerRad = 2.5; // Velocity Gain (volt seconds/radian)
        public static final double kAVoltSecondSquaredPerRad = .01; // Acceleration Gain (volt seconds^2/radian)

        public static final double k_ARM_KP = 18.0; // P Gain - Volts
        public static final double k_ARM_KI = 0.0; // I Gain - Volts
        public static final double k_ARM_KD = 0.21; // D Gain - Volts

        public static final double kMaxVelocityRadPerSecond = 4.0;
        public static final double kMaxAccelerationRadPerSecSquared = 10.0;

        // Observed Arm Offsets
        // Measured against the hardstop when the Arm is in the STOWED position
        public static final double kARM_STARTING_OFFSET = Units.degreesToRadians(161.4); // marked for depreciation
        public static final double k_ARM_HORIZONTAL_OFFSET_RADIANS = Units.degreesToRadians(180.4); // 180.4 deg when measured against the hardstop (stowed)

        public static final double kDuty_Cycle_Min = 1.0/1025.0;
        public static final double kDuty_Cycle_Max = 1024.0/1025.0;
    }

    public static class PhotonVisionConstants {

        // Constants such as camera and target height stored. Change per robot and goal!
        public final static double LEFT_CAMERA_HEIGHT_METERS = Units.inchesToMeters(8.693);
        public final static double RIGHT_CAMERA_HEIGHT_METERS = 0.53;
        public final static double TARGET_HEIGHT_METERS = Units.feetToMeters(5);

        // Angle between horizontal and the camera.
        public final static double LEFT_CAMERA_PITCH_RADIANS = Units.degreesToRadians(-25);
        public final static double RIGHT_CAMERA_PITCH_RADIANS = Units.degreesToRadians(-40);

        public static class front_left_cam {
            public static final String kCameraName = "front_left";
            public static final Transform3d kRobotToCam = new Transform3d(
                    new Translation3d(Units.inchesToMeters(9.812), Units.inchesToMeters(9.29),
                            Units.inchesToMeters(8.693)),
                    new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(0)));
        }

        public static class top_right_cam {
            public static final String kCameraName = "top_right";
            public static final Transform3d kRobotToCam = new Transform3d(
                    new Translation3d(.159, -.213,.53),
                    new Rotation3d(0, Units.degreesToRadians(-40), Units.degreesToRadians(0)));
        }

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        
    }
    
    public static class FieldConstants{
        // Assume that Blue is the Origin for everything. See https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#rotation-conventions
        public static final Pose2d BLUE_SPEAKER = new Pose2d(Units.inchesToMeters(-1.5 + 12), Units.inchesToMeters(218.42), new Rotation2d(0));
        public static final Pose2d RED_SPEAKER = new Pose2d(Units.inchesToMeters(652.73 - 12), Units.inchesToMeters(218.42), new Rotation2d(Math.PI));
        public static final Pose2d BLUE_FEED = new Pose2d(1.25, 6.62, new Rotation2d(0));
        public static final Pose2d RED_FEED = new Pose2d(15.250, 6.62, new Rotation2d(0));
        public static final Pose2d BLUE_AMP = new Pose2d(Units.inchesToMeters(72.5),Units.inchesToMeters(323.00),new Rotation2d(Math.PI/2));
        public static final Pose2d RED_AMP = new Pose2d(Units.inchesToMeters(578.77),Units.inchesToMeters(323.00),new Rotation2d(-Math.PI/2));
        public static final double BLUE_AUTO_PENALTY_LINE = 9; // X distance from origin to center of the robot almost fully crossing the midline
        public static final double RED_AUTO_PENALTY_LINE = 7.4; // X distance from origin to center of the robot almost fully crossing the midline

        public static final Rotation2d ampAngle = new Rotation2d(Math.PI / 2);
    }
}
