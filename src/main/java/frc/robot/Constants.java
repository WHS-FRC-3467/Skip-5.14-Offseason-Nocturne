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
        public static final int kOperatorControllerPort = 1;
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
        public static final boolean kIsArmTuningMode = true;
        public static final boolean kIsIntakeTuningMode = true;
        public static final boolean kIsStageTuningMode = true;
        public static final boolean kIsShooterTuningMode = true;
        public static final boolean k_shouldUseLimelight = true;
    }

    public static class IntakeConstants {
        public static final double k_INTAKE_FWD_SPEED = 0.5;
        public static final double k_INTAKE_REV_SPEED = -0.3;
    }

    public static class ShooterConstants {

        public static final double k_DEFAULT_FWD_VELOCITY = 15;
        public static final double k_SHOOTER_REV_VELOCITY = -8;
        public static final double k_SHOOTER_VELOCITY_TOLERANCE = 5;

        public static final double kSVolts = 0.1;
        public static final double kVVoltSecondsPerRotation = 0.12;
        public static final double kP = 0.11;
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
        public static final double k_ARM_KD = 0.2; // D Gain - Volts

        public static final double kMaxVelocityRadPerSecond = 4.0;
        public static final double kMaxAccelerationRadPerSecSquared = 10.0;

        public static final double k_ARM_ENCODER_OFFSET_RADIANS = 0; // 161.4 deg when measured against the hardstop (stowed)

    }

    public static class PhotonVisionConstants {

        // Constants such as camera and target height stored. Change per robot and goal!
        public final static double LEFT_CAMERA_HEIGHT_METERS = Units.inchesToMeters(8.693);
        public final static double RIGHT_CAMERA_HEIGHT_METERS = 0.53;
        public final static double TARGET_HEIGHT_METERS = Units.feetToMeters(5);

        // Angle between horizontal and the camera.
        public final static double LEFT_CAMERA_PITCH_RADIANS = Units.degreesToRadians(-25);
        public final static double RIGHT_CAMERA_PITCH_RADIANS = Units.degreesToRadians(-40);
    }
}
