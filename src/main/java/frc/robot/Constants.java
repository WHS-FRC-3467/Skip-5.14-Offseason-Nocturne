// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
/** Add your docs here. */
public class Constants {


    public static final class ExampleCTREMotorConfig { 
        public static final int ID_Motor = 0;

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0; 

            m_configuration.CurrentLimits.SupplyCurrentLimit = 20; 
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1; 
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true; 
            m_configuration.CurrentLimits.StatorCurrentLimit = 70;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true; 

            return m_configuration;
        }
    }
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final double triggerThreshold = 0.4;

    }


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

    public static class RobotConstants {
        public static final boolean kIsTuningMode = true;
        public static final boolean kIsDriveTuningMode = true;
        public static final boolean kIsArmTuningMode = true; // Only make this true for arm testing thorugh shuffleboard
        public static final boolean kIsIntakeTuningMode = true;
        public static final boolean kIsStageTuningMode = true;
        public static final boolean kIsShooterTuningMode = true;
        public static final boolean k_shouldUseLimelight = true;
    }

    public static class ArmConstants {

            // Arm CAN IDs
        public static final int ID_ArmLeader = 25;
        public static final int ID_ArmFollower = 26;    
            // Arm DIO Constant
        public static final int k_ARM_ENCODER_ID = 0;

        public static TalonFXConfiguration motorConfig() {
                // Documentation: https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/api-usage/configuration.html
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();
                // Set BRAKE as neutralmodevalue - USE COAST WHEN TESTING FOR OFFSETS - KEEP THE ROBOT DISABLED
            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            m_configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0; 
                /* Arm Supply Current limit of 30 amps */
            m_configuration.CurrentLimits.SupplyCurrentLimit = 30; 
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 85;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1; 
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true; 
            m_configuration.CurrentLimits.StatorCurrentLimit = 60;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true; 

            return m_configuration;
        }

        public static final double kSVolts = 0.5; // Static Gain (volts)
        public static final double kGVolts = 0.4; // Gravity Gain (volts)
        public static final double kVVoltSecondPerRad = 2.5; // Velocity Gain (volt seconds/radian)
        public static final double kAVoltSecondSquaredPerRad = .01; // Acceleration Gain (volt seconds^2/radian)

        public static final double k_ARM_KP = 18.0; // P Gain - Volts
        public static final double k_ARM_KI = 0.0; // I Gain - Volts
        public static final double k_ARM_KD = 0.21; // D Gain - Volts

        public static final double kMaxVelocityRadPerSecond = 2.0;  // originally 4
        public static final double kMaxAccelerationRadPerSecSquared = 5.0; // originally 10

        // Observed Arm Offsets
        // Measured against the hardstop when the Arm is in the STOWED position. Divide by 360 to convert from degrees to duty cycle
        public static final double k_ARM_HORIZONTAL_OFFSET_DUTYCYCLE = 180.4/360; // 180.4 deg when measured against the hardstop (stowed)

        public static final double kDuty_Cycle_Min = 1.0/1025.0;
        public static final double kDuty_Cycle_Max = 1024.0/1025.0;
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
