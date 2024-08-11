// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
}
