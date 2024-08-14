// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class Constants {


    public static final class RobotConstants {

        public static final boolean kIsTuningMode = true;
        public static final boolean kIsDriveTuningMode = false;
        public static final boolean kIsArmTuningMode = true;
        public static final boolean kIsIntakeTuningMode = false;
        public static final boolean kIsStageTuningMode = true;
        public static final boolean kIsShooterTuningMode = true;
        public static final boolean kIsAutoAimTuningMode = true;
        public static final boolean kIsTrapTuningMode = false;
    
        // Shooter and Arm Setpoints

        public static final double robotAtAngleTolerance = 2;
        public static final double robotMaxStaticShotDist = 5;
        public static final double robotMaxDynamicShotDist = 4.5;
        public static final double robotMaxFeedShotDist = 15;

        public static final double kTimeToShoot = .15; //Time it takes before note leave the shooter
        public static final double kTimeToReady = .5;   // Past number was 0.5 
    }

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

    public static final class CanConstants {
        public static final int ID_ShooterLeft = 15;
        public static final int ID_ShooterRight = 17;

        // Intake CAN IDs
        public static final int ID_IntakeMotor = 19;
        public static final int ID_IntakeCtrRoller = 20;
        public static final int ID_IntakeFollower = 21;

        // Stage CAN IDs
        public static final int ID_StageMotor = 23;

        // Arm CAN IDs
        public static final int ID_ArmLeader = 25;
        public static final int ID_ArmFollower = 26;    
        public static final int LED_CANDLE = 27;
    }

    public static final class StageConstants {

        public static final double kIntakeSpeed = 0.3;
        public static final double kFeedToShooterSpeed = 1.0;
        public static final double kFeedToAmpSpeed = 0.7;
        public static final double kFeedToTrapSpeed = 0.5;

        public static final double kFeedToShooterTime = 0.5;
        public static final double kFeedToAmpTime = 1.0;
        public static final double kFeedToTrapTime = 5.0;
    }

    public static final class DIOConstants {

        public static final int kArmAbsEncoder = 0;
        public static final int kStageBeamBreak = 1;
    }
}
