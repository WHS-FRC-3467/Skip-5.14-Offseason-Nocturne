// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


/** Add your docs here. */
public class Constants {

    public static final class ArmConstants { 
        public static final int ID_ArmLeader = 25;
        public static final int ID_ArmFollower = 26;  
        public static final int ID_ArmAbsEncoder = 0;

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            m_configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            m_configuration.MotorOutput.DutyCycleNeutralDeadband = .005;
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0; 

            m_configuration.CurrentLimits.SupplyCurrentLimit = 30; 
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 50;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1; 
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true; 
            m_configuration.CurrentLimits.StatorCurrentLimit = 60;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true; 

            return m_configuration;
        }
    }

    public static final class IntakeConstants { 
        public static final int ID_IntakeMotor = 19;
        public static final int ID_IntakeFollower = 21;

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

    public static final class ShooterConstants { 
        public static final int ID_ShooterLeft = 15;
        public static final int ID_ShooterRight = 17;

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            m_configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0; 

            m_configuration.CurrentLimits.SupplyCurrentLimit = 60; 
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 80;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1; 
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true; 
            m_configuration.CurrentLimits.StatorCurrentLimit = 70;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true; 

            m_configuration.Slot0.kP = .03;
            m_configuration.Slot0.kI = 0;
            m_configuration.Slot0.kD = 0;
            m_configuration.Slot0.kV = .125;



            return m_configuration;
        }

        public static TalonFXConfiguration motor2Config() {
            TalonFXConfiguration m_configuration = motorConfig();
            m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            return m_configuration;
        }
    }

    public static final class StageConstants { 
        // Stage CAN IDs
        public static final int ID_StageMotor = 23;
        public static final int ID_StageBeamBreak = 1;

/*         public static TalonFXConfiguration StageConfig() {
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



        } */
    }

    public static final class ControllerConstants {

        public static final double leftTriggerMin = 0.3;
        public static final double leftTriggerMid = 0.6;

    }

}

