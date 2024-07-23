// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import com.ctre.phoenix6.hardware.CANcoder;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

/* 
 * ArmSubsystem - Subsystem to control all Arm motion using a Trapezoidal Profiled PID controller
 * 
 * For more details on how this works, see:
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/profilepid-subsystems-commands.html
 *
 */
public class ArmSubsystem extends ProfiledPIDSubsystem {

    TalonFX m_armLead = new TalonFX(CanConstants.ID_ArmLeader);
    TalonFX m_armFollow = new TalonFX(CanConstants.ID_ArmFollower);
    CANcoder m_armEncoder  = new CANcoder(DIOConstants.k_ARM_ENCODER_ID);

    private final ArmFeedforward m_feedforward = new ArmFeedforward(
            ArmConstants.kSVolts, ArmConstants.kGVolts,
            ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

    // Keep track of what the Arm is doing
    //First double is setpoint angle in degrees, second double is tolerance
    @RequiredArgsConstructor
    @Getter
    public enum ArmState {
        STOWED  (()-> -22.0, ()-> 2.0),
        SUBWOOFER(()-> -20.0, ()-> 2.0),
        PODIUM  (()-> -2.0, ()-> 0.5),
        WING    (()-> 10.0, ()-> 0.5), // Specific Wing Shot
        AMP     (()-> 90.0, ()-> 0.5),
        CLIMB   (()-> 95.0, ()-> 1.0),
        HARMONY (()-> 100.0, ()-> 1.0),
        AIMING  (()-> 40, ()-> 2.0),      // Dynamic
        MOVING  (()-> 50, ()-> 2.0),      // Dynamic
        FEED    (()-> 0.0, ()-> 2.0);

        private final DoubleSupplier angleSupplier;
        private final DoubleSupplier toleranceSupplier;

        private double getStateOutput() {
            return angleSupplier.getAsDouble();
        }

        private double getTolerance() {
            return toleranceSupplier.getAsDouble();
        }
    }
    
    @Getter
    @Setter
    ArmState m_ArmState = ArmState.STOWED;

    @Getter
    @Setter
    ArmState m_FutureArm = ArmState.STOWED; // May be marked for depreciation

    public BooleanSupplier isAtState = ()-> false;

    private final NeutralOut m_neutral = new NeutralOut();

    /*
     * Constructor
     */
    public ArmSubsystem() {

        super(
                new ProfiledPIDController(
                        ArmConstants.k_ARM_KP,
                        ArmConstants.k_ARM_KI,
                        ArmConstants.k_ARM_KD,
                        new TrapezoidProfile.Constraints(
                                ArmConstants.kMaxVelocityRadPerSecond,
                                ArmConstants.kMaxAccelerationRadPerSecSquared)),
                0);
        // Start arm at rest in neutral position
        setGoal(ArmConstants.k_ARM_ENCODER_OFFSET_RADIANS);

        // https://v6.docs.ctr-electronics.com/en/latest/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
        var talonFXConfigurator = new TalonFXConfiguration();
        // enable stator current limit
        talonFXConfigurator.CurrentLimits.StatorCurrentLimit = 120;
        talonFXConfigurator.CurrentLimits.StatorCurrentLimitEnable = true;

        // Set brake as neutralmodevalue
        talonFXConfigurator.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Check this out: https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/api-usage/configuration.html
        /*
         * config.supplyCurrLimit.enable = true;
         * config.supplyCurrLimit.triggerThresholdCurrent = 40; // the peak supply
         * current, in amps
         * config.supplyCurrLimit.triggerThresholdTime = 1.5; // the time at the peak
         * supply current before the limit triggers, in sec
         * config.supplyCurrLimit.currentLimit = 30; // the current to maintain if the
         * peak supply limit is triggered
         * m_armLead.configAllSettings(config); // apply the config settings; this
         * selects the quadrature encoder
         * m_armFollow.configAllSettings(config); // apply the config settings; this
         * selects the quadrature encoder
         * 
         * 
         * //ErrorCode error = motor.getLastError(); // gets the last error generated by
         * the motor controller
         * //Faults faults = new Faults();
         * //ErrorCode faultsError = motor.getFaults(faults); // fills faults with the
         * current motor controller faults; returns the last error generated
         * 
         * 
         * /*
         * Apply the configurations to the motors, and set one to follow the other in
         * the same direction
         */
        m_armLead.getConfigurator().apply(talonFXConfigurator);   
        m_armFollow.getConfigurator().apply(talonFXConfigurator);
        m_armFollow.setControl(new Follower(m_armLead.getDeviceID(), true));

    }

    @Override
    public void periodic() {

        // Put the measurement of the arm and state of the arm on shuffleboard
        SmartDashboard.putBoolean("Arm at state?", isArmAtState().getAsBoolean());
        SmartDashboard.putString("Arm state", getM_ArmState().toString());
        SmartDashboard.putNumber("Arm Angle Corrected", getMeasurement());
        SmartDashboard.putNumber("Arm Angle uncorrected", getMeasurement() + ArmConstants.k_ARM_ENCODER_OFFSET_RADIANS);
    }

    @Override
    protected void useOutput(double output, State setpoint) {

        double correctedPosition = setpoint.position - ArmConstants.k_ARM_ENCODER_OFFSET_RADIANS;
        // Calculate the feedforward from the sepoint
        double feedforward = m_feedforward.calculate(correctedPosition, setpoint.velocity);
        // Add the feedforward to the PID output to get the motor output
        m_armLead.setVoltage(output + feedforward);

    }

    @Override
    protected double getMeasurement() {
        //getAbsolutePosition returns in rotations, not radians, hence the x2pi
        // returns in radians
        return (m_armEncoder.getAbsolutePosition().getValueAsDouble()*2*Math.PI) - ArmConstants.k_ARM_ENCODER_OFFSET_RADIANS;

    }

    public BooleanSupplier isArmAtState() {
        if (MathUtil.isNear(Math.toRadians(m_ArmState.getStateOutput()), getMeasurement(), m_ArmState.getTolerance())) {
            isAtState = ()-> true;
            return ()-> true;
        }
        isAtState = ()-> false;
        return ()-> false;
    }

    /* return a command that:
    *  1. Change the Armstate
    *  2. Sets the m_controller's tolerance from parent ProfiledPIDSubsystem class to what the state machine dictates
    *  3. Because the state has just changed, set a new goal state in ProfiledPIDSubsystem
    */
    public Command setStateCommand(ArmState state) {
        return runOnce(() -> this.m_ArmState = state)
        .andThen(()-> this.m_controller.setTolerance(m_ArmState.getTolerance()))
        .andThen(()-> setGoal(m_ArmState.getStateOutput()));
    }
}
