// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Constants.ArmConstants;
import frc.robot.Util.TunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

/* 
 * ArmSubsystem - Subsystem to control all Arm motion using a Trapezoidal Profiled PID controller
 *
 */
public class Arm extends SubsystemBase {

    /** Arm subsystem singleton. For superstructure. */
    private static Arm instance = null;

    // Keep track of what the Arm is doing
    //First double is setpoint angle in degrees, second double is tolerance
    @RequiredArgsConstructor
    @Getter
    public enum ArmState {
        STOWED  (()-> -19.0, ()-> 2.0),
        INTAKE  (()-> -17.0, ()-> 2.0),
        SUBWOOFER(()-> -7.6, ()-> 1.0),
        AMP     (()-> 77.0, ()-> 0.4), // May getDistanceToTarget() for amp and feed
        CLIMB   (()-> 71.0, ()-> 1),
        HARMONY (()-> 105.0, ()-> 1),
        AIMING  (()-> RobotState.getInstance().getShotAngle(), ()-> 0.5),  // Dynamic - Used for aiming
        TUNING  (()-> tempDegree.get(), ()-> 1.0),
        FEED    (()-> -3.0, ()-> 2.0);

        private final DoubleSupplier angleSupplier;
        private final DoubleSupplier toleranceSupplier;
        
        private double getStateOutput() {
            return Units.degreesToRadians(angleSupplier.getAsDouble());
        }

        private double getTolerance() {
            return toleranceSupplier.getAsDouble();
        }
    }
    
    @Getter
    @Setter
    ArmState m_ArmState = ArmState.STOWED;

    public BooleanSupplier isAtState = ()-> false;

        // Limit the amount of degrees that the arm can go
    private double lowerLimit = Units.degreesToRadians(-18.0);
    private double upperLimit = Units.degreesToRadians(106.0);
        
    TalonFX m_armLead = new TalonFX(ArmConstants.ID_ArmLeader);
    TalonFX m_armFollow = new TalonFX(ArmConstants.ID_ArmFollower);
    DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(ArmConstants.k_ARM_ENCODER_ID);

    private final ArmFeedforward m_feedforward = new ArmFeedforward(
            ArmConstants.kSVolts, ArmConstants.kGVolts,
            ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);
    
        // Arm Angle Adjustment via Shuffleboard
             // If testing arm angle through Tunablenumber tempDegree, set the arm to the manually desired angle\r\n" + //
    @Getter
    private static TunableNumber tempDegree = new TunableNumber("Set Arm To Degrees", -17.0);
    private final NeutralOut m_neutral = new NeutralOut();
        // Declare the ProfiledPIDController
    ProfiledPIDController m_controller;
        // Declare the goalAngle of the Arm - intermediate step
    double goalAngle;
        // Declare variable that determines whether to display info on SmartDashboard
    boolean debugging = false;

    /**
    * Creates a new Arm.
    *
    * @return the arm subsystem instance.
    */
    public static Arm getInstance() {
        if (instance == null) {
        instance = new Arm();
        }

        return instance;
    }

    /*
     * Constructor
     */
    public Arm() {

        m_controller = new ProfiledPIDController(
                        ArmConstants.k_ARM_KP,
                        ArmConstants.k_ARM_KI,
                        ArmConstants.k_ARM_KD,
                        new TrapezoidProfile.Constraints( //Bryson: lower these speeds
                                ArmConstants.kMaxVelocityRadPerSecond,
                                ArmConstants.kMaxAccelerationRadPerSecSquared));

        /*
         * Apply the configurations to the motors, and set one to follow the other in
         * the same direction
         */
        m_armLead.getConfigurator().apply(ArmConstants.motorConfig());
        m_armFollow.getConfigurator().apply(ArmConstants.motorConfig());
        m_armFollow.setControl(new Follower(m_armLead.getDeviceID(), true));
            /* Set range of duty cycle encoder in fractions of rotation */
        m_armEncoder.setDutyCycleRange(ArmConstants.kDuty_Cycle_Min, ArmConstants.kDuty_Cycle_Max);

            // Position offset in duty cycle, not rads
        m_armEncoder.setPositionOffset(ArmConstants.k_ARM_HORIZONTAL_OFFSET_DUTYCYCLE);
        // this sets the distance per rotation to be equal to 2pi radians
        m_armEncoder.setDistancePerRotation(Math.PI*2);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putData("Tuning Arm command", setStateCommand(ArmState.TUNING));
        SmartDashboard.putData("Debugging Arm command", setDebugCommand());

        displayInfo(debugging);
       
        if ((m_ArmState == ArmState.STOWED) && (m_controller.atGoal())) {
            // If the arm is already stowed, hold it using neutral mode (Coast or brake)
            m_armLead.setControl(m_neutral);
        } else {
            // The "regular" case
            goalAngle = MathUtil.clamp(m_ArmState.getStateOutput(), lowerLimit, upperLimit);
            m_controller.setTolerance(m_ArmState.getTolerance()); 
            m_controller.setGoal(goalAngle);
                // Use Output
            // Calculate the feedforward from the controller setpoint
            double feedforward = m_feedforward.calculate(m_controller.getSetpoint().position, m_controller.getSetpoint().velocity);
            // Add the feedforward to the PID output to get the motor output
            m_armLead.setVoltage(m_controller.calculate(m_armEncoder.getDistance()) + feedforward);

        }

    }

    public boolean isAtGoal() {
        // m_armEncoder.getDistance() gets the position in radians (which is 2pi * duty cycle units)
        // MathUtil.isNear(Math.toRadians(m_ArmState.getStateOutput()), m_armEncoder.getDistance(), Math.toRadians(m_ArmState.getTolerance()))
        return m_controller.atGoal();
    }

    /* return a command that:
    *  1. Change the Armstate
    *  2. Sets the m_controller's tolerance from parent ProfiledPIDSubsystem class to what the state machine dictates
    *  3. Because the state has just changed, set a new goal state in ProfiledPIDSubsystem
    */
    public Command setStateCommand(ArmState state) {
        return startEnd(() -> this.m_ArmState = state, () -> this.m_ArmState = ArmState.STOWED);

    }
    
    /* return a command that
     * When debugging button in pressed...
     * change from not debugging to debugging or vice versa
     */
    private Command setDebugCommand() {
        return runOnce(()-> debugging = !debugging);
    }
    // Put the measurement of the arm and state of the arm on shuffleboard
    public void displayInfo(boolean debug) {
        if (debug) {
            SmartDashboard.putBoolean("Arm at state?", m_controller.atGoal());  // formerly isArmAtState().getAsBoolean()
            SmartDashboard.putString("Arm state", getM_ArmState().toString());
            SmartDashboard.putNumber("Arm Setpoint", m_ArmState.getStateOutput());
            SmartDashboard.putNumber("Arm Angle Corrected", Units.radiansToDegrees(m_armEncoder.getDistance())); // Get distance returns radians
            SmartDashboard.putNumber("Arm Angle uncorrected", m_armEncoder.getAbsolutePosition()*360.0); // Displays angle in degrees
        }
    }
}