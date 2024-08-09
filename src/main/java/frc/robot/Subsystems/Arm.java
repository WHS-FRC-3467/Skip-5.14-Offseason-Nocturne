// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Subsystems.Shooter.ShooterState;
import frc.robot.Util.ShooterPreset;
import frc.robot.Util.TunableNumber;
import frc.robot.Util.VisionLookUpTable;
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
public class Arm extends ProfiledPIDSubsystem {

    /** Arm subsystem singleton. For superstructure. */
    private static Arm instance = null;

    TalonFX m_armLead = new TalonFX(CanConstants.ID_ArmLeader);
    TalonFX m_armFollow = new TalonFX(CanConstants.ID_ArmFollower);
    //CANcoder m_armEncoder  = new CANcoder(DIOConstants.k_ARM_ENCODER_ID);
    DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(DIOConstants.k_ARM_ENCODER_ID);

    private final ArmFeedforward m_feedforward = new ArmFeedforward(
            ArmConstants.kSVolts, ArmConstants.kGVolts,
            ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

    // Keep track of what the Arm is doing
    //First double is setpoint angle in degrees, second double is tolerance
    @RequiredArgsConstructor
    @Getter
    public enum ArmState {
        STOWED  (()-> -17.0, ()-> 2.0),
        SUBWOOFER(()-> -7.6, ()-> 1.0),
        PODIUM  (()-> 6.0, ()-> 0.4),
        WING    (()-> 10.0, ()-> 0.4), // Specific Wing Shot
        AMP     (()-> 77.0, ()-> 0.4),
        CLIMB   (()-> 71.0, ()-> 1),
        HARMONY (()-> 105.0, ()-> 1),
        AIMING  (()-> 0, ()-> 1.0),      // Dynamic - Used for aiming - tje angle supplier is just a filler value
        FEED    (()-> -3.0, ()-> 2.0);

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

    // Distance for aiming
    public DoubleSupplier m_distance = ()-> -0.1;

    // Limit the amount of degrees that the arm can go
    private double lowerLimit = -18.0;
    private double upperLimit = 106.0;

    // Arm Angle Adjustment via Shuffleboard
    @Getter
    private TunableNumber tempDegree = new TunableNumber("Set Arm To Degrees", 0.0);

    private final NeutralOut m_neutral = new NeutralOut();

    // For superstructure
    /**
    * Returns the arm subsystem instance.
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

        super(
                new ProfiledPIDController(
                        ArmConstants.k_ARM_KP,
                        ArmConstants.k_ARM_KI,
                        ArmConstants.k_ARM_KD,
                        new TrapezoidProfile.Constraints(
                                ArmConstants.kMaxVelocityRadPerSecond,
                                ArmConstants.kMaxAccelerationRadPerSecSquared)),
                0);

        // https://v6.docs.ctr-electronics.com/en/latest/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
        var talonFXConfigurator = new TalonFXConfiguration();
        /* Arm: enable stator current limit */
        talonFXConfigurator.CurrentLimits.StatorCurrentLimit = 60; // Limit stator to 60 amps
        talonFXConfigurator.CurrentLimits.StatorCurrentLimitEnable = true; // enable the limiting
        /* Arm Supply Current limit of 30 amps */
        talonFXConfigurator.CurrentLimits.SupplyCurrentLimit = 30;
        talonFXConfigurator.CurrentLimits.SupplyCurrentThreshold = 85; // If we exceed 50 amps
        talonFXConfigurator.CurrentLimits.SupplyTimeThreshold = 0.1; // For at least 0.1 second
        talonFXConfigurator.CurrentLimits.SupplyCurrentLimitEnable = true; // enable supply current limiting

        // Set BRAKE as neutralmodevalue - USE COAST WHEN TESTING FOR OFFSETS - KEEP THE ROBOT DISABLED
        talonFXConfigurator.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Check this out: https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/api-usage/configuration.html
        /*
         * Apply the configurations to the motors, and set one to follow the other in
         * the same direction
         */
        m_armLead.getConfigurator().apply(talonFXConfigurator);   
        m_armFollow.getConfigurator().apply(talonFXConfigurator);
        m_armFollow.setControl(new Follower(m_armLead.getDeviceID(), true));

        m_armEncoder.setDutyCycleRange(ArmConstants.kDuty_Cycle_Min, ArmConstants.kDuty_Cycle_Max);
    }

    @Override
    public void periodic() {

        // Put the measurement of the arm and state of the arm on shuffleboard
        SmartDashboard.putBoolean("Arm at state?", isArmAtState().getAsBoolean());
        SmartDashboard.putString("Arm state", getM_ArmState().toString());
        SmartDashboard.putNumber("Arm Angle Corrected", Units.radiansToDegrees(getMeasurement()));
        SmartDashboard.putNumber("Arm Angle uncorrected", m_armEncoder.getAbsolutePosition()*360.0);
        SmartDashboard.putBoolean("Arm at tempDegree?", isArmAtTempSetpoint().getAsBoolean());
        
       
        // If testing arm angle through Tunablenumber tempDegree, set the arm to the manually desired angle
        if (Constants.RobotConstants.kIsArmTuningMode) {
            // Prevent arm from going too low or high
            if (tempDegree.get() > upperLimit) {
                tempDegree.set(upperLimit);
            } else if (tempDegree.get() < lowerLimit) {
                tempDegree.set(lowerLimit);
            }
            // Apply controls
            m_controller.setTolerance(m_ArmState.getTolerance()); 
            m_controller.setGoal(Units.degreesToRadians(tempDegree.get()));
            useOutput(m_controller.calculate(getMeasurement()), m_controller.getSetpoint());
        }
        
        else if ((m_ArmState == ArmState.AIMING)) {
            // If distance is less than 0 then distance value for aiming is invalid
            if (m_distance.getAsDouble() > 0.0) {
                VisionLookUpTable m_LookUpTable = new VisionLookUpTable();
                ShooterPreset m_shot = m_LookUpTable.getShooterPreset(m_distance.getAsDouble());
                m_controller.setGoal(Units.degreesToRadians(m_shot.getArmAngle()));
                useOutput(m_controller.calculate(getMeasurement()), m_controller.getSetpoint());
            }
        } else {
            // The "regular" case
            m_controller.setGoal(Units.degreesToRadians(m_ArmState.getStateOutput()));
            useOutput(m_controller.calculate(getMeasurement()), m_controller.getSetpoint());     
        }

    }

    @Override
    protected void useOutput(double output, State setpoint) {

        double correctedPosition = setpoint.position - ArmConstants.kARM_STARTING_OFFSET;
        // Calculate the feedforward from the sepoint
        double feedforward = m_feedforward.calculate(correctedPosition, setpoint.velocity);
        // Add the feedforward to the PID output to get the motor output
        m_armLead.setVoltage(output + feedforward);

    }

    @Override
    protected double getMeasurement() {
        //getAbsolutePosition returns in rotations, not radians, hence the x2pi
        // returns in radians
        return ((m_armEncoder.getAbsolutePosition() - m_armEncoder.getPositionOffset())*2*Math.PI) - ArmConstants.k_ARM_HORIZONTAL_OFFSET_RADIANS;

    }

    public BooleanSupplier isArmAtState() {
        if (MathUtil.isNear(Math.toRadians(m_ArmState.getStateOutput()), getMeasurement(), Math.toRadians(m_ArmState.getTolerance()))) {
            isAtState = ()-> true;
            return ()-> true;
        }
        isAtState = ()-> false;
        return ()-> false;
    }

    public BooleanSupplier isArmAtTempSetpoint() {
        if (MathUtil.isNear(Math.toRadians(tempDegree.get()), getMeasurement(), Math.toRadians(m_ArmState.getTolerance()))) {
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
        return startEnd(() -> this.m_ArmState = state, () -> this.m_ArmState = ArmState.STOWED);

    }

    /*
     * When aiming, setStateCommand() to AIMING for Arm and Shooter
     * At the same time, setDistanceToTarget() for Arm and Shooter equal to distance reported by PhotonVision
     */
    public Command setDistanceToTarget(double distance) {
        return startEnd(() -> this.m_distance = ()-> distance, () -> this.m_distance = () -> -0.1);
    }
}
