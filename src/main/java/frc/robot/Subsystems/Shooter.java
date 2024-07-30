package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanConstants;
//import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Util.RobotState;
import frc.robot.Util.TunableNumber;
//import frc.robot.sim.PhysicsSim;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Shooter extends SubsystemBase {

    /** Shooter subsystem singleton. For superstructure. */
    private static Shooter instance = null;

    /* Declare ShooterState Variables - This enum is used to keep track of what the shooter is doing
    *   Numbers should be in rotations per second. Left supplier corresponds to left shooter motor, right supplier to right shooter motor
    *   Example: ShooterState currentShooterState = ShooterState.SPOOLING;
    */ 
    @RequiredArgsConstructor
    @Getter
    public enum ShooterState {
        STOP    (()-> 0.0, ()-> 0.0),
        PASSTHROUGH    (()-> 10.0, ()-> 10.0), // Poop & Scoot
        AMP (()-> 40.0, ()-> 40.0), 
        SUBWOOFER(()-> 27.0, ()-> 27.0),
        SHOOT   (()-> 70.0, ()-> 40.0), // Default
        FEED    (()-> 28.0, ()-> 28.0),
        REVERSE (()-> Constants.ShooterConstants.k_SHOOTER_REV_VELOCITY, ()-> Constants.ShooterConstants.k_SHOOTER_REV_VELOCITY); // Hopefully never have to use this irl

        private final DoubleSupplier leftSupplier;
        private final DoubleSupplier rightSupplier;

        private double getLeftStateOutput() {
            return leftSupplier.getAsDouble();
        }

        private double getRightStateOutput() {
            return rightSupplier.getAsDouble();
        }
    }

    @Getter
    @Setter
    ShooterState m_ShooterState = ShooterState.STOP;

    // Initialize devices
    TalonFX m_shooterLeft = new TalonFX(CanConstants.ID_ShooterLeft);
    TalonFX m_shooterRight = new TalonFX(CanConstants.ID_ShooterRight);

    NeutralOut m_neutralOut = new NeutralOut();

    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    // For superstructure
    /**
    * Returns the shooter subsystem instance.
    *
    * @return the shooter subsystem instance.
    */
    public static Shooter getInstance() {
        if (instance == null) {
        instance = new Shooter();
        }

        return instance;
    }

    public Shooter() {
        
        var talonFXConfigurator = new TalonFXConfiguration();
        // enable stator current limit
        talonFXConfigurator.CurrentLimits.StatorCurrentLimit = 120;
        talonFXConfigurator.CurrentLimits.StatorCurrentLimitEnable = true;

        // Set brake as neutralmodevalue
        talonFXConfigurator.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Set direction of motors and apply configs
        talonFXConfigurator.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_shooterLeft.getConfigurator().apply(talonFXConfigurator);   
        
        talonFXConfigurator.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_shooterRight.getConfigurator().apply(talonFXConfigurator);

        // in init function, set slot 0 gains - for Velocity PID
        // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/basic-pid-control.html
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = ShooterConstants.kSVolts; // Add 0.1 V output to overcome static friction
        slot0Configs.kV = ShooterConstants.kVVoltSecondsPerRotation; // A velocity target of 1 rps results in 0.125 V output
        slot0Configs.kP = ShooterConstants.kP; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = ShooterConstants.kI; // no output for integrated error
        slot0Configs.kD = ShooterConstants.kD; // no output for error derivative

        m_shooterLeft.getConfigurator().apply(slot0Configs);
        m_shooterRight.getConfigurator().apply(slot0Configs);

    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Shooter at speed", isShooterAtSpeed());
        SmartDashboard.putString("Shooter state", getM_ShooterState().toString());
        
        // set velocity according to state, add 0.5 V to overcome gravity
        if (m_ShooterState == ShooterState.STOP) {
            m_shooterLeft.setControl(m_neutralOut);
        } else {
            // create a velocity closed-loop request, voltage output, slot 0 configs
            m_shooterRight.setControl(m_request.withVelocity(m_ShooterState.getRightStateOutput()).withFeedForward(0.5));
            m_shooterLeft.setControl(m_request.withVelocity(m_ShooterState.getLeftStateOutput()).withFeedForward(0.5));
        }
    }

    public boolean isShooterAtSpeed() {
        // TalonFX getVelocity() gets the Velocity of the device in mechanism rotations per second
        double leftError = Math.abs(m_ShooterState.getLeftStateOutput() - m_shooterLeft.getVelocity().getValueAsDouble());
        double rightError = Math.abs(m_ShooterState.getRightStateOutput() - m_shooterRight.getVelocity().getValueAsDouble());
        if ((leftError + rightError)/2 < ShooterConstants.k_SHOOTER_VELOCITY_TOLERANCE) {
            return true;
        }
        return false;
    }
    
        /**
         * Command factory method. Parameter is a ShooterState enum value
         * @return runOnce that sets m_ShooterState to parameter
         */
    public Command setStateCommand(ShooterState state) {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return startEnd(() -> this.m_ShooterState = state, ()-> this.m_ShooterState = ShooterState.STOP);
      }
}