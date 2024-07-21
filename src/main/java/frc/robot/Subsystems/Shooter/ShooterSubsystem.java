package frc.robot.Subsystems.Shooter;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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
import frc.robot.Constants.StageConstants;
import frc.robot.Util.Setpoints;
import frc.robot.Util.TunableNumber;
//import frc.robot.sim.PhysicsSim;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class ShooterSubsystem extends SubsystemBase {

    // Initialize devices
    TalonFX m_shooterLeft = new TalonFX(CanConstants.ID_ShooterLeft);
    TalonFX m_shooterRight = new TalonFX(CanConstants.ID_ShooterRight);

    /* Declare ShooterState Variables - This enum is used to keep track of what the shooter is doing
    *   Numbers should be in rotations per second. Left supplier corresponds to left shooter motor, right supplier to right shooter motor
    *   Example: ShooterState currentShooterState = ShooterState.SPOOLING;
    */ 
    @RequiredArgsConstructor
    @Getter
    public enum ShooterState {
        STOP    (()-> 0.0, ()-> 0.0),
        SPOOLING(()-> 10.0, ()-> 10.0), // Poop & Scoot
        REVVING (()-> 40.0, ()-> 35.0), // Dynamic
        READY   (()-> Constants.ShooterConstants.k_DEFAULT_FWD_VELOCITY, ()-> Constants.ShooterConstants.k_DEFAULT_FWD_VELOCITY), // Shooter constants?
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

    @Getter
    @Setter
    ShooterState m_GoalState = ShooterState.STOP; // May be depreciated we'll see

    public ShooterSubsystem() {
        
        var talonFXConfigurator = new TalonFXConfiguration();
        // enable stator current limit
        talonFXConfigurator.CurrentLimits.StatorCurrentLimit = 120;
        talonFXConfigurator.CurrentLimits.StatorCurrentLimitEnable = true;

        // Set brake as neutralmodevalue
        talonFXConfigurator.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_shooterLeft.getConfigurator().apply(talonFXConfigurator);   
        m_shooterRight.getConfigurator().apply(talonFXConfigurator);

        // in init function, set slot 0 gains - for Velocity PID
        // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/basic-pid-control.html
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = ShooterConstants.kSVolts; // Add 0.1 V output to overcome static friction
        slot0Configs.kV = ShooterConstants.kVVoltSecondsPerRotation; // A velocity target of 1 rps results in 0.12 V output
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
        
        // create a velocity closed-loop request, voltage output, slot 0 configs
        final VelocityVoltage m_requestRight = new VelocityVoltage(0).withSlot(0);
        final VelocityVoltage m_requestLeft = new VelocityVoltage(0).withSlot(0);
        // set velocity according to state, add 0.5 V to overcome gravity
        m_shooterRight.setControl(m_requestRight.withVelocity(m_ShooterState.getRightStateOutput()).withFeedForward(0.5));
        m_shooterLeft.setControl(m_requestLeft.withVelocity(m_ShooterState.getLeftStateOutput()).withFeedForward(0.5));

    }

    public boolean isShooterAtSpeed() {
        // TalonFX getVelocity() gets the Velocity of the device in mechanism rotations per second
        double leftError = Math.abs(m_ShooterState.getLeftStateOutput() - m_shooterLeft.getVelocity().getValueAsDouble());
        double rightError = Math.abs(m_ShooterState.getRightStateOutput() - m_shooterRight.getVelocity().getValueAsDouble());
        if ((leftError + rightError)/2 < ShooterConstants.k_SHOOTER_VELOCITY_TOLERANCE) {
            m_GoalState = ShooterState.READY;
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
        return runOnce(() -> this.m_ShooterState = state);
      }
}