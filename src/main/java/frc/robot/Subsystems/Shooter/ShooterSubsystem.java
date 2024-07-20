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
    TalonFX m_shooterLeft = new TalonFX(18);
    TalonFX m_shooterRight = new TalonFX(19);

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

    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Shooter at speed", isShooterAtSpeed());
        SmartDashboard.putString("Shooter state", getM_ShooterState().toString());

        // Will be phased out to VelocityVoltage reqest using setControl(Velocity Voltage request) from CoreTalonFX
        m_shooterLeft.set(m_ShooterState.getLeftStateOutput()/100);
        m_shooterRight.set(m_ShooterState.getLeftStateOutput()/100);

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