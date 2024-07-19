package frc.robot.Subsystems.Shooter;

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
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.Constants.StageConstants;
import frc.robot.Util.Setpoints;
import frc.robot.Util.TunableNumber;
//import frc.robot.sim.PhysicsSim;

public class ShooterSubsystem extends SubsystemBase {

    // Initialize devices
    TalonFX m_shooterLeft = new TalonFX(18);
    TalonFX m_shooterRight = new TalonFX(19);

    // Declare ShooterState Variables - keep track of what the shooter is doing
    ShooterState m_ShooterState = ShooterState.STOP;
    ShooterState m_GoalState = ShooterState.STOP;

    public ShooterSubsystem() {
        
        var talonFXConfigurator = m_shooterLeft.getConfigurator();
        var limitConfigs = new CurrentLimitsConfigs();
        var outputConfigs = new MotorOutputConfigs();

        // enable stator current limit
        limitConfigs.StatorCurrentLimit = 120;
        limitConfigs.StatorCurrentLimitEnable = true;

        // Set brake as neutralmodevalue
        outputConfigs.NeutralMode = NeutralModeValue.Brake;

        talonFXConfigurator.apply(limitConfigs);
        talonFXConfigurator.apply(outputConfigs);
        
        m_shooterRight.getConfigurator().apply(limitConfigs);
        m_shooterRight.getConfigurator().apply(outputConfigs);

    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Shooter at speed", isShooterAtSpeed(getSpeedSetPoint()));
        SmartDashboard.putString("Shooter state", getShooterState().toString());
    }

    public ShooterState getShooterState() {
        return m_ShooterState;
    }

    public double getSpeedSetPoint() {
        // TODO: make the (goal) shooter state enum associate to double values that represent speed
        return 0.0;
    }

    public boolean isShooterAtSpeed(double expectedVelocity) {
        // TalonFX getVelocity() gets the Velocity of the device in mechanism rotations per second
        if (MathUtil.isNear(expectedVelocity, m_shooterLeft.getVelocity().getValueAsDouble(), ShooterConstants.k_SHOOTER_VELOCITY_TOLERANCE)) {
            m_ShooterState = ShooterState.READY;
            return true;
        }
        return false;
        // .get gets the set speed from 0 to 1. 319/3 is the free speed of a Falcon 500 (rotations per second)
    }
    
    public void runShooter(double speed, boolean isSpooling) {
        // Actually tell motors to run at the speed
        if (speed >= 0.1) {
            m_shooterLeft.set(speed);
            if (isSpooling){
                m_ShooterState = ShooterState.SPOOLING;
            } else {
            m_ShooterState = ShooterState.REVVING;
            }
        }
    }

    public void runShooter() {
        m_shooterLeft.set(Constants.ShooterConstants.k_DEFAULT_FWD_VELOCITY);
        m_ShooterState = ShooterState.REVVING;
    }
    public void reverseShooter() {
        m_shooterLeft.set(Constants.ShooterConstants.k_SHOOTER_REV_VELOCITY);
        m_ShooterState = ShooterState.REVERSE;
        m_GoalState = ShooterState.REVERSE;
    }

    public void stopShooter() {
        m_shooterLeft.stopMotor();
        m_GoalState = ShooterState.STOP;
    }

        /**
     * Example command factory method.
     *
     * @return runShooter(speed)
     */

    public Command runShooterCommand(double speed, boolean isSpooling) {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> runShooter(speed, isSpooling));
    }

        /**
         * Speed is set to 0.8
         * @return runShooter()
         */
    public Command runShooterCommand() {
        return runOnce(() -> runShooter());
    }

    public Command reverseShooterCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(() -> reverseShooter());

    }

    public Command stopShooterCommand() {
        return runOnce(() -> stopShooter());
    }

}