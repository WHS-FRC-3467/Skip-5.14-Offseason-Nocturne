// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

//import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.IntakeConstants;

import java.util.function.DoubleSupplier;

public class IntakeSubsystem extends SubsystemBase {

    public enum State{

        FWD (() -> IntakeConstants.k_INTAKE_FWD_SPEED),
        REV (() -> IntakeConstants.k_INTAKE_REV_SPEED),
        OFF(() -> 0.0);

        private State(DoubleSupplier outputSupplier) {
            this.outputSupplier = outputSupplier;
        }
        private final DoubleSupplier outputSupplier;

        private double getStateOutput() {
            return outputSupplier.getAsDouble();
        }
    }

    private State state = State.OFF;

    // Initalize Motors and Beam Break
    TalonFX m_intakeLead = new TalonFX(CanConstants.k_INTAKE_LEFT_CAN_ID);
    TalonFX m_intakeFollow = new TalonFX(CanConstants.k_INTAKE_RIGHT_CAN_ID);

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        
        var talonFXConfigurator = m_intakeLead.getConfigurator();
        var limitConfigs = new CurrentLimitsConfigs();
        var outputConfigs = new MotorOutputConfigs();

        // enable stator current limit
        limitConfigs.StatorCurrentLimit = 120;
        limitConfigs.StatorCurrentLimitEnable = true;

        // Set brake as neutralmodevalue
        outputConfigs.NeutralMode = NeutralModeValue.Brake;

        talonFXConfigurator.apply(limitConfigs);
        talonFXConfigurator.apply(outputConfigs);
        
        m_intakeFollow.getConfigurator().apply(limitConfigs);
        m_intakeFollow.getConfigurator().apply(outputConfigs);
        m_intakeFollow.setControl(new Follower(m_intakeLead.getDeviceID(), true));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Intake State", state.getStateOutput());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    /**
     * A method querying a boolean state of the subsystem (for example, a
     * digital sensor).
     *
     * @return true if intake speed if it is within the allowed tolerance, returns false if not

     */
    public boolean isIntakeAtSpeed(double targetSpeed, double tolerance) {
        // Note that CoreTalonFX getVelocity() returns in rotations per second
        double intakeAverage = (m_intakeLead.getVelocity().getValueAsDouble() + m_intakeFollow.getVelocity().getValueAsDouble()) / 2.0;
        return MathUtil.isNear(targetSpeed, intakeAverage, tolerance);
    }

    public void intakeForward(double speed) {
        // Actually tell motors to run at the speed
        if (Math.abs(speed) >= 0.1) {
            if (speed < 0.0) {
                state = State.REV;
            } else {
                state = State.FWD;
            }
            m_intakeLead.set(state.getStateOutput());
        }
    }



    public void stopIntake() {
        state = State.OFF;
        m_intakeLead.set(state.getStateOutput());
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */

    public Command intakeOnCommand(double speed) {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> intakeForward(speed));
    }

    public Command intakeOffCommand() {
        return runOnce(() -> stopIntake());
    }
}
