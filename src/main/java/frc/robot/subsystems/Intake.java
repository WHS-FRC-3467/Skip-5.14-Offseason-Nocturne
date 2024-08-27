// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {

    /** Intake subsystem singleton. For superstructure. */
    private static Intake instance = null;

    @RequiredArgsConstructor
    @Getter
    public enum State{

        FWD (() -> IntakeConstants.k_INTAKE_FWD_SPEED),
        REV (() -> IntakeConstants.k_INTAKE_REV_SPEED),
        OFF(() -> 0.0);

        private final DoubleSupplier outputSupplier;

        private double getStateOutput() {
            return outputSupplier.getAsDouble();
        }
    }

    @Getter
    @Setter
    private State state = State.OFF;

    // Initalize Motors and Beam Break
    TalonFX m_intakeLead = new TalonFX(IntakeConstants.ID_IntakeLeader);
    TalonFX m_intakeFollow = new TalonFX(IntakeConstants.ID_IntakeFollower);

    // For superstructure
    /**
    * Returns the intake subsystem instance.
    *
    * @return the intake subsystem instance.
    */
    public static Intake getInstance() {
        if (instance == null) {
        instance = new Intake();
        }

        return instance;
    }

    /** Creates a new IntakeSubsystem. */
    public Intake() {
        
        m_intakeLead.getConfigurator().apply(IntakeConstants.motorConfig());
        m_intakeFollow.setControl(new Follower(m_intakeLead.getDeviceID(), false));
        m_intakeFollow.getConfigurator().apply(IntakeConstants.motorConfig());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        displayInfo(true);

        m_intakeLead.set(state.getStateOutput());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    /**
     * Example command factory method. Periodic tells the intake to run according to the state
     *
     * @return a command setting the intake state to the argument
     */
    public Command setStateCommand(State intakestate) {
        return startEnd(() -> this.state = intakestate, () -> this.state = State.OFF);
    }

    public void displayInfo(boolean debug) {
        if (debug) {
            SmartDashboard.putString("Intake State", getState().toString());
            SmartDashboard.putNumber("Intake Setpoint", state.getStateOutput());
            SmartDashboard.putNumber("Intake speed", m_intakeLead.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber("Intake Current Draw", m_intakeLead.getSupplyCurrent().getValueAsDouble());
        }
    }
}
