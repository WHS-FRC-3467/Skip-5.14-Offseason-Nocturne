// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
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
import java.util.function.DoubleSupplier;

public class IntakeSubsystem extends SubsystemBase {

    public enum State{

        ON (() -> 1.0),
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

    // m_intakeLead.setNeutralMode(1);

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {

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
            m_intakeLead.set(speed);
        }
    }

    public void intakeReverse(double speed) {
        if (speed <= -0.1) {
            m_intakeLead.set(speed);
        }
    }

    public void stopIntake() {
        m_intakeLead.set(0.0);
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
