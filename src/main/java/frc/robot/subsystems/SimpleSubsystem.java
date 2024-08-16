// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Util.ThriftyNova;
import frc.robot.Constants.CanConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class SimpleSubsystem extends SubsystemBase {

  @RequiredArgsConstructor
  @Getter
  public enum State {
    ON(() -> 1.0),
    OFF(() -> 0.0);

    private final DoubleSupplier outputSupplier;

    private double getStateOutput() {
      return outputSupplier.getAsDouble();
    }
  }

  @Getter
  @Setter
  private State state = State.OFF;

  TalonFX m_motor = new TalonFX(Constants.ExampleCTREMotorConfig.ID_Motor);
  //ThriftyNova thrifty_nova = new ThriftyNova(CanConstants.ID_Motor);
  // Make sure to either use TalonFX or ThriftyNova 
  private final DutyCycleOut m_percent = new DutyCycleOut(0);
  private final NeutralOut m_brake = new NeutralOut();

  /** Creates a new SimpleSubsystem. */
  public SimpleSubsystem() {
    m_motor.getConfigurator().apply(Constants.ExampleCTREMotorConfig.motorConfig());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (state == State.OFF) {
      m_motor.setControl(m_brake);
    } else {
      m_motor.setControl(m_percent.withOutput(state.getStateOutput()));
    }
  }

  public Command setStateCommand(State state) {
    return startEnd(() -> this.state = state, () -> this.state = State.OFF);
  }
}
