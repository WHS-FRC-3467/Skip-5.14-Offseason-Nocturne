// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Shooter extends SubsystemBase {

  @RequiredArgsConstructor
  @Getter
  public enum State {
    OFF(0.0, 0.0),
    SUBWOOFER(27.0, 27.0),
    AMP(40.0, 40.0),
    FEED(28.0, 28.0),
    PASSTHROUGH(10.0, 10.0),
    SHOOT(70.0, 40.0);

    private final double leftVelocity;
    private final double rightVelocity;
  }

  @Getter
  @Setter
  private State state = State.OFF;

  private double tolerance = 2;

  TalonFX m_leftShooterMotor = new TalonFX(0);
  TalonFX m_rightShooterMotor = new TalonFX(0);
  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0.0);
  private final NeutralOut m_neutralOut = new NeutralOut();

  /** Creates a new SimpleSubsystem. */
  public Shooter() {
    var motorConfig = new TalonFXConfiguration();

    /* set motors to Coast */
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.Slot0.kP = .03;
    motorConfig.Slot0.kI = 0;
    motorConfig.Slot0.kD = 0;
    motorConfig.Slot0.kV = .125;

    /* Apply configs */
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_leftShooterMotor.getConfigurator().apply(motorConfig);
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_rightShooterMotor.getConfigurator().apply(motorConfig);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (state == State.OFF) {
      m_leftShooterMotor.setControl(m_neutralOut);
    } else {
      m_leftShooterMotor.setControl(m_voltageVelocity.withVelocity(state.getLeftVelocity()));
      m_rightShooterMotor.setControl(m_voltageVelocity.withVelocity(state.getLeftVelocity()));
    }
  }

  public boolean atGoal() {
    return state == State.OFF || (
          MathUtil.isNear(state.getLeftVelocity(), m_leftShooterMotor.getVelocity().getValueAsDouble(), tolerance) &&
          MathUtil.isNear(state.getRightVelocity(), m_rightShooterMotor.getVelocity().getValueAsDouble(), tolerance));
  }

  public Command setStateCommand(State state) {
    return startEnd(() -> setState(state),() -> setState(State.OFF));
  }
}
