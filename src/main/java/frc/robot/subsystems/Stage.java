// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.StageConstants;
import frc.robot.Util.ThriftyNova;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Stage extends SubsystemBase {

  @RequiredArgsConstructor
  @Getter
  public enum State {
    OFF(0.0),
    INTAKE(0.25),
    SHOOT(1.0),
    UNJAM(-1.0);

    private final double outputSupplier;
  }

  @Getter
  @Setter
  private State state = State.OFF;

  ThriftyNova thrifty_nova = new ThriftyNova(StageConstants.ID_StageMotor);
  DigitalInput m_stageBeamBreak = new DigitalInput(StageConstants.ID_StageBeamBreak);

  /** Creates a new StageSubsystem. */
  public Stage() {
    thrifty_nova.setInverted(false);
    thrifty_nova.setBrakeMode(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (state == State.OFF) {
      thrifty_nova.setPercentOutput(0);
    } else {
      thrifty_nova.setPercentOutput(state.getOutputSupplier());
    }

    displayInfo(true);
  }

  //TODO: evalute if needs to move to different class
  public boolean hasNote() {
    return !m_stageBeamBreak.get();
  }

  public Command setStateCommand(State state) {
    return startEnd(() -> setState(state), () -> setState(State.OFF));
  }

  private void displayInfo(boolean debug) {
    if (debug) {
      SmartDashboard.putString("Stage State ", state.toString());
      SmartDashboard.putNumber("Stage Setpoint ", state.getOutputSupplier());
      SmartDashboard.putNumber("Stage Output ", thrifty_nova.getVelocity());
      SmartDashboard.putNumber("Stage Current Draw", thrifty_nova.getCurrentDraw());
      SmartDashboard.putBoolean("Stage has Note?", hasNote());
    }

  }
}
