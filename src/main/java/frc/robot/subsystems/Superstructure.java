// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Superstructure extends SubsystemBase {

  @RequiredArgsConstructor
  @Getter
  public enum State {
    OFF(0.0),
    INTAKE(0.45),
    EJECT(1.0),
    FEED(.3),
    OUTTAKE(-1.0);

    private final double outputSupplier;
  }

  @Getter
  @Setter
  private State state = State.OFF;


  /** Creates a new StageSubsystem. */
  public Superstructure() {


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


  }



  public Command setStateCommand(State state) {
    return startEnd(() -> setState(state),() -> setState(State.OFF));
  }
}
