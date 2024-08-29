// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.StageConstants;
import frc.robot.Util.ThriftyNova;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class Stage extends SubsystemBase {

  @RequiredArgsConstructor
  @Getter
  public enum State {
    INTAKE(1.0),
    EJECT(-1.0),
    SHOOT(1.0),
    OFF(0.0);

    private final double output;
    //Bryson: you can removed this once you switch to doubles
    
  }

  private State state = State.OFF;

  ThriftyNova thrifty_nova = new ThriftyNova(CanConstants.ID_StageMotor);
  // Make sure to import CanConstants in Constants.java
  

  boolean m_noteInStage = false;
 

  public void runStage(double speed) {
    thrifty_nova.setPercentOutput(speed);

  }

  public void stopStage() {
    thrifty_nova.setPercentOutput(0.0);

  }

  DigitalInput m_stageBeamBreak = new DigitalInput(DIOConstants.kStageBeamBreak);
  // Make sure to Import DIOConstants from Constants.java

  // Creates a new Stage
  public Stage() {
    thrifty_nova.setInverted(false);
    thrifty_nova.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    StageInfo(true);
    
    if (state == State.OFF) {
     thrifty_nova.setPercentOutput(0);
    } else {
    thrifty_nova.setPercentOutput(state.getOutput());
    }
  }

  private void StageInfo(boolean debug) {
    if (debug) {
      SmartDashboard.putBoolean("Note In Stage?", isNoteInStage());
      SmartDashboard.putString("SimpleSubsystem State ", state.toString());
      SmartDashboard.putNumber("StageVelocity ", thrifty_nova.getExtVelocity());
      SmartDashboard.putNumber("Stage Current Draw", thrifty_nova.getCurrentDraw()); 
    }
  }  

  public boolean isNoteInStage() {
    return !m_stageBeamBreak.get();
  }


  public Command setStateCommand(State state) {
    return startEnd(() -> this.state = state, () -> this.state = State.OFF);
  }

}
