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
import frc.robot.Util.ThriftyNova;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class Stage extends SubsystemBase {

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

  ThriftyNova thrifty_nova = new ThriftyNova(CanConstants.ID_StageMotor);
    // Make sure to import CanConstants in Constants.java
    boolean m_noteInStage = false;
    BooleanSupplier m_noteInStageSupplier;
    boolean m_stageRunning = false;
    DigitalInput m_stageBeamBreak = new DigitalInput(DIOConstants.kStageBeamBreak);
    // Make sure to Import DIOConstants from Constants.java

  /** Creates a new Stage. */
  public Stage() {
    thrifty_nova.setInverted(false);
    thrifty_nova.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_noteInStage = m_stageBeamBreak.get() ? false : true;

        SmartDashboard.putBoolean("Note In Stage?", m_noteInStage);

        if (RobotConstants.kIsStageTuningMode) {
            //SmartDashboard.putNumber("Stage Current Draw", m_stageMotor.getSupplyCurrent());
            SmartDashboard.putBoolean("Is Stage Running", isStageRunning());
        }
  }
}
