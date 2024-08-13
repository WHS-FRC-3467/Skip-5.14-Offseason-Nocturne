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
    STOWED,
    INTAKE,
    AMP,
    SUBWOOFER,
    SPEAKER,
    FEED,
    SHOOT,
    CLIMB,
    UNJAM,


  }

  @Getter
  @Setter
  private State state = State.STOWED;

  private Arm arm;
  private Drivetrain drivetrain;
  private Intake intake;
  private Shooter shooter;
  private Stage stage;


  /** Creates a new StageSubsystem. */
  public Superstructure(Arm arm, Drivetrain drivetrain, Intake intake, Shooter shooter, Stage stage) {
    this.arm = arm;
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.shooter = shooter;
    this.stage = stage;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (state) {
      case STOWED:
        arm.setState(Arm.State.STOW);
        drivetrain.setState(Drivetrain.State.TELEOP);
        intake.setState(Intake.State.OFF);
        shooter.setState(Shooter.State.OFF);
        stage.setState(Stage.State.OFF);
        break;

      case INTAKE:
        arm.setState(Arm.State.INTAKE);
        shooter.setState(Shooter.State.OFF);
        if (arm.atGoal() && !stage.hasNote()) {
            intake.setState(Intake.State.INTAKE);
            stage.setState(Stage.State.INTAKE);
        } else {
          intake.setState(Intake.State.OFF);
          stage.setState(Stage.State.OFF);
        }
        break;

      case AMP:
      arm.setState(Arm.State.AMP);
      shooter.setState(Shooter.State.AMP);

    }

  }


  public Command setStateCommand(State state) {
    return startEnd(() -> setState(state),() -> setState(State.STOWED));
  }
}
