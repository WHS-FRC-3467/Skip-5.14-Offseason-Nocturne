// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Arm extends SubsystemBase {

  @RequiredArgsConstructor
  @Getter
  public enum State { //0 angle is straight horizontal
    STOW(() -> -20),
    INTAKE(() -> -18.0),
    SUBWOOFER(() -> -18.0),
    AMP(() -> 74.0),
    FEED(() -> -9.0),
    CLIMB(() -> 69.0),
    HARMONY(() -> 103.0),
    LOOKUP(() -> 0.0); //TODO: Add call to robot state to get lookup angle

    private final DoubleSupplier outputSupplier;

    private double getStateOutput() {
      return Units.degreesToRadians(outputSupplier.getAsDouble());
    }
  }

  @Getter
  @Setter
  private State state = State.STOW;

  private final double upperLimit = Units.degreesToRadians(130);
  private final double lowerLimit = Units.degreesToRadians(-20);
  private final double tolerance = Units.degreesToRadians(.5);
  private final double maxVelocity = Math.PI;
  private final double maxAcceleration = Math.PI / 2;

  private ProfiledPIDController pidController = new ProfiledPIDController(18, 0, 0.2,
      new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
  private double goalAngle;
  private ArmFeedforward ff = new ArmFeedforward(0.5, 0.4, 2.5, 0.01);
  private double output = 0;

  TalonFX m_armMotor = new TalonFX(ArmConstants.ID_ArmLeader);
  TalonFX m_armFollowerMotor = new TalonFX(ArmConstants.ID_ArmFollower);

  private VoltageOut m_VoltageOutput = new VoltageOut(0.0);
  private final NeutralOut m_neutralOut = new NeutralOut();

  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(ArmConstants.ID_ArmAbsEncoder);

  private Debouncer m_debounce = new Debouncer(.1);
  


  /** Creates a new ArmSubsystem. */
  public Arm() {

    m_armMotor.getConfigurator().apply(ArmConstants.motorConfig());
    m_armFollowerMotor.getConfigurator().apply(ArmConstants.motorConfig());
    m_armFollowerMotor.setControl(new Follower(ArmConstants.ID_ArmLeader, true));

    m_encoder.setDutyCycleRange(1.0/1025.0, 1024.0/1025.0);
    m_encoder.setDistancePerRotation(2*Math.PI);
    m_encoder.setPositionOffset(0.50666666);
    

    pidController.setTolerance(tolerance);
    pidController.setIZone(100); // TODO: Figure out actual value

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    goalAngle = MathUtil.clamp(state.getStateOutput(), lowerLimit, upperLimit);
    pidController.setGoal(goalAngle);
    
    if (state == State.STOW && atGoal()) {
      m_armMotor.setControl(m_neutralOut);
    } else {
      output = pidController.calculate(m_encoder.getDistance()) + ff.calculate(goalAngle, 0);
      m_armMotor.setControl(m_VoltageOutput.withOutput(output));
    }

    displayInfo(true);

  }

  public boolean atGoal() {
    return m_debounce.calculate(MathUtil.isNear(goalAngle, m_encoder.getDistance(), tolerance));
  }

  public Command setStateCommand(State state) {
    return startEnd(() -> setState(state),() -> setState(State.STOW));
  }

  private void displayInfo(boolean debug) {
    if (debug) {
      SmartDashboard.putString("Arm State ", state.toString());
      SmartDashboard.putNumber("Arm Setpoint ", Units.radiansToDegrees(goalAngle));
      SmartDashboard.putNumber("Arm Angle ", Units.radiansToDegrees(m_encoder.getDistance()));
      SmartDashboard.putBoolean("Arm at Goal?", atGoal());
      SmartDashboard.putNumber("Arm Current Draw", m_armMotor.getSupplyCurrent().getValueAsDouble());
      SmartDashboard.putData("Arm PID",pidController);
    }

  }
}
