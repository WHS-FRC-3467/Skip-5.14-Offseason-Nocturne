// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;


import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

// import frc.robot.Util.TunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  @RequiredArgsConstructor
  @Getter
  public enum State {

        OFF(() -> 0.0, () -> 0.0), // To either hold a note or stop running
        SHOOT(() -> 30.0, () -> 30.0), // Shoots or just runs the shooter
        AMP(() -> 20.0, () -> 30.0), // "Shoots" into the amp
        FEED(() -> 40.0, () -> 40.0); //A random shooter state (all of the above values are random as well)

    private final DoubleSupplier outputSupplierUno;
    private final DoubleSupplier outputSupplierDos;

    private double getLeftStateOutput() {
      return outputSupplierUno.getAsDouble();
    }

    private double getRightStateOutput() {
      return outputSupplierDos.getAsDouble();
    }
  }

    NeutralOut m_neutral = new NeutralOut();
    VelocityVoltage m_temp = new VelocityVoltage(0.0, 0.0, false, 0.0, 0, false, false, false);
  
    @Getter
    @Setter
    private State state = State.OFF;

    // Initialize motor controllers
    TalonFX m_leftShooter = new TalonFX(ShooterConstants.ID_ShooterLeft); 
    TalonFX m_rightShooter = new TalonFX(ShooterConstants.ID_ShooterRight);

    //vocab: instantiate - to create an object from a class (I believe)



  public Shooter() {

        m_leftShooter.getConfigurator().apply(ShooterConstants.shooterMotorConfig(m_leftShooter.getDeviceID()));
        m_rightShooter.getConfigurator().apply(ShooterConstants.shooterMotorConfig(m_rightShooter.getDeviceID()));

      
  }

    @Override
    public void periodic() {
        //Periodic will check if the state changed (due to input coming from the xbox controls) then the robot will respond accordingly to the state
      
      displayInfo(true);


      if (state == State.OFF) {
          m_leftShooter.setControl(m_neutral);
          m_rightShooter.setControl(m_neutral);
      }     //Bryson: the implementation you have commented out is correct

      else {

        //Set m_temp to be a percentage of VelocityVoltage --> not sure if this is necessary
         m_leftShooter.setControl(m_temp.withVelocity(state.getLeftStateOutput()));
         m_rightShooter.setControl(m_temp.withVelocity(state.getRightStateOutput()));
        //if the state is SHOOT, then hopefully set m_temp as a percent of xbox controler input for shooting
      }

    }

    public boolean atState() {
      
      return MathUtil.isNear(state.getLeftStateOutput(), m_leftShooter.getVelocity().getValueAsDouble(), ShooterConstants.ShooterVelocityTolerance) 
      && MathUtil.isNear(state.getRightStateOutput(), m_rightShooter.getVelocity().getValueAsDouble(), ShooterConstants.ShooterVelocityTolerance);
    }
    
    
    public void displayInfo(boolean debug) {
      if (debug) {
        SmartDashboard.putString("Shooter State", state.toString());
        SmartDashboard.putBoolean("AtState", atState());
      }
    }

  public Command setStateCommand(State state) {
    return startEnd(() -> this.state = state, () -> this.state = State.OFF);
  }


  }