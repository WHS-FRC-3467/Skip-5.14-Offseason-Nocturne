// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. *
  public Shooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration; // Go to class descrpitions for more info
//import com.ctre.phoenix6.controls.Follower; I am not sure if I need this, it may just be for the Arm
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.Constants.CanConstants;
//import frc.robot.Constants.RobotConstants;
//import frc.robot.Constants.ShooterConstants;
// import frc.robot.Util.TunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

//import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    @RequiredArgsConstructor
    @Getter
    public enum State {

        OFF(() -> 0.0, () -> 0.0), // To either hold a note or stop running
        SHOOT(() -> 30.0, () -> 30.0), // Gets the note from the stage
        AMP(() -> 20.0, () -> 30.0), // Shoots or just runs the shooter
        PODIUM(() -> 40.0, () -> 40.0); //A random shooter state

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
    public double maxVelocity = 20; //Use Nocturn's values
    @Getter
    @Setter
    public double maxAcceleration = 5; //Or this
    @Getter
    @Setter

    private ProfiledPIDController pidController = new ProfiledPIDController(2, 3, 4,
      new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));


  
    @Getter
    @Setter
    private State state = State.OFF;


    //Some Constants
        public static final int ID_ShooterLeft = 15;
        public static final int ID_ShooterRight = 17;

    // Initialize motor controllers
    TalonFX m_leftShooter = new TalonFX(18);
    TalonFX m_rightShooter = new TalonFX(19);

    //Voltage Velocity - instantiate (to create an object from a class (I think)) --> done
    //Set max velocity --> Done
    //Declare a PID controler --> Attempted
    //Declare what P, I, and D are. --> Also Attempted
    // Neutral mode - what it can do while stopped --> Did Coast (not sure if Brake is required)

    // Current limits


    public Shooter() {

        m_leftShooter.setInverted(true);
        m_rightShooter.setInverted(false);
        TalonFXConfiguration m_configuration = new TalonFXConfiguration();
          m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast; 
          m_configuration.CurrentLimits.StatorCurrentLimit = 0;
          m_configuration.CurrentLimits.StatorCurrentLimitEnable = false;
          m_configuration.CurrentLimits.SupplyCurrentLimit = 60;
          m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
          m_configuration.CurrentLimits.SupplyCurrentThreshold = 150;
          m_configuration.CurrentLimits.SupplyTimeThreshold = 2;
        //NeutralOut m_neutral = new NeutralOut();
        //talonFXConfigurator.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        //TalonFXConfiguration m_configuration = new TalonFXConfiguration();

    }

    @Override
    public void periodic() {
    //Periodic will check if there is input coming from the xbox controls (I hope)
    //then the states will respond accordingly (in theory)

      if (state == State.OFF) {
          m_leftShooter.set(0.0);
          m_rightShooter.set(0);
      } else {
          //Set m_temp to VelocityVoltage based on state

        //Set m_temp to be a percentage of VoltageVelocity
         m_leftShooter.setControl(m_temp.withVelocity(state.getLeftStateOutput()));
         m_rightShooter.setControl(m_temp.withVelocity(state.getRightStateOutput()));
        //if the state is SHOOT, then set m_temp as a percent of xbox controler input for shooting
      }
  }


    public Command setStateCommand(State state) {
      return startEnd(() -> this.state = state, () -> this.state = State.OFF);
    }

}
//Note to self, it is VelocityVoltage, not VoltageVelocity :)