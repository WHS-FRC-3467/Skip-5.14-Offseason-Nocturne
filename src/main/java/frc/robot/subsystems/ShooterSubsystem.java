package frc.robot.subsystems;
package frc.robot.Subsystems.Shooter;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanConstants;
//import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;
// import frc.robot.Util.Setpoints;
// import frc.robot.Util.TunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    @RequiredArgsConstructor
    @Getter
    public enum State {

        STAYSTILL(() -> 0.0, () -> 0.0), // To either hold a note or stop running
        GETNOTE(() -> 1.0, () -> 1.0), // Gets the note from the stage
        VOMIT(() -> 1.0, () -> 1.0); // Shoots or just runs the shooter

        private final DoubleSupplier outputSupplierUno;
        private final DoubleSupplier outputSupplierDos;

        private double getLeftStateOutput() {
            return outputSupplierUno.getAsDouble();
        }

        private double getRightStateOutput() {
            return outputSupplierDos.getAsDouble();
        }
    }

    @Getter
    @Setter
    private State state = State.STAYSTILL;

    // Initialize motor controllers
    TalonFX m_leftShooter = new TalonFX(18);

    public ShooterSubsystem() {

        m_leftShooter.setInverted(true);
    }

@Override
  public void periodic() {
//Periodic will check if there is input coming from the xbox controls (I hope)
//then the states will respond accordingly (in theory)
    if (state == State.OFF) {
        return boolean STAYSTILL
    } else {
      // m_intakeMotor.setControl(m_percent.withOutput(state.getStateOutput()));
    }
  }

}