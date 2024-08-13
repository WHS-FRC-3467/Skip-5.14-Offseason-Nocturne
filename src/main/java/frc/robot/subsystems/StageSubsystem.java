package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import edu.wpi.first.hal.CANAPIJNI;
import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;

public class StageSubsystem extends SubsystemBase {

    @RequiredArgsConstructor
    @Getter
    public enum State {

        STAYSTILL(() -> 0.0), // To either hold a note or stop running
        GETNOTE(() -> 1.0), // Gets the note from the intake
        VOMIT(() -> 1.0), // Does the stage shoot notes? If not, then this just feeds notes to the shooter.
    

        private final DoubleSupplier outputSupplier;

        private double getLeftStateOutput() {
            return outputSupplier.getAsDouble();
        }

    }

    @Getter @Setter
    
    
    private State state = State.STAYSTILL;

    public StageSubsystem() {

        thrifty_nova.STAYSTILL(false);
        thrifty_nova.setInverted(true);

@Override
  public void periodic() {
//Periodic will check if there is a note coming from the intake and/or if the shooter needs a note
//then the states will respond accordingly (hopefully)
    if (state == State.OFF) {
        return boolean STAYSTILL
    } else {
      // m_intakeMotor.setControl(m_percent.withOutput(state.getStateOutput()));
    }
  }

}
