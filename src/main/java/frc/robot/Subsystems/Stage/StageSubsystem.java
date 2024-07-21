package frc.robot.Subsystems.Stage;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import frc.robot.Constants;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.StageConstants;
// import frc.robot.sim.PhysicsSim;
import frc.robot.Util.ThriftyNova;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
public class StageSubsystem extends SubsystemBase {

    @RequiredArgsConstructor
    @Getter
    public enum State{

        INTAKE (() -> StageConstants.k_STAGE_VELOCITY),
        SHOOTING (() -> StageConstants.kFeedToShooterSpeed),
        AMP (() -> StageConstants.kFeedToAmpSpeed),
        REV (() -> StageConstants.k_STAGE_REV_VELOCITY),
        OFF(() -> 0.0);

        private final DoubleSupplier outputSupplier;

        private double getStateOutput() {
            return outputSupplier.getAsDouble();
        }
    }
    
    @Getter
    @Setter
    private State state = State.OFF;

    // To get a boolean supplier for the beam break to use in commands
    // False if no note in stage. Should be set to true if note is in stage.
    public BooleanSupplier beambreakSupplier = ()-> false;

    // Initialize devices
    ThriftyNova thrifty_nova = new ThriftyNova(CanConstants.k_STAGE_CAN_ID);
    DigitalInput m_stageBeamBreak = new DigitalInput(DIOConstants.k_INTAKE_BEAM_BREAK);

    /** Creates a new StageSubsystem. */
    public StageSubsystem() {

        thrifty_nova.setBrakeMode(false);
        thrifty_nova.setInverted(false);

    }

    @Override
    public void periodic() {
        // Change state of beambreak boolean supplier as necessary
        // If beamstate is open and there is a note in the beambreak then change the boolean supplier
        if ((!beambreakSupplier.getAsBoolean()) && (!m_stageBeamBreak.get())) {
            beambreakSupplier = ()-> true; // If note in stage, close beamstate
        } else if ((beambreakSupplier.getAsBoolean()) && (m_stageBeamBreak.get())){
            // Vice versa - if boolean supplier says there is a note in stage but there isn't, then switch boolean supplier
            beambreakSupplier = ()-> false; // If note not in stage anymore, open beamstate
        }

        SmartDashboard.putNumber("Stage Setpoint Velocity", state.getStateOutput());
        SmartDashboard.putNumber("Stage 'Velocity'", thrifty_nova.getVelocity());
        SmartDashboard.putBoolean("Note in Stage", !m_stageBeamBreak.get());
        SmartDashboard.putNumber("Stage Current Draw", thrifty_nova.getCurrentDraw());

        thrifty_nova.setPercentOutput(state.getStateOutput());
    }

    /**
     * 
     * @return whether the difference between set speed and actual speed of stage is within tolerance
     */

    public boolean isAtSpeed() {
        return MathUtil.isNear(state.getStateOutput(), thrifty_nova.getVelocity(), StageConstants.k_STAGE_VELOCITY_TOLERANCE);
    }
    
    public boolean isNoteInStage() {
        return !m_stageBeamBreak.get();
    }

    /*
     * Command Factories
     */

    /**
     * Run Stage until note is detected, which ends the command
     *
     * @return a command setting the stage state to INTAKE until a note is in stage, then it will set state to OFF.
     */
    public Command runStageUntilNoteCommand() {
        return new RunCommand(()-> setStateCommand(State.INTAKE)).until(beambreakSupplier).andThen(()-> setStateCommand(State.OFF));
    }
    
    /**
     * Example command factory method. Periodic tells the stage to run according to the state
     *
     * @return a command setting the stage state to the argument
     */
    public Command setStateCommand(State stageState) {
        return runOnce(() -> this.state = stageState);
    }

}
