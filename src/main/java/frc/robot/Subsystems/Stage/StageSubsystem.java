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
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterState;
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

        FWD (() -> StageConstants.k_STAGE_VELOCITY),
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

    @RequiredArgsConstructor
    @Getter
    public enum BeamState{

        OPEN (() -> false), // Note not in stage
        CLOSED(() -> true); // Note in stage

        private final BooleanSupplier bbSupplier;

        private boolean getStateOutput() {
            return bbSupplier.getAsBoolean();
        }
    }
    
    @Getter
    @Setter
    private BeamState beambreak = BeamState.OPEN;

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
        SmartDashboard.putNumber("Stage State", state.getStateOutput());
        SmartDashboard.putBoolean("Note in Stage", !m_stageBeamBreak.get());
        SmartDashboard.putNumber("Stage Current Draw", thrifty_nova.getCurrentDraw());

        thrifty_nova.setPercentOutput(state.getStateOutput());

        // Change state of beambreak boolean supplier as necessary
        if ((beambreak == BeamState.OPEN) && (!m_stageBeamBreak.get())) {
            beambreak = BeamState.CLOSED; // If note in stage, close beamstate
        } else if ((beambreak == BeamState.OPEN) && (m_stageBeamBreak.get())){
            beambreak = BeamState.OPEN; // If note not in stage anymore, open beamstate
        }
    }

    /**
     * 
     * @param speed speed that the Stage motor is set at
     */

    public boolean isAtSpeed(int speed) {
        return MathUtil.isNear(speed, thrifty_nova.getVelocity(), StageConstants.k_STAGE_VELOCITY_TOLERANCE);
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
     * @return a command setting the intake state to the argument
     */
    public Command runStageUntilNoteCommand() {
        return new RunCommand(()-> setStateCommand(State.FWD)).until(beambreak.getBbSupplier());
    }
    
    /**
     * Example command factory method. Periodic tells the stage to run according to the state
     *
     * @return a command setting the intake state to the argument
     */
    public Command setStateCommand(State stageState) {
        return runOnce(() -> this.state = stageState);
    }

}
