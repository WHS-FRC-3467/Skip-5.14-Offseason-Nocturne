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

        public enum State{

        FWD (() -> StageConstants.k_STAGE_VELOCITY),
        REV (() -> StageConstants.k_STAGE_REV_VELOCITY),
        OFF(() -> 0.0);

        private State(DoubleSupplier outputSupplier) {
            this.outputSupplier = outputSupplier;
        }
        private final DoubleSupplier outputSupplier;

        private double getStateOutput() {
            return outputSupplier.getAsDouble();
        }
    }

    private State state = State.OFF;

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

    public void runStage() {
        state = State.FWD;
        thrifty_nova.setPercentOutput(state.getStateOutput());
    }
    public void reverseStage() {
        state = State.REV;
        thrifty_nova.setPercentOutput(state.getStateOutput());
    }

    public void stopStage() {
        state = State.OFF;
        thrifty_nova.setPercentOutput(state.getStateOutput());
    }

    /*
     * Command Factories
     */

         /**
         * Speed is set to 0.8
         * @return runStage()
         */
    public Command runStageCommand() {
        return new InstantCommand(() -> runStage());
    }

    public Command reverseStageCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return new InstantCommand(() -> reverseStage());

    }

    public Command stageOffCommand() {
        return runOnce(() -> stopStage());
    }

}
