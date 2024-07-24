package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class StageSubsystem extends SubsystemBase {

    @RequiredArgsConstructor
    @Getter
    public enum State {

        STAYSTILL(() -> 0.0), // To either hold a note or stop running
        GETNOTE(() -> 1.0), // Gets the note from the intake
        VOMIT(() -> 1.0), // Does the stage shoot notes? If not, then this just feeds notes to the
                          // shooter.

        private final DoubleSupplier outputSupplier;

        private double getLeftStateOutput() {
            return outputSupplier.getAsDouble();
        }

    }

    @Getter @Setter

}
