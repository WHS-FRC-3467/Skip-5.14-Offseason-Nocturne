// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Stage.State;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.Vision.Limelight;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Superstructure extends SubsystemBase {

    /** Superstucture singleton. */
    private static Superstructure instance = null;

    @Getter
    @Setter
    public RobotState m_RobotState;


    /** Arm subsystem reference. */
    private final Arm m_ArmSubsystem;

    /** Intake subsystem reference. */
    private final Intake m_IntakeSubsystem;

    /** Shooter subsystem reference. */
    private final Shooter m_ShooterSubsystem;

    /** Stage subsystem reference. */
    private final Stage m_StageSubsystem;

        /** Drivetrain subsystem reference. */
    private final CommandSwerveDrivetrain m_Drivetrain;
    
    /**
    * Returns the superstructure instance.
    *
    * @return the superstructure instance.
    */
    public static Superstructure getInstance() {
        if (instance == null) {
        instance = new Superstructure();
        }

        return instance;
    }

    /** Creates a new SuperstructureSubsystem. */
    public Superstructure() {

        m_ArmSubsystem = Arm.getInstance();
        m_IntakeSubsystem = Intake.getInstance();
        m_ShooterSubsystem = Shooter.getInstance();
        m_StageSubsystem = Stage.getInstance();
        m_Drivetrain = CommandSwerveDrivetrain.getInstance();
        m_RobotState = RobotState.getInstance();
    
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        displayInfo(true);

        // Update SuperState periodically to for eventual use of LEDs
    }


    /**
     * @param state - use one of the defined states (INTAKE, EJECT, SUBWOOFER, etc)
     * @return a startEnd command that sets m_Superstate to the parameter for the duration of the command
    */
   // public Command setTargetCommand(RobotState.Target target) {
        // return new RunCommand(m_RobotState.setTargetCommand(target)); // should not be null, superstructure should be ignored for now
    // }

    public void displayInfo(boolean debug) {
        if (debug) {
            SmartDashboard.putString("Target", m_RobotState.getTarget().toString());
            SmartDashboard.putNumber("Distance to Target", m_RobotState.getDistanceToTarget());
        // getAngleToTarget returns the angle in a Rotation2d object, hence the need to getDegrees()
            SmartDashboard.putNumber("Angle to Target", m_RobotState.getAngleToTarget().getDegrees());
        }
    }
}