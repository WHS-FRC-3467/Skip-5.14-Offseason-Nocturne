# Skip-5.14-Offseason-Nocturne
Nocturne 2024 Offseason Bot

- This code base was started with Jason Daming's improved version of the CTRE Swerve Example (see notes below).
- Added basic Subsystems: Drive, Arm, Shooter, Intake and Stage.
- Cleaned up and reorganized RobotContainer.jave to make it clearer for new programmers.
- Arm, Shooter and Intake use Falcons and therefore the Phoenix 6 library. Intake and uses Talon SRX, and therefore the Phoenix 5 library.
- Stage no longer uses Talon SRX, it is testing Thrify Nova.
- This code uses the CTRE Swerve Generator built into Tuner X. After generating a project using the Generator, copy the newly-generated TunerConstants.java into this code.
- Development, Testing, and Stable will be used to determine the progress for subsystems/commands/features

# Development

# Testing/Ready For Testing
- Arm PID

# Stable
- Features that work, but may be put back in development again if it needs updating
- Drivetrain (TELEOP mode)
- Intaking (Intake + Stage Subsystem) and Eject
- Basic arm functionality including Arm to Angle via Shuffleboard (Noah's-Branch version)
- Driver Controls
- Basic shooter functionality

# To be Started
- Manual Arm control
- Note Following
- Trap Subsystem
- Look and Shoot
- Shoot & Move
- Pathplanner Autos

# Driver Controls
- The two joysticks: Drive robot
- Hold Left Bumper: Turtle Mode (Drivetrain) for fine adjustments
- Press B: Start Shooter
- Press A: Stop Shooter
- Press X: Manual Intake
- Press Y: Manual Shoot Note (stage)
- Hold Left Trigger: Regular Intake
- Hold Right Trigger: Eject Note (manual outtake)

# Operator Controls
- Press DPad Left: Shooter + Arm to Podium position
- Press DPad Up: Shooter + Arm to AMP position
- Press DPad Right: Arm to Harmony position
- Press DPad Down: Arm to Subwoofer position
- Drivetrain testing (see RobotContainer)

# CTRE Swerve Example

Repo: https://github.com/jasondaming/ctre_swerve

Features:
- Improved Limelight support
- Improved PathPlanner autonomous setup
- Added SysID options for running swerve drivetrain characterization
- Different xbox joystick controller layout to try for driver comfort
- Easily scale down the max speed to help newer drivers learn
- "Turtle Mode" (left bumper) temporarily slows the drivetrain down for fine adjustments

This is an expanded version of the CTRE [SwerveWithPathPlanner](https://github.com/CrossTheRoadElec/Phoenix6-Examples/tree/main/java/SwerveWithPathPlanner) example using the CTRE Swerve Builder.  To use it copy the generated/TunerConstants.java file from the generated project and replace the generated/TunerConstants.java file in this project.  You will also need to set your team number.

To use the limelight ensure you have a 36h11 pipeline properly configured and then change the [enable constant](https://github.com/jasondaming/ctre_swerve/blob/master/src/main/java/frc/robot/Vision/Limelight.java#L22) to true.

Will add specific steps on characterization after testing.