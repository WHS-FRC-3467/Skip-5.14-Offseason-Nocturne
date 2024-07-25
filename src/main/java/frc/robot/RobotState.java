// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class RobotState {
    private final Rotation2d ampAngle = new Rotation2d(-Math.PI/2);

    public Rotation2d getAmpAngle() {
    return ampAngle;
}
    
}


