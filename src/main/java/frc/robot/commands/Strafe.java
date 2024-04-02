// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Strafe extends InstantCommand {

    double strafeValue;
    private Swerve s_Swerve;

    public Strafe(double strafeValue) {
        strafeValue = this.strafeValue;
        // Use addRequirements() here to declare subsystem dependencies.
        this.s_Swerve = Swerve.getInstance();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        s_Swerve.drive(0, strafeValue, 0, false, false);
    }
}
