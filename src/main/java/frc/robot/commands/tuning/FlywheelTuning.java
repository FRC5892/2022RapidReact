// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tuning;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Flywheel;

public class FlywheelTuning extends CommandBase {
	private Flywheel flywheel;

	/** Creates a new OutputFlywheelEncoder. */
	public FlywheelTuning(Flywheel f) {
		// Use addRequirements() here to declare subsystem dependencies.
		flywheel = f;
		addRequirements(flywheel);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		flywheel.enable();

		SmartDashboard.putNumber("Flywheel P", flywheel.getController().getP());
		SmartDashboard.putNumber("Flywheel I", flywheel.getController().getI());
		SmartDashboard.putNumber("Flywheel D", flywheel.getController().getD());
		SmartDashboard.putNumber("Flywheel Setpoint", flywheel.getSetpoint());
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		SmartDashboard.putNumber("Flywheel Setpoint", flywheel.getSetpoint());
		SmartDashboard.getNumber("Flywheel P", 0);
		SmartDashboard.getNumber("Flywheel I", 0);
		SmartDashboard.getNumber("Flywheel D", 0);

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		flywheel.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
