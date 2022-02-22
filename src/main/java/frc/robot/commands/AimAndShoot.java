// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.Flywheel;
import frc.robot.subsystems.Shooter.Hood;
import frc.robot.subsystems.Shooter.Turret;
import frc.robot.subsystems.Shooter.TurretVision;

public class AimAndShoot extends CommandBase {
	private Flywheel flywheel;
	private Turret turret;
	private Hood hood;
	private TurretVision turretVision;
	private boolean finished;

	/** Creates a new AimAndShoot. */
	public AimAndShoot(Flywheel f, Turret t, Hood h, TurretVision tv) {
		flywheel = f;
		turret = t;
		hood = h;
		turretVision = tv;

		addRequirements(flywheel, turret, hood, tv);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
    flywheel.setSetpoint(Constants.FLYWHEEL_SHOOTING_SPEED);
    hood.setSetpoint(turretVision.distanceFromTarget());
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		flywheel.stop();
		turret.stop();
		hood.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return finished;
	}
}
