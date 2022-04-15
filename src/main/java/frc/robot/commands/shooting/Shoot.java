// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.Constants;
import frc.robot.subsystems.serializer.Kicker;
import frc.robot.subsystems.serializer.Tower;
import frc.robot.subsystems.shooter.Flywheel;

public class Shoot extends CommandBase {
	private Flywheel flywheel;
	private Tower tower;
	private Kicker kicker;
	private Hood hood;
	private Double speed;
	private Double angle;

	/** Creates a new AimAndShoot. */
	public Shoot(Flywheel f, Tower tw, Kicker k, Hood h, Double s, Double ag) {
		flywheel = f;
		tower = tw;
		kicker = k;
		hood = h;
		speed = s;
		angle = ag;

		addRequirements(flywheel, tower, kicker, hood);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		flywheel.setSetpoint(speed);
		hood.setSetpoint(angle);
		hood.enable();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (flywheel.atSetpoint() && hood.atSetpoint()) {
			kicker.setMotors(Constants.KICKER_SHOOT_SPEED);
		}
		else {
			if (!kicker.hasBall()) {
				kicker.setMotors(Constants.KICKER_SPEED);
				tower.setMotors(Constants.TOWER_SPEED);
			}
			else if (!tower.hasBall()) {
				tower.setMotors(Constants.TOWER_SPEED);
			}
			if (kicker.hasBall()) {
				kicker.stopMotors();
			}
			if (kicker.hasBall() && tower.hasBall()) {
				kicker.stopMotors();
				tower.stopMotors();
			}
		}

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		flywheel.stopMotors();
		tower.stopMotors();
		kicker.stopMotors();
		hood.stopMotors();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
