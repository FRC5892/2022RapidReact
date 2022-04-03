// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.PolynomialFunction;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.serializer.Accumulator;
import frc.robot.subsystems.serializer.Kicker;
import frc.robot.subsystems.serializer.Tower;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.subsystems.shooter.TurretVision;

public class AimAndShoot extends CommandBase {
	private Flywheel flywheel;
	private Turret turret;
	private Hood hood;
	private TurretVision turretVision;
	private boolean finished;
	private boolean reverse;
	private double[] hoodRangingCoefficients = new double[] { 1, 2, 3 };
	private Accumulator accumulator;
	private Tower tower;
	private Kicker kicker;
	private DriveTrain driveTrain;
	private PIDController driveTrainPIDController = new PIDController(Constants.DRIVETRAIN_AIM_PID_CONSTANTS[0],
			Constants.DRIVETRAIN_AIM_PID_CONSTANTS[1], Constants.DRIVETRAIN_AIM_PID_CONSTANTS[2]);

	/** Creates a new AimAndShoot. */
	public AimAndShoot(Flywheel f, Turret t, Hood h, Accumulator a, Tower tw, Kicker k, TurretVision tv,
			DriveTrain dt) {
		flywheel = f;
		turret = t;
		hood = h;
		accumulator = a;
		tower = tw;
		kicker = k;
		turretVision = tv;
		driveTrain = dt;

		addRequirements(flywheel, turret, hood, accumulator, tower, kicker, turretVision, driveTrain);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		flywheel.setSetpoint(Constants.FLYWHEEL_SHOOTING_SPEED);
		flywheel.enable();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (turretVision.hasTargets()) {
			// turret.setSetpoint(turret.getMeasurement() - turretVision.xAngle());
			hood.setSetpoint(
					PolynomialFunction.polynomailFunction(turretVision.distanceFromTarget(), hoodRangingCoefficients));
			driveTrain.arcadeDrive(0, driveTrainPIDController.calculate(turretVision.xAngle(), 0));
			if (hood.atSetpoint() /* && turret.atSetpoint() */ && flywheel.atSetpoint()) {
				// shoot
				kicker.setMotors(Constants.KICKER_SPEED);
				accumulator.setMotors(Constants.ACCUMULATOR_SPEED);
			}
			else {
				// preload balls
				if (!kicker.hasBall()) {
					kicker.setMotors(Constants.KICKER_SPEED);
					tower.setMotors(Constants.TOWER_SPEED);
					accumulator.setMotors(Constants.ACCUMULATOR_SPEED);
				}
				else if (!tower.hasBall()) {
					accumulator.setMotors(Constants.ACCUMULATOR_SPEED);
					tower.setMotors(Constants.TOWER_SPEED);
				}
				if (kicker.hasBall()) {
					kicker.stopMotors();
				}
				if (kicker.hasBall() && tower.hasBall()) {
					kicker.stopMotors();
					accumulator.stopMotors();
					tower.stopMotors();
				}
			}
		}
		// else {
		// if (turret.atLeftLimit()) {
		// reverse = true;
		// }
		// else if (turret.atRightLimit()) {
		// reverse = false;
		// }
		// if (reverse) {
		// turret.setMotor(-(Constants.TURRET_SCAN_SPEED));
		// }
		// else {
		// turret.setMotor(Constants.TURRET_SCAN_SPEED);
		// }
		// }

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		flywheel.stop();
		// turret.stop();
		hood.stop();
		accumulator.stopMotors();
		tower.stopMotors();
		kicker.stopMotors();
		driveTrain.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return finished;
	}
}
