// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
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
	private Tower tower;
	private Kicker kicker;
	private DriveTrain driveTrain;
	private PIDController driveTrainPIDController = new PIDController(Constants.DRIVETRAIN_AIM_PID_CONSTANTS[0],
			Constants.DRIVETRAIN_AIM_PID_CONSTANTS[1], Constants.DRIVETRAIN_AIM_PID_CONSTANTS[2]);

	private boolean shootWhenReady;
	private double[] xCoords;
	private double[] shooterYCoords;
	private double[] hoodYCoords;

	

	/** Creates a new AimAndShoot. */
	public AimAndShoot(Flywheel f, Turret t, Hood h, Tower tw, Kicker k, TurretVision tv,
			DriveTrain dt) {
		flywheel = f;
		turret = t;
		hood = h;
		tower = tw;
		kicker = k;
		turretVision = tv;
		driveTrain = dt;
		driveTrainPIDController.setTolerance(.25);
		addRequirements(flywheel, turret, hood, tower, kicker, turretVision, driveTrain);

		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		shootWhenReady = false;
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (turretVision.hasTargets()) {
			// turret.setSetpoint(turret.getMeasurement() - turretVision.xAngle());
			xCoords = new double[] {0,50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210};
			shooterYCoords = new double[] {2050, 2050, 2050, 2050, 2050, 2050, 2150, 2200, 2200, 2200, 2250, 2300, 2360, 2400, 2550, 2580, 2650, 2700};
			hoodYCoords = new double[] {28, 28, 30, 34, 37, 40, 42, 43, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44};

			if (shootWhenReady) {
				if (hood.atSetpoint() && flywheel.atSetpoint() /*&& turret.atSetpoint()*/) {
					// shoot
					kicker.setMotors(Constants.KICKER_SHOOT_SPEED);
					tower.setMotors(Constants.TOWER_SPEED);
				}
			} else {
				driveTrain.arcadeDrive(0, -driveTrainPIDController.calculate(turretVision.xAngle(), -3));

				if (driveTrainPIDController.atSetpoint()) {
					driveTrain.stopMotors();
					flywheel.setSetpoint(LinearInterpolation.calculate(xCoords, shooterYCoords,  turretVision.distanceFromTarget()));
					hood.setSetpoint((double) LinearInterpolation.calculate(xCoords, hoodYCoords,  turretVision.distanceFromTarget()));
					hood.enable();
					shootWhenReady = true;
				}
			}
		}

		// preload balls
		if (!shootWhenReady) {
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
			
			else {
				kicker.setMotors(Constants.KICKER_SPEED);
				tower.setMotors(Constants.TOWER_SPEED);
			}
			if (tower.hasBall() && kicker.hasBall()) {
				tower.stopMotors();
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
		flywheel.stopMotors();
		// turret.stop();
		hood.stopMotors();
		tower.stopMotors();
		kicker.stopMotors();
		driveTrain.stopMotors();
		System.out.println("Stopping");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return finished;
	}
}
