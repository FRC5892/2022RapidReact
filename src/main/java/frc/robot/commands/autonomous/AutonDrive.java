package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.serializer.Kicker;

public class AutonDrive extends CommandBase {
	DriveTrain driveTrain;
	Boolean finish;
	private double distance;
	private boolean inverted;
	private double initialPosition;
	private Kicker kicker;

	public AutonDrive(DriveTrain d, double dist, boolean invert, Kicker k) {
		driveTrain = d;
		distance = dist;
		kicker = k;
		addRequirements(driveTrain, kicker);
		inverted = invert;
	}

	@Override
	public void initialize() {
		finish = false;
		initialPosition = driveTrain.getLeftPosition();
		kicker.setMotors(Constants.KICKER_SPEED);
	}

	@Override
	public void execute() {
		if (Math.abs(driveTrain.getLeftPosition() - initialPosition) <= distance) {
			if (inverted) {
				driveTrain.arcadeDrive(-Constants.AUTONOMOUS_SPEED, 0);
			}
			else {
				driveTrain.arcadeDrive(Constants.AUTONOMOUS_SPEED, 0);
			}
		}
		else {
			driveTrain.stopMotors();
			finish = true;
		}
	}

	@Override
	public void end(boolean interrupted) {
		driveTrain.stopMotors();
		kicker.stopMotors();
		System.out.println("Stopping AutonDrive");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return finish;
	}
}
