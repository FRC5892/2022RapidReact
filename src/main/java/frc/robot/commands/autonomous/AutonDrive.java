package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class AutonDrive extends CommandBase {
	DriveTrain driveTrain;
	Boolean finish;
	private double distance;
	private boolean inverted;
	private double initialPosition;

	public AutonDrive(DriveTrain d, double dist, boolean invert) {
		driveTrain = d;
		distance = dist;
		addRequirements(driveTrain);
		inverted = invert;
	}

	@Override
	public void initialize() {
		finish = false;
		initialPosition = driveTrain.getLeftPosition();
	}

	@Override
	public void execute() {
		if (Math.abs(driveTrain.getLeftPosition() - initialPosition) <= distance) {
			if (inverted) {
				driveTrain.driveWithJoysticks(-Constants.AUTONOMOUS_SPEED, 0);
			}
			else {
				driveTrain.driveWithJoysticks(Constants.AUTONOMOUS_SPEED, 0);
			}
		}
		else {
			driveTrain.stop();
			finish = true;
		}
	}

	@Override
	public void end(boolean interrupted) {
		driveTrain.stop();
		System.out.println("Stopping AutonDrive");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return finish;
	}
}
