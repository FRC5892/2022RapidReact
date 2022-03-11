package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class AutonDrive extends CommandBase {
	DriveTrain driveTrain;
	Boolean finish;

	public AutonDrive(DriveTrain d) {
		driveTrain = d;
		addRequirements(driveTrain);
	}

	@Override
	public void initialize() {
		finish = false;
	}

	public void execute(double distance) {
		if (Math.abs(driveTrain.getLeftPosition()) <= distance) {
			driveTrain.driveWithJoysticks(-Constants.AUTONOMOUS_SPEED, 0);
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
