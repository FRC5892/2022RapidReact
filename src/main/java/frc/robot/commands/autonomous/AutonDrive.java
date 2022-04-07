package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.serializer.Kicker;
import frc.robot.subsystems.serializer.Tower;
import frc.robot.subsystems.serializer.Intake;

public class AutonDrive extends CommandBase {
	DriveTrain driveTrain;
	Boolean finish;
	private double distance;
	private boolean inverted;
	private double initialPosition;
	private Kicker kicker;
	private Intake intake;

	public AutonDrive(DriveTrain d, double dist, boolean invert, Kicker k, Intake i, Tower t) {
		driveTrain = d;
		distance = dist;
		kicker = k;
		intake = i;
		addRequirements(driveTrain, kicker, intake);
		inverted = invert;
		new Timer();
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
				driveTrain.arcadeDrive(-Constants.AUTONOMOUS_SPEED, 0);
			}
		else {
			driveTrain.stop();
			finish = true;
		}

		
	}

	@Override
	public void end(boolean interrupted) {
		driveTrain.stop();
		kicker.stopMotors();
		System.out.println("Stopping AutonDrive");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return finish;
	}
}
