package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.timedShoot;
import frc.robot.subsystems.Accumulator;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Shooter.Flywheel;
import frc.robot.subsystems.Shooter.Hood;
import frc.robot.subsystems.Shooter.Turret;
import frc.robot.subsystems.Shooter.TurretVision;

public class SimpleAuton extends SequentialCommandGroup {

	public SimpleAuton(Flywheel f, Turret t, Hood h, Accumulator a, Tower tw, Kicker k, TurretVision tv, DriveTrain dt,
			double timerValue) {
		addCommands(new timedShoot(f, a, tw, k, timerValue), new AutonDrive(dt, 1, true));
	}
}
