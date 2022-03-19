package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.shooting.TimedShoot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.serializer.Accumulator;
import frc.robot.subsystems.serializer.Kicker;
import frc.robot.subsystems.serializer.Tower;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.subsystems.shooter.TurretVision;

public class SimpleAuton extends SequentialCommandGroup {

	public SimpleAuton(Flywheel f, Turret t, Hood h, Accumulator a, Tower tw, Kicker k, TurretVision tv, DriveTrain dt) {
		addCommands(new TimedShoot(f, a, tw, k, Constants.AUTONOMOUS_SHOOT_TIMER), new AutonDrive(dt, 3, false, k));
	}
}
