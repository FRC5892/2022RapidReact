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
import frc.robot.commands.autonomous.RotateRobot;

public class AutonSetup1 extends SequentialCommandGroup {

	public AutonSetup1(Flywheel f, Turret t, Hood h, Accumulator a, Tower tw, Kicker k, TurretVision tv,
			DriveTrain dt, RotateRobot rR) {
		addCommands(new RotateRobot(dt, 180));
	}
}
//new TimedShoot(f, a, tw, k, Constants.AUTONOMOUS_SHOOT_TIMER)
//new AutonDrive(dt, 1, false, k)
