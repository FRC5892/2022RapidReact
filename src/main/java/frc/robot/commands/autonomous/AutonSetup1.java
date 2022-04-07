package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.shooting.AimAndShoot;
import frc.robot.commands.shooting.TimedShoot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.serializer.Accumulator;
import frc.robot.subsystems.serializer.Intake;
import frc.robot.subsystems.serializer.Kicker;
import frc.robot.subsystems.serializer.Tower;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.subsystems.shooter.TurretVision;

public class AutonSetup1 extends SequentialCommandGroup {

	public AutonSetup1(Flywheel f, Turret t, Hood h, Accumulator a, Tower tw, Kicker k, TurretVision tv,
			DriveTrain dt, Intake i) {
		addCommands(new TimedShoot(f, a, tw, k, Constants.AUTONOMOUS_SHOOT_TIMER), new RotateRobot(dt, 177), new AutonDriveAndIntake(f, t, h, a, tw, k, tv, dt, i), new RotateRobot(dt, 177), new AimAndShoot(f, t, h, a, tw, k, tv, dt));
	}
}
//new TimedShoot(f, a, tw, k, Constants.AUTONOMOUS_SHOOT_TIMER)
//new AutonDrive(dt, 1, false, k)
