package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimAndShoot;
import frc.robot.subsystems.Accumulator;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Shooter.Flywheel;
import frc.robot.subsystems.Shooter.Hood;
import frc.robot.subsystems.Shooter.Turret;
import frc.robot.subsystems.Shooter.TurretVision;

public class ComplexAuto extends SequentialCommandGroup {
	private Flywheel f;
	private Turret t;
	private Hood h;
	private Accumulator a;
	private Tower tw;
	private Kicker k;
	private TurretVision tv;

	private DriveTrain driveTrain;
	// AimAndShoot aimAndShoot = new AimAndShoot(f, t, h, a, tw, k, tv)

	public ComplexAuto() {
		addCommands(new AutonDrive(driveTrain), new AimAndShoot(f, t, h, a, tw, k, tv));
	}
}
