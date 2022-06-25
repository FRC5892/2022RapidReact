// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooting.AimAndShoot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.serializer.Intake;
import frc.robot.subsystems.serializer.Kicker;
import frc.robot.subsystems.serializer.Tower;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.subsystems.shooter.TurretVision;
import frc.robot.commands.autonomous.AutonDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DumbAuton extends SequentialCommandGroup {
  /** Creates a new DumbAuton. */
  public DumbAuton(Flywheel f, Turret t, Hood h, Tower tw, Kicker k, TurretVision tv,
  DriveTrain dt, Intake i) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AimAndShoot(f, t, h,  tw, k, tv, dt), new AutonDrive(dt, -1.5, true, k, i, tw));
  }
}
