// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.serializer.Accumulator;
import frc.robot.subsystems.serializer.Intake;
import frc.robot.subsystems.serializer.Kicker;
import frc.robot.subsystems.serializer.Tower;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.subsystems.shooter.TurretVision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonDriveAndIntake extends ParallelDeadlineGroup {
  /** Creates a new AutonShootAndAim. */
  public AutonDriveAndIntake(Flywheel f, Turret t, Hood h, Accumulator a, Tower tw, Kicker k, TurretVision tv,
  DriveTrain dt, Intake i) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new AutonDrive(dt, Constants.AUTON_DISTANCE, false, k, i, tw));
    addCommands(new AutonIntakeRollers(i), new AutonRunAccumulator(), new AutonRunTower(k, tw), new AutonRunKicker(k, tw));
    
    // addCommands(new FooCommand(), new BarCommand());
  }
}
