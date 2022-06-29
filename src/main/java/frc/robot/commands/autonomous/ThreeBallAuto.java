// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.shooting.AimAndShoot;
import frc.robot.subsystems.DriveTrain;
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
public class ThreeBallAuto extends SequentialCommandGroup {
  /** Creates a new ThreeBallAuto. */
  public ThreeBallAuto(Flywheel f, Turret t, Hood h, Tower tw, Kicker k, TurretVision tv,
  DriveTrain dt, Intake i) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AimAndShoot(f, t, h, tw, k, tv, dt), new RotateRobot2(dt, 176), new InstantCommand(i::openPistons, i), 
    new InstantCommand(i::runMotorFoward, i), new AutonDrive(dt, Units.inchesToMeters(41.5), false, k, i, tw), new RotateRobot2(dt, 118), //122
    new AutonDrive(dt, Units.inchesToMeters(113.1), false, k, i, tw), new RotateRobot2(dt, 50), new AimAndShoot(f, t, h, tw, k, tv, dt),
    new InstantCommand(i::closePistons, i), new InstantCommand(i::stopMotors, i));
  }
}
