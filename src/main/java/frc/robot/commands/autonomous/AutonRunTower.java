// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.serializer.Kicker;
import frc.robot.subsystems.serializer.Tower;

public class AutonRunTower extends CommandBase {
  private Kicker kicker;
	private Timer timer;
	private Tower tower;
  /** Creates a new AutonRunTower. */
  public AutonRunTower(Kicker k, Tower t) {
    kicker = k;
		tower = t;
		// Use addRequirements() here to declare subsystem dependencies.
		//addRequirements(tower);
		timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer.reset();
		timer.start();
		tower.setMotors(Constants.TOWER_SPEED);
    if ((kicker.hasBall() && tower.hasBall())) {
			tower.stopMotors();
			timer.stop();
		}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tower.stopMotors();
		timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
