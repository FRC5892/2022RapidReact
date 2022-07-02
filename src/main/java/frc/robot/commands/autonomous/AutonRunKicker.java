// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.serializer.Kicker;
import frc.robot.subsystems.serializer.Tower;

public class AutonRunKicker extends CommandBase {
  private Kicker kicker;
	private Timer timer;
	/** Creates a new AutonRunKicker. */
  public AutonRunKicker(Kicker k) {
    kicker = k;
		timer = new Timer();
		// Use addRequirements() here to declare subsystem dependencies.
		//addRequirements(kicker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer.reset();
		timer.start();
		kicker.setMotors(Constants.KICKER_SPEED);
    if (kicker.hasBall()) {
			kicker.stopMotors();
			timer.stop();
		}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    kicker.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
