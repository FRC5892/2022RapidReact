// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.serializer.Intake;

public class AutonIntakeToggle extends CommandBase {
  private Intake intake;
  Boolean finish;
  /** Creates a new AutonIntakeToggle. */
  public AutonIntakeToggle(Intake i) {
    intake = i;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.togglePistons();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    finish = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
