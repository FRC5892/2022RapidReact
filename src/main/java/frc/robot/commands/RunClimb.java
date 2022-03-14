// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OperatorInput;
import frc.robot.subsystems.Climb;

public class RunClimb extends CommandBase {
  private Climb climb;

  /** Creates a new RunClimb. */
  public RunClimb(Climb c) {
    // Use addRequirements() here to declare subsystem dependencies.
    climb = c;
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.driveArms(OperatorInput.codriverJoystick.getLeftY(), OperatorInput.codriverJoystick.getRightY());
    // if (OperatorInput.codriverJoystick.getLeftY() != 0 || OperatorInput.codriverJoystick.getRightY() != 0) {
    //   climb.unlockTelescope();
    // }
    // else {
    //   climb.lockTelescope();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // climb.lockTelescope();
    climb.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
