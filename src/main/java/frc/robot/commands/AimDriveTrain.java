// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.shooter.TurretVision;

public class AimDriveTrain extends CommandBase {
  private DriveTrain driveTrain;
  private PIDController driveTrainPIDController = new PIDController(Constants.DRIVETRAIN_AIM_PID_CONSTANTS[0],
			Constants.DRIVETRAIN_AIM_PID_CONSTANTS[1], Constants.DRIVETRAIN_AIM_PID_CONSTANTS[2]);
  private TurretVision turretVision;

  /** Creates a new AimDriveTrain. */
  public AimDriveTrain(DriveTrain dt, TurretVision tv) {
    driveTrain = dt;
    turretVision = tv;
    addRequirements(driveTrain);
    SmartDashboard.putNumber("Point Angle", 0);
    SmartDashboard.putNumber("DriveTrain P", Constants.DRIVETRAIN_AIM_PID_CONSTANTS[0]);
		SmartDashboard.putNumber("DriveTrain I", Constants.DRIVETRAIN_AIM_PID_CONSTANTS[1]);
		SmartDashboard.putNumber("DriveTrain D", Constants.DRIVETRAIN_AIM_PID_CONSTANTS[2]);
    driveTrainPIDController.setTolerance(0.25);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    driveTrain.arcadeDrive(0, -driveTrainPIDController.calculate(turretVision.xAngle(), SmartDashboard.getNumber("Point Angle", -3)));
    // driveTrainPIDController.setP(SmartDashboard.getNumber("Shooter P", 0));
		// driveTrainPIDController.setI(SmartDashboard.getNumber("Shooter I", 0));
		// driveTrainPIDController.setD(SmartDashboard.getNumber("Shooter D", 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveTrainPIDController.atSetpoint();
  }
}
