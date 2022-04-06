// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogGyro;
//import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.serializer.Kicker;

public class RotateRobot extends CommandBase {
  private AnalogGyro gyro;
  private boolean inverted;
  DriveTrain driveTrain;
  Boolean finish;
  double angle;
  Rotation2d heading;
  private Rotation2d gangle;
  /** Creates a new rotateRobot. */
  public RotateRobot(DriveTrain d, double ang) {
    gyro = new AnalogGyro(1);
    driveTrain = d;
    //inverted = invert;
    addRequirements(driveTrain);
    angle = ang;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finish = false;
    gyro.reset();
    gyro.
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.arcadeDrive(0, 0.1);
    gangle = Rotation2d.fromDegrees(gyro.getAngle());
    System.out.println(gangle);
    if(gangle == Rotation2d.fromDegrees(180)){
      finish = true;
    }
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
