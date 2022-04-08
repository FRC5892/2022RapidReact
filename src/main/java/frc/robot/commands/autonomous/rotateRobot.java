// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;



//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class RotateRobot extends CommandBase {
  private double gyroangle;
  DriveTrain driveTrain;
  Boolean finish;
  double angle;
  Rotation2d heading;
  ADXRS450_Gyro gyro;
  /** Creates a new rotateRobot. */
  public RotateRobot(DriveTrain d, double ang) {
    gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
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
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gyroangle = gyro.getAngle();
    if(gyroangle < -angle){
      finish = true;
    }
    driveTrain.arcadeDrive(0, 0.2);
    System.out.println(gyroangle);
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
