// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Robot;
public class PhotonVision extends DriveTrain {
  DriveTrain driveTrain;
  
  PhotonCamera camera = new PhotonCamera("photonvision");
  /** Creates a new PhotonVision. */
  public PhotonVision(DriveTrain dt) {
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
    final double LINEAR_P = 0.1;
    final double LINEAR_D = 0.0;
    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
    final double ANGULAR_P = 0.1;
    final double ANGULAR_D = 0.0;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(3);
    var result = camera.getLatestResult();
    driveTrain = dt;
    double rotationSpeed = 0;

            if (result.hasTargets()) {
                // Calculate angular turn power
                // -1.0 required to ensure positive PID controller effort _increases_ yaw
                rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
            } else {
                // If we have no targets, stay still.
                rotationSpeed = 0;
            }
            
            drive.arcadeDrive(0, rotationSpeed);
          }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
