// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.serializing;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.OperatorInput;
import frc.robot.subsystems.serializer.Intake;
import frc.robot.subsystems.serializer.Kicker;
import frc.robot.subsystems.serializer.Tower;

public class RunIntakeRollers extends CommandBase {
	private Intake intake;
	private Tower tower;
	private Kicker kicker;
	private boolean running;
	private Timer timer;
	private WaitCommand wait;
	private boolean stopping;

	/** Creates a new RunIntakeRollers. */
	public RunIntakeRollers(Intake i, Tower t, Kicker k) {
		// Use addRequirements() here to declare subsystem dependencies.
		intake = i;
		tower = t;
		kicker = k;
		running = false;
		stopping = false;
		timer = new Timer();
		wait = new WaitCommand(0.5);
		addRequirements(intake);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// default
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		System.out.print("INTAKESPEED" + intake.getMotors());
		System.out.println("INTAKE TIMER = " + timer.get());
		System.out.println("RUNNING = "+running);
		System.out.println("STOPPING = "+stopping);
		

		if (intake.returnPistons() == Value.kReverse){
			if (OperatorInput.driverJoystick.getRightTriggerAxis() > 0.05){
				intake.openPistons();
				intake.setMotors(-Constants.INTAKE_SPEED_MULTIPLIER);
				System.out.println("Backwards");
				running = true;
			}
			else if (OperatorInput.driverJoystick.getLeftTriggerAxis() > 0.05){
				intake.openPistons();
				intake.setMotors(Constants.INTAKE_SPEED_MULTIPLIER);
				System.out.println("Fowards");
				running = true;
			}
		}
		if (intake.returnPistons() == Value.kForward){
			if(OperatorInput.driverJoystick.getLeftTriggerAxis() < 0.05 && OperatorInput.driverJoystick.getRightTriggerAxis() < 0.05){
				intake.closePistons();
				if (running){
					timer.start();
					running = false;
					stopping = true;
				}
				
			}
		
			}
		if (stopping){
			if (timer.get() > 0.1){
				intake.setMotors(0);
				timer.stop();
				timer.reset();
				System.out.println("STOPPING INTAKE");
				stopping = false;
				}
		}
		
		}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		intake.stopMotors();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
