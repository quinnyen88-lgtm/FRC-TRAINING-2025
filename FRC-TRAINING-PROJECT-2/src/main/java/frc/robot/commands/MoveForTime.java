// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveForTime extends Command {

  private Timer m_timer = new Timer();
  private Drivetrain m_motor;
  private double m_speed;
  private double m_timeInSeconds;

  /** Creates a new MoveForTime. */
  public MoveForTime( Drivetrain motor, double speed, double timeInSeconds) {
    m_motor = motor;
    m_speed = speed;
    m_timeInSeconds = timeInSeconds;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_motor.setLeftSpeed(m_speed);
    m_motor.setRightSpeed(m_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_motor.setRightSpeed(0);
    m_motor.setLeftSpeed(0);
    m_timer.stop();
    m_timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_timeInSeconds);

  }
}
