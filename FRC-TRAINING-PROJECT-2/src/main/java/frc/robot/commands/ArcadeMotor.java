// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Drivetrain;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArcadeMotor extends Command {

  private  Drivetrain m_motor;
  private Joystick m_joystick;

  //Speed, turn, left, right variables
  /*
  Seperation because drivetrain only has two methods for setting left and right speeds.
  so need to figure out how much speed on each side to get it to turn and go.
  */
  private double m_speed;
  private double m_turn;
  private double m_left;
  private double m_right;



  /** Creates a new ArcadeMotor. */
  public ArcadeMotor(Drivetrain motor, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_motor = motor;
    m_joystick = joystick;
    addRequirements(m_motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_speed = m_joystick.getY();
    m_turn = m_joystick.getX();
    //for it to turn one of sides has to have a negative
    m_left = m_speed + m_turn;
    m_right = m_speed - m_turn;
    m_motor.setLeftSpeed(m_left);
    m_motor.setRightSpeed(m_right);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
