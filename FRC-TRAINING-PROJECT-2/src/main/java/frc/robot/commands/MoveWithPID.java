// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveWithPID extends Command {

  //Pid controller
  private PIDController m_pidController = new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD);

  private Drivetrain m_motor;
  private double m_goalDistanceInFeet;
  private double m_speed;

  private double m_startingPositionTicks;
  private double m_error;
  private double m_ticksToTravel;
  private MoveForPIDSendable m_moveWithPIDSendable = new MoveForPIDSendable();
  /** Creates a new MoveWithPID. */
  public MoveWithPID(Drivetrain motor, double goalDistanceInfeet) {

    m_motor = motor;
    m_goalDistanceInFeet = goalDistanceInfeet;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_motor);
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //converting the goal distance in feet to Ticks
    m_ticksToTravel = (m_goalDistanceInFeet / (Math.PI * DrivetrainConstants.kDiameterInInches)) * DrivetrainConstants.kTicksPerRevolution;

    //getting starting position of ticks
    m_startingPositionTicks = m_motor.getCurrentTickRightPosition();

    m_pidController.setTolerance(DrivetrainConstants.kSetPoint);
    m_pidController.setIZone(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //getting how much distance is left to travel
    m_error = (m_startingPositionTicks + m_ticksToTravel) - m_motor.getCurrentTickRightPosition(); 
    m_speed = m_pidController.calculate(m_motor.getCurrentTickRightPosition(), m_goalDistanceInFeet);
    m_motor.setLeftSpeed(m_speed);
    m_motor.setRightSpeed(m_speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_motor.setLeftSpeed(0);
    m_motor.setRightSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pidController.atSetpoint();
  }

  public Sendable getSendable() {
    return m_moveWithPIDSendable;
  }
 
  private class MoveForPIDSendable implements Sendable {

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("Move With PID");
      builder.addDoubleProperty("Speed", ()-> m_speed, null);
      builder.addDoubleProperty("Error", ()-> m_error, null);
      builder.addDoubleProperty("Goal Distance in feet", ()-> m_goalDistanceInFeet, null);
      builder.addDoubleProperty("ticks to travel", ()-> m_ticksToTravel, null);
      builder.addDoubleProperty("Starting tick position", ()-> m_startingPositionTicks, null);
    }
  }
}
