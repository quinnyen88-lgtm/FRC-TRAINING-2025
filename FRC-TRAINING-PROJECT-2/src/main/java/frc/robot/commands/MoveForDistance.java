// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveForDistance extends Command {

  private Drivetrain m_motor;
  private double m_speed;
  private double m_distanceInFeet;
  private double m_ticksToTravel;
  private double m_ticksLeftToRotate;
  private double m_startingTicks;
  private MoveForDistanceSendable m_moveForDistanceSendable = new MoveForDistanceSendable();
  
  

  /** Creates a new MoveForDistance. */
  public MoveForDistance(Drivetrain motor, double distanceInFeet, double speed) {
    m_motor = motor;
    m_distanceInFeet = distanceInFeet;
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ticksToTravel = (m_distanceInFeet / (Math.PI * DrivetrainConstants.kDiameterInInches)) * DrivetrainConstants.kTicksPerRevolution;
    m_startingTicks = m_motor.getCurrentTickRightPosition();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_motor.setLeftSpeed(DrivetrainConstants.kSpeed);
    m_motor.setRightSpeed(DrivetrainConstants.kSpeed);
    m_ticksLeftToRotate = (m_startingTicks + m_ticksToTravel) - m_motor.getCurrentTickRightPosition();
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
    return m_ticksLeftToRotate <= 0;
  }

  public Sendable getSendable() {
    return m_moveForDistanceSendable;
  }

  private class MoveForDistanceSendable implements Sendable {
  
    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
        builder.setSmartDashboardType("Move For Distance");
        builder.addDoubleProperty("Speed", () -> m_speed, null);
        builder.addDoubleProperty("Ticks left to rotate", () -> m_ticksLeftToRotate, null);
        builder.addDoubleProperty("Goal feet to travel", () -> m_distanceInFeet, null);
        
    }
  }
}
