// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ArcadeMotor;
import frc.robot.commands.MoveForDistance;
import frc.robot.commands.MoveForTime;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.IOConstants;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  private Drivetrain m_motor = new Drivetrain();
  private Joystick m_joystick = new Joystick(IOConstants.kJoystickPort);
  private ArcadeMotor m_arcadeMotor = new ArcadeMotor(m_motor, m_joystick);
  private MoveForTime m_moveForTime = new MoveForTime(m_motor, DrivetrainConstants.kSpeed, DrivetrainConstants.kTimeInSeconds);
  private MoveForDistance m_moveForDistance = new MoveForDistance(m_motor, 10, 0.5);



  public RobotContainer() {
    m_motor.setDefaultCommand(m_arcadeMotor);
    SendableRegistry.add(m_moveForDistance.getSendable(), "Move For Distance");
    Shuffleboard.getTab("SmartDashBoard").add(m_moveForDistance.getSendable()).withWidget("Move For Distance");
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return m_moveForTime;
  }
}
