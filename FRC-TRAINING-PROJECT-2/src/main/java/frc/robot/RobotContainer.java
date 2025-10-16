// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ArcadeMotor;
import frc.robot.constants.IOConstants;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  private Drivetrain m_motor = new Drivetrain();
  private Joystick m_joystick = new Joystick(IOConstants.kJoystickPort);
  private ArcadeMotor m_arcadeMotor = new ArcadeMotor(m_motor, m_joystick);



  public RobotContainer() {
    m_motor.setDefaultCommand(m_arcadeMotor);
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
