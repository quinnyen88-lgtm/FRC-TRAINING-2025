// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;




import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {

  TalonSRX m_talonLeft = new TalonSRX(DrivetrainConstants.kTalonLeftID);
  TalonSRX m_talonRight = new TalonSRX(DrivetrainConstants.kTalonRightID);

  //VictorSPX Motor initialization
  VictorSPX m_victorLeft = new VictorSPX(DrivetrainConstants.kVictorLeftID);
  VictorSPX m_victorRight = new VictorSPX(DrivetrainConstants.kVictorRightID);



  /** Creates a new Motor. */
  public Drivetrain() {
    //so the robot will hold its position
    m_talonRight.setNeutralMode(NeutralMode.Brake);
    m_talonLeft.setNeutralMode(NeutralMode.Brake);
    
    //so the motors spin in opposite directions but when installed they will spin towards the same side
    m_talonRight.setInverted(DrivetrainConstants.kisInverted);

    //so the motors match the same speed as the primary motors
    m_victorLeft.follow(m_talonLeft);
    m_victorRight.follow(m_talonRight);

    //so the motors spin in opposite directions but when installed they will spin towards the same side 
    m_victorRight.setInverted(DrivetrainConstants.kisInverted);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //for testing purposes. makes it easier
    SmartDashboard.putNumber( "Left Motor Speed", getLeftSpeed());
    SmartDashboard.putNumber( "Right Motor Speed", getRightSpeed());
  }

  public void setRightSpeed(double rightSpeed) {
    //percent output is the percentage of the motor going forward or reverse or stop. 0.0 being stop
    m_talonRight.set(ControlMode.PercentOutput, rightSpeed);
  }

  public double getRightSpeed() {
    return m_talonRight.getMotorOutputPercent();
  }

  public void setLeftSpeed(double leftSpeed) {
    m_talonLeft.set(ControlMode.PercentOutput, leftSpeed);
  }

  public double getLeftSpeed() {
    return m_talonLeft.getMotorOutputPercent();

    
  }

  public double getCurrentTickLeftPosition() {
    return m_talonLeft.getSelectedSensorPosition();
  }

  public double getCurrentTickRightPosition() {
    return m_talonRight.getSelectedSensorPosition();
  }
}
