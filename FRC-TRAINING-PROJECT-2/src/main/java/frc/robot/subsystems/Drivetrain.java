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
import frc.robot.constants.MotorConstants;

public class Drivetrain extends SubsystemBase {

  TalonSRX m_talonLeft = new TalonSRX(MotorConstants.kTalonLeftID);
  TalonSRX m_talonRight = new TalonSRX(MotorConstants.kTalonRightID);

  //VictorSPX Motor initialization
  VictorSPX m_victorLeft = new VictorSPX(MotorConstants.kVictorLeftID);
  VictorSPX m_victorRight = new VictorSPX(MotorConstants.kVictorRightID);



  /** Creates a new Motor. */
  public Drivetrain() {
    //No idea why yet
    m_talonRight.setNeutralMode(NeutralMode.Brake);
    m_talonLeft.setNeutralMode(NeutralMode.Brake);
    
    //No idea why this needs to be inverted
    m_talonRight.setInverted(MotorConstants.kisInverted);

    //Victor motors
    m_victorLeft.follow(m_talonLeft);
    m_victorRight.follow(m_talonRight);

    //no idea why this needs to be inverted. will fix later
    m_victorRight.setInverted(MotorConstants.kisInverted);
    
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
}
