// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  AbsoluteEncoder encoder;
  SparkPIDController pID;
  double p = 3;
  double i = 0;
  double d = 0;
  double zone = 0;
  double fF = 0;
  double minOutput = 6;
  double maxOutput = 6;
  double setPoint = 0;

  public Wrist(CANSparkMax rightMotor, CANSparkMax leftmotor) {

    pID = rightMotor.getPIDController();
    encoder = rightMotor.getAbsoluteEncoder(Type.kDutyCycle);
    pID.setFeedbackDevice(encoder);
    
    pID.setP(p);
    pID.setI(i);
    pID.setD(d);
    pID.setIZone(zone);
    // pID.setFF(fF);
    pID.setOutputRange(minOutput, maxOutput);
  }

  public void setPoint(double setPoint) {
    this.setPoint = setPoint;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pID.setReference(setPoint, ControlType.kDutyCycle);
    SmartDashboard.putNumber("setPoint", setPoint);
    SmartDashboard.putNumber("encoder position", encoder.getPosition());
  }
}
