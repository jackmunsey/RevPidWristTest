// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
// import com.revrobotics.SparkMaxAlternateEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  AbsoluteEncoder encoder;
  SparkPIDController pID;
  double p = 2.5;
  double i = 0;
  double d = 0.3;
  double zone = 0;
  double fF = 0;
  double minOutput = -2;
  double maxOutput = 2;
  double setPoint = 0;

  public Wrist(CANSparkMax rightMotor, CANSparkMax leftMotor) {

    pID = leftMotor.getPIDController();
    //encoder = leftMotor.getAlternateEncoder();
    encoder = leftMotor.getAbsoluteEncoder(Type.kDutyCycle);
    pID.setFeedbackDevice(encoder);
    pID.setPositionPIDWrappingEnabled(true);
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
    pID.setReference(setPoint, ControlType.kPosition);
    SmartDashboard.putNumber("setPoint", setPoint);
    SmartDashboard.putNumber("encoder position", encoder.getPosition());
    SmartDashboard.putNumber("velocity", encoder.getVelocity());
    // SmartDashboard.putNumber("output", rightMotor.GetAppliedOutput());
  }
}
