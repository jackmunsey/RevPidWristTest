// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private Wrist mWrist;
  private CommandXboxController mXboxController;

  public RobotContainer() {
    CANSparkMax leftMotor;
    CANSparkMax rightMotor;
    

    leftMotor = new CANSparkMax(10, MotorType.kBrushless);
    rightMotor = new CANSparkMax(11, MotorType.kBrushless);

    rightMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    leftMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    rightMotor.setInverted(true);
    leftMotor.setInverted(false);

    leftMotor.follow(rightMotor);

    this.mWrist = new Wrist(rightMotor, leftMotor);

    mXboxController = new CommandXboxController(0);

    configureBindings();
  }

  private void configureBindings() {
    mXboxController.a().onTrue(Commands.run(() -> mWrist.setPoint(0.25), mWrist))
    .onFalse(Commands.runOnce(() -> mWrist.setPoint(0), mWrist));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
