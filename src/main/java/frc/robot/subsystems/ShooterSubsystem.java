// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;

import com.revrobotics.spark.ClosedLoopSlot;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;






public class ShooterSubsystem extends SubsystemBase {
  private SparkMaxConfig config;
  private SparkMax sparkMaxShooterMotor;
  private SparkClosedLoopController pidController;
  private AbsoluteEncoder encoder;

  public ShooterSubsystem() {
  
    sparkMaxShooterMotor = new SparkMax(3, MotorType.kBrushless);
    pidController = sparkMaxShooterMotor.getClosedLoopController();
    encoder = sparkMaxShooterMotor.getAbsoluteEncoder();

  
    config = new SparkMaxConfig();

    config.absoluteEncoder
        .positionConversionFactor(1);

    config.closedLoop
                    .pid(Constants.ShooterConstant.kP, Constants.ShooterConstant.kI , Constants.ShooterConstant.kD)
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .maxOutput(Constants.ShooterConstant.kMaxOutput)
                    .minOutput(Constants.ShooterConstant.kMinOutput)

                    .p(0.0001, ClosedLoopSlot.kSlot1)
                    .i(0,ClosedLoopSlot.kSlot1)
                    .d(0,ClosedLoopSlot.kSlot1)
                    .outputRange(-1,1, ClosedLoopSlot.kSlot1);

    sparkMaxShooterMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      
  }
 
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public Command setGoal(double pos){
    return run(
      () -> {
            pidController.setReference(pos, ControlType.kPosition);
      }
    );
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Encoder Position:", encoder.getPosition());
    SmartDashboard.putNumber("Velocity", encoder.getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
