 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwervePod extends SubsystemBase {
  private CANSparkMax driveMotor;
  private CANSparkMax swerveMotor;
  private CANcoder compass;
  private RelativeEncoder sonic;
  private PIDController directionControl;
  private double encoderOffset; 
  private double fieldAdjust;
  private int podID;


  public SwervePod(int driveID, double encoderOffset) {
    driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    swerveMotor = new CANSparkMax(driveID + 10, MotorType.kBrushless);
    this.encoderOffset = encoderOffset;
    fieldAdjust = 0;

    sonic = driveMotor.getEncoder();
    compass = new CANcoder(driveID + 20);
    podID = driveID;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getSpeed() {
    return sonic.getVelocity()/8.14;

  }

  public double getDistance() {
    return sonic.getPosition()/8.14;

  }

  public void setFieldAdjust(double fieldAdjust) {
    this.fieldAdjust = fieldAdjust;
  }
  public double getAngle() {
    double angle = compass.getAbsolutePosition().getValue() + encoderOffset - fieldAdjust;
    if(angle > 180) {
      angle -= 360;
    }
    if(angle < -180) {
      angle +=360;
    }
    return angle;
  }
  public void turnPod(double turn) {
    swerveMotor.set(turn);
  }
  public void spinWheel(double speed) {
    if(Math.abs(speed) > .5) {
      driveMotor.set(speed);
    } 
    else {
      driveMotor.set(0);
    }
  }
  public void setDirection(double direction) {
      double altDir;
      if(direction > 0) {
        altDir = direction - 180;
      }
      else {
        altDir = direction + 180;
      }
      if(Math.abs(getAngle() - direction) > Math.abs(getAngle() - altDir)) {
        direction = altDir;
        driveMotor.setInverted(true);
      }
      else {
      driveMotor.setInverted(false);
      } 
      turnPod(directionControl.calculate(getAngle(), direction));
  }
  public void drivePod(double drive, double direction) {
    //feildAdjust=yaw;
    setDirection(direction);
    if(Math.abs(drive) > .1){
      driveMotor.set(drive);
    }
    else {
      driveMotor.set(0);
    }
  }
}
