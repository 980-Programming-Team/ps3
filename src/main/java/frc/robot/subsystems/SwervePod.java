 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwervePod extends SubsystemBase {
  private CANSparkMax driveMotor;
  private CANSparkFlex swerveMotor;
  private CANcoder compass;
  private RelativeEncoder sonic;
  private int positionID;

  private PIDController directionControl; 

  private double S_P = 1.0/150;
  private double S_I = 0;
  private double S_D = 0;
  
  private PIDController velocityControl;
  private final double SPEED_LIMIT = 11;//Actual top speed is about 11.7, limiting for vel control
  private SimpleMotorFeedforward ff;
  private boolean manualOveride;

  private final double D_P = 0.5;
  private final double D_I = 0.0;
  private final double D_D = 0.0;
  private final double KS = 0.05;
  private final double KV = 12.0 / 11.5; // actual max speed



  /*public SwervePod(int driveID) {
    driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    swerveMotor = new CANSparkFlex(driveID + 10, MotorType.kBrushless);
    //fieldAdjust = 0;

    sonic = driveMotor.getEncoder();
    compass = new CANcoder(driveID + 20);
    podID = driveID;
    directionControl = new PIDController(S_P, S_I, S_D);
    directionControl.enableContinuousInput(-180, 180);
    sonic.setPositionConversionFactor(((4 / 12.0) * Math.PI) / 8.14);
    sonic.setVelocityConversionFactor(((4 / 12.0) * Math.PI) / (8.14 * 60));
  }*/

  public SwervePod(int positionID, int podID) {
    this.positionID = positionID;
    driveMotor = new CANSparkMax(positionID, MotorType.kBrushless);
    swerveMotor = new CANSparkFlex(podID + 10, MotorType.kBrushless);
    //fieldAdjust = 0;

    sonic = driveMotor.getEncoder();
    sonic.setPositionConversionFactor(((4 / 12.0) * Math.PI) / 8.14);//circumference for 4" wheel divided by 12" to a foot / gear ratio * -> feet
    sonic.setVelocityConversionFactor(((4 / 12.0) * Math.PI) / (8.14 * 60));//circumference for 4" wheel divided by 12" to a foot / gear ratio * convert to seconds -> feet per second
  
    compass = new CANcoder(podID + 20);

    directionControl = new PIDController(S_P, S_I, S_D);
    directionControl.enableContinuousInput(-180, 180);
    
    manualOveride = false;
    velocityControl = new PIDController(D_P, D_I, D_D);
    ff = new SimpleMotorFeedforward (KS, KV);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("speed" + positionID, getSpeed()); 
    SmartDashboard.putNumber("distance" + positionID, getDistance());
    SmartDashboard.putNumber("angle" + positionID, getAngle());
  }

  public double getSpeed() {
    return sonic.getVelocity();

  }

  public double getDistance() {
    return sonic.getPosition();

  }

  public double getAngle() {
    double angle = compass.getAbsolutePosition().getValue() * 360;
    return angle;
  }

  public void turnPod(double turn) {
    swerveMotor.set(turn);
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
    //TODO put in velocity control
    setDirection(direction);
    if(Math.abs(drive) > .1){
      driveMotor.set(drive);
    }
    else {
      driveMotor.set(0);
    }
  }
  public void setManualOveride(boolean manualOveride) {
    this.manualOveride = manualOveride;
    
  }
}
