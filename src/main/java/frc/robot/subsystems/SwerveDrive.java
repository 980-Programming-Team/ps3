// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
  private final double L = 22 + (3 / 8.0);//in inches must be measured axel to axel
  private final double W = 22 + (3 / 8.0);
  private double r;

  private SwervePod backleft;
  private SwervePod backright;
  private SwervePod frontright;
  private SwervePod frontleft;

  private PigeonIMU imu;
  private PigeonIMU.GeneralStatus imuStatus;
  private int imuErrorCode;
  private double[] ypr;

  //TODO add boolean turbo variable

  private boolean fieldOriented;

  public SwerveDrive() {
   imu = new PigeonIMU(30);
   imuStatus = new PigeonIMU.GeneralStatus();
   ypr = new double [3];
   imu.setYaw(0);

   //first argument is the position on the robot (corresponds to the Spark ID not attached to the pod)
   //second argument is the pod currently in that position (corresponds to the ID of the vortex and encoder on the pod)
   backleft = new SwervePod(4 , 4);
   backright = new SwervePod(5 , 5);
   frontleft = new SwervePod(3 , 3);
   frontright = new SwervePod(2 , 6);

   r = Math.sqrt((L * L) + (W * W));

   //TODO set turbo variable to default to false
   fieldOriented = true;
  }

  //TODO add field orented setters and turbo setters

  public void resetYaw(){
    imu.setYaw(0);
  }  

  @Override
  public void periodic() {
    imuErrorCode = imu.getGeneralStatus(imuStatus).value;
    imu.getYawPitchRoll(ypr);
    SmartDashboard.putNumber("IMU Health", imuErrorCode);
    SmartDashboard.putNumber("IMU Yaw", ypr[0]);
  }

  public void podDriver(double x1, double y1, double x2) {
    if (Math.abs(x1) <= .1) {
      x1 = 0;
    }
     if (Math.abs(y1) <= .1) {
      y1 = 0;
    }
     if (Math.abs(x2) <= .1) {
      x2 = 0;
    }

    //TODO setup turbo system to reduce top speed

    if(fieldOriented){
      double yawRad = ypr[0] * Math.PI / 180;
      double temp = y1 * Math.cos(yawRad) + x1 * Math.sin(yawRad);
      x1 = -y1 * Math.sin(yawRad) + x1 * Math.cos(yawRad);
      y1 = temp;
    }

    double a = x1 - x2 * (L / r);
    double b = x1 + x2 * (L / r);
    double c = y1 - x2 * (W / r);
    double d = y1 + x2 * (W / r);

    double backRightSpeed = Math.sqrt((a * a) + (d * d));
    double backLeftSpeed = Math.sqrt((a * a) + (c * c));
    double frontRightSpeed = Math.sqrt((b * b) + (d * d));
    double frontLeftSpeed = Math.sqrt((b * b) + (c * c));

    double backRightAngle = (Math.atan2 (a, d) / Math.PI) * 180;
    double backLeftAngle = (Math.atan2 (a, c) / Math.PI) * 180;
    double frontRightAngle = (Math.atan2 (b, d) / Math.PI) * 180;
    double frontLeftAngle = (Math.atan2 (b, c) / Math.PI) * 180;

    backright.drivePod (backRightSpeed, backRightAngle);
    backleft.drivePod (backLeftSpeed, backLeftAngle);
    frontright.drivePod (frontRightSpeed, frontRightAngle);
    frontleft.drivePod (frontLeftSpeed, frontLeftAngle);
    
  }

  //TODO add stop method to halt drive and go back to 0 angle
}
