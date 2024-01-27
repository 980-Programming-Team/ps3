// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
  private final double L = 22 + (3/16);
  private final double W = 22 + (13/16);
  private double r;
  private final double POD_1OFFSET = 0; 
  private final double POD_2OFFSET = 0;
  private final double POD_3OFFSET = 0;
  private final double POD_4OFFSET = 0;

  private SwervePod backleft;
  private SwervePod backright;
  private SwervePod frontright;
  private SwervePod frontleft;
  private PigeonIMU imu;
  private PigeonIMU.GeneralStatus imuStatus;
  private int imuErrorCode;
  private double[] ypr;

  private int whichPod;

  public SwerveDrive() {
   imu = new PigeonIMU(20);
   imuStatus = new PigeonIMU.GeneralStatus();
   ypr = new double [3];
   imu.setYaw(0);
   backleft = new SwervePod(1,POD_1OFFSET);
   backright = new SwervePod(2,POD_2OFFSET);
   frontright = new SwervePod(3,POD_3OFFSET);
   frontleft = new SwervePod(4,POD_4OFFSET);
   whichPod = 1;
   r = Math.sqrt((L * L) + (W * W));
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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

    SmartDashboard.putNumber("BR Speed", backRightSpeed);
    SmartDashboard.putNumber("BR Angle", backRightAngle);
    SmartDashboard.putNumber("BL Speed", backLeftSpeed);
    SmartDashboard.putNumber("BL Angle", backLeftAngle);
    SmartDashboard.putNumber("FR Speed", frontRightSpeed);
    SmartDashboard.putNumber("FR Angle", frontRightAngle);
    SmartDashboard.putNumber("FL Speed", frontLeftSpeed);
    SmartDashboard.putNumber("FL Angle", frontLeftAngle);
    SmartDashboard.putNumber("x1", x1);
    SmartDashboard.putNumber("y1", y1);
    SmartDashboard.putNumber("x2", x2);

    backright.drivePod (backRightSpeed, backRightAngle);
    backleft.drivePod (backLeftSpeed, backLeftAngle);
    frontright.drivePod (frontRightSpeed, frontRightAngle);
    frontleft.drivePod (frontLeftSpeed, frontLeftAngle);
    
  }


}
