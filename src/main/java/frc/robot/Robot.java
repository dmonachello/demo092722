// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  NetworkTable table;

  //private WPI_VictorSPX cimLeftFrontMotor  = new WPI_VictorSPX(Constants.LEFT_FRONT_MOTOR);
  //private WPI_VictorSPX cimLeftBackMotor   = new WPI_VictorSPX(Constants.LEFT_BACK_MOTOR);
  //private WPI_VictorSPX cimRightFrontMotor = new WPI_VictorSPX(Constants.RIGHT_FRONT_MOTOR);
  //private WPI_VictorSPX cimRightBackMotor  = new WPI_VictorSPX(Constants.RIGHT_BACK_MOTOR);

  private VictorSP vicSpAndy1 = new VictorSP(Constants.VICTOR_SP_PWM_PORT);

  //private XboxController xboxController = new XboxController(Constants.XBOX_USB_PORT);

  DifferentialDrive diffDrv;
  
  static double TargetHeight = 42.0;
  static double CameraHeight = 27.5; // camera height
  static double CameraToTargetDistance = 85.0;

  double calcCamAngle(double targAngle,        // 
                double targHeight,             // target height -- remeasure this!!!
                double camHeight,
                double cam2TargDis
                )
  {
    double camAngle = (Math.atan(targHeight - camHeight) / cam2TargDis) - targAngle;
    return camAngle;
  }


  double calcRealDistance(double targHeight,        // 
                double targAngle,             // target height -- remeasure this!!!
                double camHeight,
                double camAngle
                )
  {
    double angleToGoalDegrees = targAngle + camAngle;
    SmartDashboard.putNumber("angleToGoalDegrees", angleToGoalDegrees);
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    // d = (h2-h1) / tan(a1+a2)
    double varDistance =  (targHeight - camHeight) / (Math.tan(angleToGoalRadians));
    SmartDashboard.putNumber("varDistance", varDistance);
    double h3 = (targHeight - camHeight);
    SmartDashboard.putNumber("h3", h3);
    return varDistance;
  }

    /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */  
  @Override
    // setup the drive train
    public void robotInit() {
      /*
    cimLeftFrontMotor.configFactoryDefault();
    cimLeftBackMotor.configFactoryDefault();
    cimRightFrontMotor.configFactoryDefault();
    cimRightBackMotor.configFactoryDefault();

    cimLeftBackMotor.follow(cimLeftFrontMotor);
    cimRightBackMotor.follow(cimRightFrontMotor);
  
    cimLeftFrontMotor.setInverted(true);
    cimLeftBackMotor.setInverted(true);
    diffDrv  = new DifferentialDrive(cimLeftFrontMotor, cimRightFrontMotor);
*/

    table = NetworkTableInstance.getDefault().getTable("limelight");
    vicSpAndy1.set(0);

    
  }

  @Override
  public void robotPeriodic() {
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    //read values periodically
    double x = tx.getDouble(0.0);
    double targetAngle = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", targetAngle);
    SmartDashboard.putNumber("LimelightArea", area);

    vicSpAndy1.set(0);
//    diffDrv.tankDrive(0, 0); // to move the robot

    double camAngle = calcCamAngle(targetAngle,       
                                   TargetHeight,            
                                   CameraHeight,
                                   CameraToTargetDistance);
    SmartDashboard.putNumber("camAngle", camAngle);
    camAngle = 0.0; 
    SmartDashboard.putNumber("camAngleX", camAngle);
    
    double realDistance = calcRealDistance(TargetHeight,  
                                           targetAngle,
                                           CameraHeight,
                                           camAngle);
    SmartDashboard.putNumber("realDistance", realDistance);
     


    }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
