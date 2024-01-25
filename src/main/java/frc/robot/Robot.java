// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.XboxController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final int leftFrontDeviceID = 10; 
  private static final int leftBackDeviceID = 11; 
  private static final int rightFrontDeviceID = 12; 
  private static final int rightBackDeviceID = 13; 
  private MecanumDrive mecanumDrive;
  private XboxController xboxMovementController;
  private XboxController xboxInteractionController;
  private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  private CANSparkMax frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor;
  private double frontLeftPower = 0;
  private double backLeftPower = 0;
  private double frontRightPower = 0;
  private double backRightPower = 0;
  private SendableChooser<Boolean> controlModeChooser = new SendableChooser<>();
  private SendableChooser<Boolean> controllerModeChooser = new SendableChooser<>();
  private boolean fieldCentricControl; 
  private boolean twoControllerMode;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    frontLeftMotor = new CANSparkMax(leftFrontDeviceID, MotorType.kBrushless);
    rearLeftMotor = new CANSparkMax(leftBackDeviceID, MotorType.kBrushless);
    frontRightMotor = new CANSparkMax(rightFrontDeviceID, MotorType.kBrushless);
    rearRightMotor = new CANSparkMax(rightBackDeviceID, MotorType.kBrushless);
    mecanumDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

    xboxMovementController = new XboxController(0);  // Replace 0 with the port number of your movement Xbox controller
    xboxInteractionController = new XboxController(1);  // Replace 1 with the port number of your interaction Xbox controller

    // Add options to the controlModeChooser
    controlModeChooser.addOption("Field-Centric Control", true);
    controlModeChooser.addOption("Robot-Centric Control", false);

    // Add options to the controllerModeChooser
    controllerModeChooser.addOption("One Controller Mode", false);
    controllerModeChooser.addOption("Two Controller Mode", true);
    
    // Put the chooser on the SmartDashboard
    SmartDashboard.putData("Control Mode", controlModeChooser);
    SmartDashboard.putData("Controller Mode", controllerModeChooser);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    if (controlModeChooser.getSelected() != null && controllerModeChooser.getSelected() != null) {
      fieldCentricControl = controlModeChooser.getSelected();
      twoControllerMode = controllerModeChooser.getSelected();
    } else {
      // Handle the case where one or both values are null
      // You might want to set default values or handle it in a way that makes sense for your application.
      fieldCentricControl = false;
      twoControllerMode = false;
      
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    // Controller Inputs
    double yAxisValue = -xboxMovementController.getLeftY(); // Remember, Y stick value is reversed
    double xAxisValue = xboxMovementController.getLeftX() * 1.1; // Counteract imperfect strafing
    double zAxisValue; // Declare z outside the conditional statement
    if (twoControllerMode == true)
      {zAxisValue = xboxInteractionController.getLeftX();
    } else {
      zAxisValue = xboxMovementController.getRightX();
    }
    
    

    
    // Read our Trottle, but it's going to be a range of numbers between -1 and 1. We want this to be a range between 0 and 1 so we add 1 and half it
    //double throttleMultiple = (0.5 * (-Joystick.getThrottle() + 1)) * 0.7; // DEPRICATED WITH XBOX CONTROLLERS
    
    // TODO: push button thorttle adjustment

    // Hard coded throttle for now 
    double throttleMultiple = 0.4;

    // if field centric control is off, this is easy and we just take our values and apply them. 
    // Otherwise we have to morph them to consider the robot's direction on the field

    if (fieldCentricControl == false){

      mecanumDrive.driveCartesian(xAxisValue, yAxisValue, zAxisValue);

    } else if (fieldCentricControl == true){

      // Z doesn't Change?!?

      // double gyroAngle = gyro.getAngle(); // Grab our current Angle from the Gyro
      mecanumDrive.driveCartesian(xAxisValue, yAxisValue, zAxisValue, gyro.getRotation2d());

    }

    // Creates a SlewRateLimiter that limits the rate of change of the signal to 0.5 units per second
    SlewRateLimiter frontLeftFilter = new SlewRateLimiter(0.5);
    SlewRateLimiter frontRightFilter = new SlewRateLimiter(0.5);
    SlewRateLimiter backLeftFilter = new SlewRateLimiter(0.5);
    SlewRateLimiter backRightFilter = new SlewRateLimiter(0.5);
    // Apply Speeds to the left motors
    
    frontLeftMotor.set(frontLeftFilter.calculate(frontLeftPower) * throttleMultiple);
    rearLeftMotor.set(backLeftFilter.calculate(backLeftPower) * throttleMultiple);

    // The right side rotates counter due to the physical motor's orientation
    frontRightMotor.set(frontRightFilter.calculate(-frontRightPower) * throttleMultiple);
    rearRightMotor.set(backRightFilter.calculate(-backRightPower) * throttleMultiple);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
