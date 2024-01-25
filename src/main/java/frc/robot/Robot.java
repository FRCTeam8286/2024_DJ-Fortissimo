// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // CAN ID Values for devices attached to CAN bus
  private static final int leftFrontDeviceID = 10; 
  private static final int leftBackDeviceID = 11; 
  private static final int rightFrontDeviceID = 12; 
  private static final int rightBackDeviceID = 13; 

  // Define Controller Objects, we'll be associating these with controllers later
  private XboxController xboxMovementController;
  private XboxController xboxInteractionController;

  // Create Objects related to drive train
  private MecanumDrive mecanumDrive;
  private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  private CANSparkMax frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor;

  // Create Objects and Variables related to UI choices 
  private SendableChooser<Boolean> controlModeChooser = new SendableChooser<>();
  private SendableChooser<Boolean> controllerModeChooser = new SendableChooser<>();

  // These boolean variables are used to determine control options
  private boolean fieldCentricControl;
  private boolean twoControllerMode;

  // Creates SlewRateLimiter objects for each axis that limits the rate of change. This value is max change per second. For most imports, the range here is  -1 to 1 
  SlewRateLimiter filterX = new SlewRateLimiter(1); 
  SlewRateLimiter filterY = new SlewRateLimiter(1);
  SlewRateLimiter filterZ = new SlewRateLimiter(1);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Define the motors
    frontLeftMotor = new CANSparkMax(leftFrontDeviceID, MotorType.kBrushless);
    rearLeftMotor = new CANSparkMax(leftBackDeviceID, MotorType.kBrushless);
    frontRightMotor = new CANSparkMax(rightFrontDeviceID, MotorType.kBrushless);
    rearRightMotor = new CANSparkMax(rightBackDeviceID, MotorType.kBrushless);

    // Invert the right motors
    frontRightMotor.setInverted(true);
    rearRightMotor.setInverted(true);

    // Create a new mecanumDrive Object and associate the motors with it  
    mecanumDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

    // Initiate Xbox Controllers
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

    // If statement to see if our Mode Choser outputs worked, and if not, have some fall back values (Mostly for Simulation Mode)
    if (controlModeChooser.getSelected() != null && controllerModeChooser.getSelected() != null) {
      
      // Set variables fieldCentricControl and twoControllerMode to options selected on interactive chooser by the operators
      fieldCentricControl = controlModeChooser.getSelected();
      twoControllerMode = controllerModeChooser.getSelected();

    } else {

      // Handle the case where one or both values are null (simulation mode)
      fieldCentricControl = false;
      twoControllerMode = false;
      
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // Define Controller Inputs

    // TODO: Add a way for driver to switch between field oriented or robot oriented controls

    // TODO: Add a way for driver to calibrate (reset) the Gyroscope

    // TODO: Add a way to adjust top speed with driver's controller "bumpers"

    double yAxisValue = -xboxMovementController.getLeftY(); // Remember, Y stick value is reversed
    double xAxisValue = xboxMovementController.getLeftX() * 1.1; // Counteract imperfect strafing
    double zAxisValue; // Declare z outside the conditional statement

    if (twoControllerMode == true) // zAxis changes based on if we have two controllers (operators) or not
      {zAxisValue = xboxInteractionController.getLeftX();
    } else {
      zAxisValue = xboxMovementController.getRightX();
    }

    // Sending Values to Drive System
    // We only want to specify gyro rotation if we've opted to use field centric controls

    if (fieldCentricControl == false){ 

      mecanumDrive.driveCartesian(filterX.calculate(xAxisValue), filterY.calculate(yAxisValue), filterZ.calculate(zAxisValue));

    } else if (fieldCentricControl == true){

      mecanumDrive.driveCartesian(filterX.calculate(xAxisValue), filterY.calculate(yAxisValue), filterZ.calculate(zAxisValue), gyro.getRotation2d());

    }

    
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
