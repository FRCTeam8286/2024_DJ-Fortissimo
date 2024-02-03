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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // Setup Debug Mode
  private static final boolean debug  = true;

  // CAN ID values for devices attached to CAN bus
  /**
  * CAN Bus Mapping:
  * - Motor Controllers:
  *   - Front Left Motor: CAN ID 10
  *   - Back Left Motor: CAN ID 11
  *   - Front Right Motor: CAN ID 12
  *   - Back Right Motor: CAN ID 13
  */
  private static final int leftFrontDeviceID = 10; 
  private static final int leftBackDeviceID = 11; 
  private static final int rightFrontDeviceID = 12; 
  private static final int rightBackDeviceID = 13; 
  
  // Assuming Blinkin LED controller is connected to PWM port 0
  private static final int blinkinPWMChannel = 0;
  
  // Create PWMSparkMax for controlling Blinkin LED
  private PWMSparkMax blinkinLED = new PWMSparkMax(blinkinPWMChannel);

  // Define Controller Objects, we'll be associating these with controllers later
  private XboxController xboxMovementController;
  private XboxController xboxInteractionController;

  // Create objects related to drive train
  private MecanumDrive mecanumDrive;
  private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  private CANSparkMax leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;

  // Create objects and variables related to UI choices 
  private SendableChooser<Boolean> controlModeChooser = new SendableChooser<>();
  private SendableChooser<Boolean> controllerModeChooser = new SendableChooser<>();

  // These boolean variables are used to determine control options
  private boolean fieldCentricControl;
  private boolean twoControllerMode;

  // Creates SlewRateLimiter objects for each axis that limits the rate of change. This value is max change per second. For most imports, the range here is  -1 to 1 
  SlewRateLimiter filterX = new SlewRateLimiter(1); 
  SlewRateLimiter filterY = new SlewRateLimiter(1);
  SlewRateLimiter filterZ = new SlewRateLimiter(1);
  
  // Movment Modifiers (so the robot doesn't go so fast)
  private static final double yModifier = 0.7;
  private static final double xModifier = 0.7;
  private static final double zModifier = 0.5;
  
  // Controller Dead-zone Values (to help eleminate drift)
  private static final double yDeadZone = 0.2;
  private static final double xDeadZone = 0.2;
  private static final double zDeadZone = 0.2;
  
  // Setup some standard LED colors to use with the Blinkin
  /**
  * - LED Color Map:
  *
  *  - ledRed: Indicates the robot is in Teleop mode but doesn't have the game piece loaded
  *  - ledBlue: Indicates the robot is busy initializing or calibrating
  *  - ledBlack: Indicates the robot is stopped or no specific condition
  *  - ledGreen: Indicates the robot is in Teleop mode and has the game piece loaded
  *  - ledPattern: Used when robot is first turned on to show Farwell School Colors
  */
  private static final double ledRed = 0.61;
  private static final double ledGreen = 0.77;
  private static final double ledBlue = 0.87;
  private static final double ledBlack = 0.99;
  private static final double ledPattern = 0.51;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // If debug mode is on, write a line that lets us know what mode we're entering
    if (debug) { System.out.println("Entering robotInit Phase");}

    // Have LEDs blink in farwell school colors pattern
    blinkinLED.set(ledPattern);
	  
    // Define the motors

    leftFrontMotor = new CANSparkMax(leftFrontDeviceID, MotorType.kBrushless);
    leftBackMotor = new CANSparkMax(leftBackDeviceID, MotorType.kBrushless);
    rightFrontMotor = new CANSparkMax(rightFrontDeviceID, MotorType.kBrushless);
    rightBackMotor = new CANSparkMax(rightBackDeviceID, MotorType.kBrushless);

    // Invert the right motors
    
    leftFrontMotor.setInverted(false);
    leftBackMotor.setInverted(false);
    rightFrontMotor.setInverted(true);
    rightBackMotor.setInverted(true);
    
    // Create a new mecanumDrive Object and associate the motors with it  
    mecanumDrive = new MecanumDrive(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor);

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
    SmartDashboard.putData("Control Mode Chooser", controlModeChooser);
    SmartDashboard.putData("Controller Mode Chooser", controllerModeChooser);

    // Calibrate and Reset Gyro
    gyro.reset();
    gyro.calibrate();

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
    // If debug mode is on, write a line that lets us know what mode we're entering
    if (debug) { System.out.println("Entering autonomousInit Phase");}
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  // If debug mode is on, write a line that lets us know what mode we're entering
  if (debug) { System.out.println("Entering teleopInit Phase");}

	// Set LEDs to Blue so operator can tell the robot is busy initializing
	blinkinLED.set(ledBlue);
    // If statement to see if our Mode Choser outputs worked, and if not, have some fall back values (Mostly for Simulation Mode)
    if (controlModeChooser.getSelected() != null && controllerModeChooser.getSelected() != null) {
      
      // Set variables fieldCentricControl and twoControllerMode to options selected on interactive chooser by the operators
      fieldCentricControl = controlModeChooser.getSelected();
      twoControllerMode = controllerModeChooser.getSelected();

    } else {

      // Handle the case where one or both values are null (simulation mode). Also log message because that is probably interesting to see
      System.err.println("Error: Unable to retrieve control mode or controller mode from choosers. Using default values.");
      fieldCentricControl = false;
      twoControllerMode = false;
      
    }

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
	// Set LEDs to Red so operator can tell the robot is in Teleop mode but no item is picked up
	blinkinLED.set(ledRed);
    
	// Define Controller Inputs

    /**
     * Button Mapping:
     * - XboxMovementController (Port 0):
     *   - Left Y Axis: Drive forward/reverse
     *   - Left X Axis: Strafe left/right
     *   - Right X Axis: Rotate (when in one-controller mode)
     *   - Start Button: Calibrate Gyro
     *   - Y Button: Toggle Field Centric
     *   - Right Bumper: Increase top speed
     *   - Left Bumper: Decrease top speed
     *
     * - XboxInteractionController (Port 1):
     *   - Left X Axis: Rotate (when in two-controller mode)
     */

    // TODO: Add a way to adjust top speed with driver's controller "bumpers"

    double yAxisValue = -xboxMovementController.getLeftY() * yModifier; // Remember, Y stick value is reversed
    double xAxisValue = xboxMovementController.getLeftX() * xModifier; // Counteract imperfect strafing
    double zAxisValue; // Declare z outside the conditional statement

    
    if (twoControllerMode == true) // zAxis changes based on if we have two controllers (operators) or not
      {zAxisValue = xboxInteractionController.getLeftX() * zModifier;
    } else {
      zAxisValue = xboxMovementController.getRightX() * zModifier;
    }

    // Apply Deadzones
    if (Math.abs(yAxisValue) < yDeadZone) {yAxisValue = 0;}
    if (Math.abs(xAxisValue) < xDeadZone) {xAxisValue = 0;}
    if (Math.abs(zAxisValue) < zDeadZone) {zAxisValue = 0;}
    
    // Check if the "Start" button is pressed on the movement controller
    if (xboxMovementController.getStartButtonPressed()) {

      // Set LEDs to Blue so operator can tell the robot is busy calibrating
      blinkinLED.set(ledBlue);
      // Reset the gyro when the "Start" button is pressed
      gyro.calibrate();
      System.out.println("Calibrating Gyro");

    }

    // If Y is pressed, flip between field centric and robot centric controlls
    if (xboxInteractionController.getYButtonPressed()) {
      fieldCentricControl = !fieldCentricControl;
    }

    // If debug mode is on, provide diagnostic information to the smart dashboard
    if (debug) {

      // Output X, Y, and Z values to Smart Dashboard for Troubleshooting
      SmartDashboard.putNumber("Current X Value", xAxisValue);
      SmartDashboard.putNumber("Current Y Value", yAxisValue);
      SmartDashboard.putNumber("Current Z Value", zAxisValue);

      // Output Gyro value to Smart Dashboard for troubleshooting
      SmartDashboard.putNumber("Current Gyro Rotation", gyro.getRotation2d().getDegrees());

    }

    // Sending Values to Drive System
    // We only want to specify gyro rotation if we've opted to use field centric controls

    if (fieldCentricControl){
        mecanumDrive.driveCartesian(
          filterY.calculate(yAxisValue), 
          filterX.calculate(xAxisValue), 
          filterZ.calculate(zAxisValue), 
          gyro.getRotation2d()
        );
    } else{
        mecanumDrive.driveCartesian(
          filterY.calculate(yAxisValue), 
          filterX.calculate(xAxisValue), 
          filterZ.calculate(zAxisValue)
        );
    }

    //if debug mode is on, provide diagnostic data to the smart dashboard
    if (debug) {

      // Output Motor Values to Smart Dashboard for troubleshooting
      SmartDashboard.putNumber("Left Front Motor Power", leftFrontMotor.getAppliedOutput());
      SmartDashboard.putNumber("Left Back Motor Power", leftBackMotor.getAppliedOutput());
      SmartDashboard.putNumber("Right Front Motor Power", rightFrontMotor.getAppliedOutput());
      SmartDashboard.putNumber("Right Back Motor Power", rightBackMotor.getAppliedOutput());

    }

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {	
    // If debug mode is on, write a line that lets us know what mode we're entering
    if (debug) { System.out.println("Entering disabledInit Phase");}
    // Set LEDs to Red so operator can tell the robot is stopped
    blinkinLED.set(ledBlack);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // If debug mode is on, write a line that lets us know what mode we're entering
    if (debug) { System.out.println("Entering testInit Phase");}
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    // If debug mode is on, write a line that lets us know what mode we're entering
    if (debug) { System.out.println("Entering simulationInit Phase");}
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
