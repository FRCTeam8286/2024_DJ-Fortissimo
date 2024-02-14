// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;

import com.kauailabs.navx.frc.AHRS;

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
  *   - Intake Roller Motor: CAN ID 20
  */
  private static final int leftFrontDeviceID = 10; 
  private static final int leftBackDeviceID = 11; 
  private static final int rightFrontDeviceID = 12; 
  private static final int rightBackDeviceID = 13; 
  private static final int intakeRollerDeviceID = 20; 
  
  // Assuming Blinkin LED controller is connected to PWM port 0
  private static final int blinkinPWMChannel = 0;
  
  // Create PWMSparkMax for controlling Blinkin LED
  private PWMSparkMax blinkinLED = new PWMSparkMax(blinkinPWMChannel);

  // Define Controller Objects, we'll be associating these with controllers later
  private XboxController xboxMovementController;
  private XboxController xboxInteractionController;

  // Create objects related to drive train
  private CANSparkMax leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, intakeMotor;

  // Import AHRS
  AHRS ahrs;

  // Create objects and variables related to UI choices 
  private SendableChooser<Boolean> controlModeChooser = new SendableChooser<>();
  private SendableChooser<Boolean> controllerModeChooser = new SendableChooser<>();

  // These boolean variables are used to determine control options
  private boolean fieldCentricControl;
  private boolean twoControllerMode;

  // Movement Speed Variable
  private double movementSpeed;

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

    intakeMotor = new CANSparkMax(intakeRollerDeviceID, MotorType.kBrushless);

    // Motor inversions
    
    leftFrontMotor.setInverted(false);
    leftBackMotor.setInverted(false);
    rightFrontMotor.setInverted(true);
    rightBackMotor.setInverted(true);
    intakeMotor.setInverted(false);

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

    try {
      /***********************************************************************
       * navX-MXP: - Communication via RoboRIO MXP (SPI, I2C) and USB. - See
       * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
       * 
       * navX-Micro: - Communication via I2C (RoboRIO MXP or Onboard) and USB. - See
       * http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
       * 
       * VMX-pi: - Communication via USB. - See
       * https://vmx-pi.kauailabs.com/installation/roborio-installation/
       * 
       * Multiple navX-model devices on a single robot are supported.
       ************************************************************************/
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }

    // Set Top speed to .5
    movementSpeed = .5;

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
     *   - Start Button: Calibrate Ahrs
     *   - Y Button: Toggle Field Centric
     *   - Right Bumper: Increase movement speed
     *   - Left Bumper: Decrease movement speed
     *   - X Button: Run Intake for 5 seconds
     *
     * - XboxInteractionController (Port 1):
     *   - Left X Axis: Rotate (when in two-controller mode)
     *   - Left Trugger: Run Intake
     */
    
     // 
     if (xboxMovementController.getRightBumperPressed()){

        // Increase movement speed by 0.25 (up to a maximum of 1.0)
        movementSpeed += 0.25;
        movementSpeed = Math.min(movementSpeed, 1.0);

    } else if (xboxMovementController.getLeftBumperPressed()) {

        // Decrease movement speed by 0.25 (down to a minimum of 0.25)
        movementSpeed -= 0.25;
        movementSpeed = Math.max(movementSpeed, 0.25);

    }

    // Movement variables provided they are above deadzone threshold
    double yAxisValue = 0;
    double xAxisValue = 0;
    double zAxisValue = 0;
    if (Math.abs(xboxMovementController.getLeftY()) > yDeadZone) {
      yAxisValue = filterY.calculate(-xboxMovementController.getLeftY() * movementSpeed * yModifier); // Remember, Y stick value is reversed
    }

    if (Math.abs(xboxMovementController.getLeftX()) > xDeadZone) {
      xAxisValue = filterX.calculate(xboxMovementController.getLeftX() * movementSpeed * xModifier); 
    }    

    if (twoControllerMode == true && Math.abs(xboxInteractionController.getLeftX()) > zDeadZone) // zAxis changes based on if we have two controllers (operators) or not
      {zAxisValue = filterZ.calculate(xboxInteractionController.getLeftX() * zModifier);
    } else if (Math.abs(xboxMovementController.getRightX()) > zDeadZone){
      zAxisValue = filterZ.calculate(xboxMovementController.getRightX() * zModifier);
    } 
    
    // Reset the Ahrs when the "Start" button is pressed, and set the LED to blue so the operators know it's busy
    // TODO: Make LED Stay Blue for a second so the know's it happened 
    if (xboxMovementController.getStartButtonPressed()) {
      blinkinLED.set(ledBlue);
      
      if (debug) {
        System.out.println("Calibrating Ahrs");
      }
      ahrs.reset();
  
    }

    // If Y is pressed, flip between field centric and robot centric controls
    if (xboxMovementController.getYButtonPressed()) {
      if (debug) {
        System.out.println("flipping between field centric and robot centric");
      }
      fieldCentricControl = !fieldCentricControl;
    }

    // If X is pressed while in single controller mode, run intake for 5 seconds
    if (twoControllerMode == false) {
        if (xboxMovementController.getXButtonPressed()) {
            if (debug) {
                System.out.println("Start Intake");
            }
            intakeMotor.set(0.2);
	    Timer.delay(5);
	    intakeMotor.set(0);
        }
    }

    // TODO: Intake Code for Two Controller Mode

    // If debug mode is on, provide diagnostic information to the smart dashboard
    if (debug) {

      // Output X, Y, and Z values to Smart Dashboard for Troubleshooting
      SmartDashboard.putNumber("Current X Value", xAxisValue);
      SmartDashboard.putNumber("Current Y Value", yAxisValue);
      SmartDashboard.putNumber("Current Z Value", zAxisValue);

      // Output Ahrs value to Smart Dashboard for troubleshooting
      SmartDashboard.putNumber("Current ahrs Rotation", ahrs.getRotation2d().getDegrees());

    }

    // Drive System Code from https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
   
    if (fieldCentricControl){

      // Grab current position from ahrs
      double botHeading = ahrs.getRotation2d().getRadians();

      // Rotate the movement direction counter to the bot's rotation
      double rotX = xAxisValue * Math.cos(-botHeading) - yAxisValue * Math.sin(-botHeading);
      double rotY = xAxisValue * Math.sin(-botHeading) + yAxisValue * Math.cos(-botHeading);

      // Denominator is the largest motor power (absolute value) or 1
      // This ensures all the powers maintain the same ratio,
      // but only if at least one is out of the range [-1, 1]
      double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(zAxisValue), 1);
      leftFrontMotor.set((rotY + rotX + zAxisValue) / denominator);
      leftBackMotor.set((rotY - rotX + zAxisValue) / denominator);
      rightFrontMotor.set((rotY - rotX - zAxisValue) / denominator);
      rightBackMotor.set((rotY + rotX - zAxisValue)  / denominator);

    } else{

      // Denominator is the largest motor power (absolute value) or 1
      // This ensures all the powers maintain the same ratio,
      // but only if at least one is out of the range [-1, 1]
      double denominator = Math.max(Math.abs(yAxisValue) + Math.abs(xAxisValue) + Math.abs(zAxisValue), 1);
      leftFrontMotor.set((yAxisValue + xAxisValue + zAxisValue) / denominator);
      leftBackMotor.set((yAxisValue - xAxisValue + zAxisValue) / denominator);
      rightFrontMotor.set((yAxisValue - xAxisValue - zAxisValue) / denominator);
      rightBackMotor.set((yAxisValue + xAxisValue - zAxisValue)  / denominator);

    }

    // if debug mode is on, provide diagnostic data to the smart dashboard
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
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {if (debug) { System.out.println("Entering testInit Phase");}}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    // If debug mode is on, write a line that lets us know what mode we're entering
    if (debug) { System.out.println("Entering simulationInit Phase");}
    REVPhysicsSim.getInstance().addSparkMax(leftFrontMotor, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(leftBackMotor, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(rightFrontMotor, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(rightBackMotor, DCMotor.getNEO(1));
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
      SmartDashboard.putNumber("Simluated Left Front Motor Power", leftFrontMotor.get());
      SmartDashboard.putNumber("Simluated Left Back Motor Power", leftBackMotor.get());
      SmartDashboard.putNumber("Simluated Right Front Motor Power", rightFrontMotor.get());
      SmartDashboard.putNumber("Simluated Right Back Motor Power", rightBackMotor.get());
  }
}
