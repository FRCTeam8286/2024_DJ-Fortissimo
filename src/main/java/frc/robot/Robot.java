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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {


  
  // Create PWMSparkMax for controlling Blinkin LED
  private PWMSparkMax blinkinLED = new PWMSparkMax(Constants.blinkinPWMChannel);

  // Define Controller Objects, we'll be associating these with controllers later
  private XboxController xboxMovementController;
  private XboxController xboxInteractionController;

  // Create motor objects
  private DriveTrain DriveTrain;
  private InteractionSystem interactionSystem;

  // Import AHRS
  AHRS ahrs;

  // Call Simulation system
  private Simulation simulation;

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
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // If debug mode is on, write a line that lets us know what mode we're entering
    if (Constants.debug) { System.out.println("Entering robotInit Phase");}
    
    DriveTrain = new DriveTrain();
    interactionSystem = new InteractionSystem();
    simulation = new Simulation(DriveTrain);

    // Have LEDs blink in farwell school colors pattern
    blinkinLED.set(Constants.ledPattern);

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
      // Attempt to initialize the AHRS (Attitude and Heading Reference System) device using the MXP SPI port.
      // AHRS is used for obtaining the robot's heading and orientation.
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      // In case of any exceptions during AHRS initialization (e.g., device not found or communication failure),
      // report the error to the Driver Station. This helps in diagnosing issues with the navX MXP sensor connectivity.
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }

    // Set Top speed to .5
    movementSpeed = Constants.defaultMovementSpeed;

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
    if (Constants.debug) { System.out.println("Entering autonomousInit Phase");}
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  // If debug mode is on, write a line that lets us know what mode we're entering
  if (Constants.debug) { System.out.println("Entering teleopInit Phase");}

	// Set LEDs to Blue so operator can tell the robot is busy initializing
	blinkinLED.set(Constants.ledBlue);
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
	blinkinLED.set(Constants.ledRed);
    
	// Define Controller Inputs

    /**
     * Button Mapping:
     * - XboxMovementController (Port 0):
     *   - Left Y Axis: Drive forward/reverse
     *   - Left X Axis: Strafe left/right
     *   - Start Button: Calibrate Ahrs
     *   - Y Button: Toggle Field Centric
     *   - Right Bumper: Increase movement speed
     *   - Left Bumper: Decrease movement speed
     *   - Right X Axis: Rotate (when in one-controller mode)
     *   - X Button: Run Intake for 5 seconds (when in one-controller mode)
     *   - A Button: Run Shooter for 5 seconds (when in one-controller mode)
     *
     * - XboxInteractionController (Port 1):
     *   - Left X Axis: Rotate (when in two-controller mode)
     *   - Left Trigger: Run Intake
     *   - Right Trigger: Run Shooter
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
    if (Math.abs(xboxMovementController.getLeftY()) > Constants.yDeadZone) {
      yAxisValue = filterY.calculate(-xboxMovementController.getLeftY() * movementSpeed * Constants.yModifier); // Remember, Y stick value is reversed
    }

    if (Math.abs(xboxMovementController.getLeftX()) > Constants.xDeadZone) {
      xAxisValue = filterX.calculate(xboxMovementController.getLeftX() * movementSpeed * Constants.xModifier); 
    }    

    if (twoControllerMode == true && Math.abs(xboxInteractionController.getLeftX()) > Constants.zDeadZone) // zAxis changes based on if we have two controllers (operators) or not
      {zAxisValue = filterZ.calculate(xboxInteractionController.getLeftX() * Constants.zModifier);
    } else if (Math.abs(xboxMovementController.getRightX()) > Constants.zDeadZone){
      zAxisValue = filterZ.calculate(xboxMovementController.getRightX() * Constants.zModifier);
    } 
    
    // Reset the Ahrs when the "Start" button is pressed, and set the LED to blue so the operators know it's busy
    // TODO: Make LED Stay Blue for a second so the know's it happened 
    if (xboxMovementController.getStartButtonPressed()) {
      blinkinLED.set(Constants.ledBlue);
      
      if (Constants.debug) {
        System.out.println("Calibrating Ahrs");
      }
      ahrs.reset();
  
    }

    // If Y is pressed, flip between field centric and robot centric controls
    if (xboxMovementController.getYButtonPressed()) {
      if (Constants.debug) {
        System.out.println("flipping between field centric and robot centric");
      }
      fieldCentricControl = !fieldCentricControl;
    }

    if (twoControllerMode == false) {
      // If X is pressed while in single controller mode, run intake for 5 seconds
      if (xboxMovementController.getXButtonPressed()) {
        if (Constants.debug) {
          System.out.println("Start Intake");
        }
        interactionSystem.timedIntake(0.2, 1);
      }
      // If A is pressed while in single controller mode, run Shooter for 5 seconds
      if (xboxMovementController.getAButtonPressed()) {
        if (Constants.debug) {
          System.out.println("Start Shooter");
        }
        interactionSystem.timedShooter(0.2, 1);
      }
    } else {

      // If Right Trigger is pressed on the interaction controller, run the shooter
      if (xboxInteractionController.getRightTriggerAxis() > 0.5) {
        if (Constants.debug) {
          System.out.println("Start Shooter");
        }
        interactionSystem.runShooter(Constants.shooterSpeed); // Adjust Constants.shooterSpeed to your desired speed
      } else {
        interactionSystem.stopShooter();
      }

      // If Left Trigger is pressed on the interaction controller, run the intake
      if (xboxInteractionController.getLeftTriggerAxis() > 0.5) {
        if (Constants.debug) {
          System.out.println("Start Intake");
        }
        interactionSystem.runIntake(Constants.intakeSpeed); // Adjust Constants.intakeSpeed to your desired speed
      } else {
        interactionSystem.stopIntake();
      }
    }

    // If debug mode is on, provide diagnostic information to the smart dashboard
    if (Constants.debug) {

      // Output X, Y, and Z values to Smart Dashboard for Troubleshooting
      SmartDashboard.putNumber("Current X Value", xAxisValue);
      SmartDashboard.putNumber("Current Y Value", yAxisValue);
      SmartDashboard.putNumber("Current Z Value", zAxisValue);

      // Output Ahrs value to Smart Dashboard for troubleshooting
      SmartDashboard.putNumber("Current ahrs Rotation", ahrs.getRotation2d().getDegrees());

    }

    DriveTrain.drive(fieldCentricControl, yAxisValue, xAxisValue, zAxisValue, ahrs, Constants.debug);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {	
    // If debug mode is on, write a line that lets us know what mode we're entering
    if (Constants.debug) { System.out.println("Entering disabledInit Phase");}
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {if (Constants.debug) { System.out.println("Entering testInit Phase");}}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    // If debug mode is on, write a line that lets us know what mode we're entering
    if (Constants.debug) { System.out.println("Entering simulationInit Phase");}
    simulation.init();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    simulation.periodic();
  }
}
