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
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // Define Controller Objects, we'll be associating these with controllers later
  private XboxController xboxMovementController;
  private XboxController xboxInteractionController;

  // Create motor objects
  private DriveTrain DriveTrain;
  private InteractionSystem interactionSystem;

  // AHRS Variable
  AHRS navx;

  // Call Simulation system
  private Simulation simulation;

  // Create objects and variables related to UI choices 
  private SendableChooser<Boolean> controlModeChooser = new SendableChooser<>();

  // These boolean variables are used to determine control options
  private boolean fieldCentricControl;

  // Movement Speed Variable
  private double movementSpeed;

  // isGamePieceLoaded Variable
  private boolean isGamePieceLoaded;

  // Creates SlewRateLimiter objects for each axis that limits the rate of change. This value is max change per second. For most imports, the range here is  -1 to 1 
  SlewRateLimiter filterX = new SlewRateLimiter(1); 
  SlewRateLimiter filterY = new SlewRateLimiter(1);
  SlewRateLimiter filterZ = new SlewRateLimiter(1);

  // Create statemanager object to help us keep track of the robot's state
  StateManager StateManager;

  // Create an instance of the intakeArm class
  private intakeArm intakeArm;


  // Create Duty Cycle encoder object for the through bore enocder
  private DutyCycleEncoder intakeHexEncoder;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
    // If debug mode is on, write a line that lets us know what mode we're entering
    if (Constants.debug) { System.out.println("Entering robotInit Phase");}
    
    intakeHexEncoder = new DutyCycleEncoder(0);
    intakeArm = new intakeArm();

    StateManager = new StateManager();

    // Pass LEDStateManager object to InteractionSystem constructor
    interactionSystem = new InteractionSystem(StateManager);

    // Drive train and Simulation
    DriveTrain = new DriveTrain();
    simulation = new Simulation(DriveTrain);

    // Initiate Xbox Controllers
    xboxMovementController = new XboxController(0);  // Replace 0 with the port number of your movement Xbox controller
    xboxInteractionController = new XboxController(1);  // Replace 1 with the port number of your interaction Xbox controller

    // Add options to the controlModeChooser
    controlModeChooser.addOption("Field-Centric Control", true);
    controlModeChooser.addOption("Robot-Centric Control", false);

    // Put the chooser on the SmartDashboard
    SmartDashboard.putData("Control Mode Chooser", controlModeChooser);

    try {
      // Attempt to initialize the AHRS (Attitude and Heading Reference System) device using the MXP SPI port.
      // AHRS is used for obtaining the robot's heading and orientation.
      navx = new AHRS(SPI.Port.kMXP);
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

    // If statement to see if our Mode Choser outputs worked, and if not, have some fall back values (Mostly for Simulation Mode)
    if (controlModeChooser.getSelected() != null) {
      
      // Set variables fieldCentricControl and twoControllerMode to options selected on interactive chooser by the operators
      fieldCentricControl = controlModeChooser.getSelected();

    } else {

      // Handle the case where one or both values are null (simulation mode). Also log message because that is probably interesting to see
      System.err.println("Error: Unable to retrieve control mode or controller mode from choosers. Using default values.");
      fieldCentricControl = false;
      
    }

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
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
     *
     * - XboxInteractionController (Port 1):
     *   - Left X Axis: Rotate
     *   - Left Trigger: Run Intake
     *   - Right Trigger: Run Shooter
     *   - A Button: Move intake arm to intake position
     *   - B Button: Move intake arm to amp position
     *   - X Button: Move intake arm to speaker position
     */

    if (isGamePieceLoaded == true){
      StateManager.setState(2);
    } else {StateManager.setState(1);}

    if (xboxMovementController.getRightBumperPressed()){

      // Increase movement speed by 0.25 (up to a maximum of 1.0)
      movementSpeed += 0.25;
      movementSpeed = Math.min(movementSpeed, 1.0);

    } 
    
    if (xboxMovementController.getLeftBumperPressed()) {

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

    if (Math.abs(xboxInteractionController.getLeftX()) > Constants.zDeadZone) // zAxis changes based on if we have two controllers (operators) or not
      {zAxisValue = filterZ.calculate(xboxInteractionController.getLeftX() * Constants.zModifier);
    }
    
    // Reset the Ahrs when the "Start" button is pressed, and set the LED to blue so the operators know it's busy
    // TODO: Make LED Stay Blue for a second so the know's it happened 
    if (xboxMovementController.getStartButtonPressed()) {
      
      if (Constants.debug) {
        System.out.println("Resetting navx");
      }
      navx.reset();

    }

    // If Right Trigger is pressed on the interaction controller, run the shooter
    if (xboxInteractionController.getRightTriggerAxis() > 0.2) {
      if (Constants.debug) {
        System.out.println("Start Shooter");
      }
      interactionSystem.runShooter(Constants.shooterSpeed); // Adjust Constants.shooterSpeed to your desired speed
      // TODO Pull from Limit Switch to see if a peice is still loaded 
    } else {
      interactionSystem.stopShooter();
    }

    // If Left Trigger is pressed on the interaction controller, run the intake
    if (xboxInteractionController.getLeftTriggerAxis() > 0.2) {
      if (Constants.debug) {
        System.out.println("Start Intake");
      }
      interactionSystem.runIntake(Constants.intakeSpeed); // Adjust Constants.intakeSpeed to your desired speed
      // TODO Pull from Limit Switch to see if a peice is still loaded 
    } else {
      interactionSystem.stopIntake();
    }

    // Intake Arm Position
    if (xboxInteractionController.getAButtonPressed()) {
      intakeArm.moveToIntakePosition(intakeHexEncoder);
      if (Constants.debug) {
        System.out.println("moveToIntakePosition");
      }
    } else if (xboxInteractionController.getBButtonPressed()) {
      intakeArm.moveToAmpPosition(intakeHexEncoder);
      if (Constants.debug) {
        System.out.println("moveToAmpPosition");
      }
    } else if (xboxInteractionController.getXButtonPressed()) {
      intakeArm.moveToSpeakerPosition(intakeHexEncoder);
      if (Constants.debug) {
        System.out.println("moveToSpeakerPosition");
      }
    }

    if (xboxInteractionController.getYButtonPressed()) {
      interactionSystem.raiseArms();
    } else if (xboxInteractionController.getStartButtonPressed()){
      interactionSystem.lowerArms();
    } else if (xboxInteractionController.getYButtonReleased()){
      interactionSystem.stopArms();
    } else if (xboxInteractionController.getStartButtonReleased()){
      interactionSystem.stopArms();
    }

    // Call the update method for the intake arm, to keep it moving if needed
    intakeArm.update(intakeHexEncoder);

    // Call the update method for the interaction system, to keep it moving if needed
    interactionSystem.update();

    // If debug mode is on, provide diagnostic information to the smart dashboard
    if (Constants.debug) {

      // Output X, Y, and Z values to Smart Dashboard for Troubleshooting
      SmartDashboard.putNumber("Current X Value", xAxisValue);
      SmartDashboard.putNumber("Current Y Value", yAxisValue);
      SmartDashboard.putNumber("Current Z Value", zAxisValue);

      // Output Ahrs value to Smart Dashboard for troubleshooting
      SmartDashboard.putNumber("Current ahrs Rotation", navx.getRotation2d().getDegrees());

    }

    DriveTrain.drive(fieldCentricControl, yAxisValue, xAxisValue, zAxisValue, navx, Constants.debug);
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
