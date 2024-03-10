// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // Setup Debug Mode
  public static final boolean debug  = true;

  /**
  * CAN Bus Mapping:
  * - Front Left Motor: CAN ID 10
  * - Back Left Motor: CAN ID 11
  * - Front Right Motor: CAN ID 12
  * - Back Right Motor: CAN ID 13
  * - Top Intake Roller Motor: CAN ID 20
  * - Bottom Intake Roller Motor: CAN ID 21
  * - Shooter Roller Motor: CAN ID 30
  * - Shooter Pivot Arm Motor: CAN ID 31
  * - Left Climber Motor: CAN ID 40
  * - Right Climber Motor: CAN ID 41
  */

  // CAN ID values for devices attached to CAN bus
  private static final int leftFrontMotorID = 10;
  private static final int leftBackMotorID = 11;
  private static final int rightFrontMotorID = 12;
  private static final int rightBackMotorID = 13;
  private static final int leftShooterRollerID = 20;
  private static final int rightShooterRollerID = 21;
  private static final int intakeArmID = 30;
  private static final int intakeRollerID = 31;
  private static final int leftClimberMotorID = 40;
  private static final int rightClimberMotorID = 41;

  /**
  * PWM Mapping:
  * - Blinkin LED Controller: PWM 0
  * - Intake Hex Encoder (Through Bore): PWM 1
  */

  // PWM Channels
  private static final int blinkinPWMChannel = 0;
  private static final int intakeHexEncoderPWMChannel = 1;

  // DIO Channels
  private static final int gamePieceDetectionSwitchDIOChannel = 0;

  // Movement Modifiers
  private static final double yModifier = 0.7;
  private static final double xModifier = 0.7;
  private static final double zModifier = 0.5;

  // Deadzones
  private static final double yDeadZone = 0.2;
  private static final double xDeadZone = 0.2;
  private static final double zDeadZone = 0.2;

  // Movement Speed Variable
  private static final double defaultMovementSpeed = 0.5;

  // Shooter Speed Variable
  private static final double shooterSpeed = 0.25;

  // Intake Speed Variable
  private static final double intakeSpeed = 0.25;

  // Climber Speeds
  private static final double climberSpeed = 0.20;

  // Define the position tolerance for the arm movement
  private static final double intakePositionTolerance = 0.05;

  // Define Controller Objects, we'll be associating these with controllers later
  private XboxController xboxMovementController;
  private XboxController xboxInteractionController;

  // AHRS Variable
  private AHRS navx;

  // Limit Switch Variable
  private DigitalInput gamePieceDetectionSwitch;

  // Create objects and variables related to UI choices 
  private SendableChooser<Boolean> controlModeChooser = new SendableChooser<>();

  // These boolean variables are used to determine control options
  private boolean fieldCentricControl;

  // Movement Speed Variable
  private double movementSpeed;

  // isGamePieceLoaded Variable
  private boolean isGamePieceLoaded;  
  
  // Variables related to intake pivoting arm
  private PIDController pidController;
  private CANSparkMax intakeArm;
  private boolean isMovingArm;
  private double targetPosition;   


  // Creates SlewRateLimiter objects for each axis that limits the rate of change. This value is max change per second. For most imports, the range here is  -1 to 1 
  SlewRateLimiter filterX = new SlewRateLimiter(1); 
  SlewRateLimiter filterY = new SlewRateLimiter(1);
  SlewRateLimiter filterZ = new SlewRateLimiter(1);

  // Create Duty Cycle encoder object for the through bore enocder
  private DutyCycleEncoder intakeHexEncoder;

    /**The robot can be in a few states, with corresponding LED colors
   * 
   * These are default states that may get overrided:
   * 0 - Default Mode = Flashing School Colors
   * 1 - No Game Piece Loaded = Solid Red
   * 2 - Game Piece Loaded = Solid Green
   * 
   * These states will override the default states, and may persist:
   * 3 - Attempting Generic Operation = Solid Blue
   * 4 - Attempting to pick up game piece = Solid Yellow
   * 5 - Attempting to Shoot Game Peice = Solid Orange
  */

  // Setup LED State Codes
  private static final int defaultColor = 0;
  private static final int redColor = 1;
  private static final int greenColor = 2;
  private static final int blueColor = 3;
  private static final int yellowColor = 4;
  private static final int orangeColor = 5;

  // Setup LED PWM Outputs
  public static final double ledRed = 0.61;
  public static final double ledOrange = 0.65;
  public static final double ledYellow = 0.69;
  public static final double ledGreen = 0.77;
  public static final double ledBlue = 0.87;
  public static final double ledBlack = 0.99;
  public static final double ledPattern = 0.49;

  private PWMSparkMax blinkinLED = new PWMSparkMax(Robot.blinkinPWMChannel);

  private int currentColor = defaultColor;
  private int nonOverrideState = defaultColor; // Variable to store the current non-override state

  private Timer overrideTimer = new Timer(); // Timer for handling override states

  private int currentState;
  private int currentNonOverrideState;  

  private CANSparkMax leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor; // Drive Motors
  private CANSparkMax leftShooterRoller, rightShooterRoller, intakeRoller, leftClimber, rightClimber; // Interaction Motors
  private boolean isIntakeRunning = false;
  private double intakeStartTime = 0;
  private double intakeDuration = 0;
  private boolean isShooterRunning = false;
  private double shooterStartTime = 0;
  private double shooterDuration = 0;

  private void setLEDColor() {
    switch (currentColor) {
        case redColor:
            blinkinLED.set(ledRed);
            break;
        case greenColor:
            blinkinLED.set(ledGreen);
            break;
        case blueColor:
            blinkinLED.set(ledBlue);
            break;
        case yellowColor:
            blinkinLED.set(ledYellow);
            break;
        case orangeColor:
            blinkinLED.set(ledOrange);
            break;
        default:
            blinkinLED.set(ledPattern);
            break;
    }
  }

  // Method to handle state transitions and overrides
  private void setState(int desiredState) {
    currentState = desiredState;
    switch (desiredState) {
        case 1:
            currentNonOverrideState = desiredState;
            nonOverrideState = redColor;
            break;
        case 2:
            currentNonOverrideState = desiredState;
            nonOverrideState = greenColor;
            break;
        case 3:
            overrideState(blueColor);
            break;
        case 4:
            overrideState(yellowColor);
            break;
        case 5:
            overrideState(orangeColor);
            break;
        default:
            // Handle other states or default behavior
            break;
    }
  }

  private void overrideState(int color) {
    // Override current state with a specific color for a duration
    currentColor = color;
    overrideTimer.reset();
    overrideTimer.start();
  }

  private void clearOverrideState() {
    currentState = currentNonOverrideState;
    currentColor = defaultColor; // Reset LED state to default color
    overrideTimer.stop(); // Stop the override timer
  }

    /**
   * Controls interaction mechanisms like intakes and shooters, offering methods
   * for operation and management of these systems within the 
   */
  

  private void InteractionSystemInit() {
      // Initializes motors for intake and shooter systems as brushless motors
      leftShooterRoller = new CANSparkMax(leftShooterRollerID, MotorType.kBrushless);
      rightShooterRoller = new CANSparkMax(rightShooterRollerID, MotorType.kBrushless);
      intakeRoller = new CANSparkMax(intakeRollerID, MotorType.kBrushless);
      leftClimber = new CANSparkMax(leftClimberMotorID, MotorType.kBrushless);
      rightClimber = new CANSparkMax(rightClimberMotorID, MotorType.kBrushless);

      // Configure initial motor settings (e.g., inversion)      
      leftShooterRoller.setInverted(true); 
      rightShooterRoller.setInverted(false);
      intakeRoller.setInverted(false);
  }

  private void runIntake(double speed) {
      // Sets intake motor speeds; positive values intake, negative values expel
      setState(4);
      intakeRoller.set(speed);
      System.out.println(speed);
      if (debug) {

          // output value to smart dashboard
          SmartDashboard.putNumber("Current Intake Roller Motor Value", intakeRoller.getAppliedOutput());
      }
  }

  private void stopIntake() {
      // Stops the intake motors
      intakeRoller.set(0);
      if (debug) {

          // output value to smart dashboard
          SmartDashboard.putNumber("Current Intake Roller Motor Value", intakeRoller.getAppliedOutput());
      }
      clearOverrideState();
  }

  private void timedIntake(double speed, double duration) {
      // Starts intake motors and schedules it to stop after a duration
      this.runIntake(speed); // Start the intake
      isIntakeRunning = true; // Flag to track intake state
      intakeStartTime = Timer.getFPGATimestamp(); // Record start time for duration tracking
      intakeDuration = duration; // Set how long the intake should run
  }

  private void runShooter(double speed) {
        // Sets shooter motor speeds; positive for shooting, negative could reverse feed
      setState(5);
      leftShooterRoller.set(speed);
      rightShooterRoller.set(speed);
      intakeRoller.set(-speed);
      if (debug) {

          // output value to smart dashboard
          SmartDashboard.putNumber("Current Left Shooter Motor Value", leftShooterRoller.getAppliedOutput());
          SmartDashboard.putNumber("Current right Shooter Motor Value", rightShooterRoller.getAppliedOutput());
      }
  }

  private void stopShooter() {
      // Stops the shooter motors
      clearOverrideState();
      leftShooterRoller.set(0);
      rightShooterRoller.set(0);
      intakeRoller.set(0);
      if (debug) {

          // output value to smart dashboard
          SmartDashboard.putNumber("Current Left Shooter Motor Value", leftShooterRoller.getAppliedOutput());
          SmartDashboard.putNumber("Current right Shooter Motor Value", rightShooterRoller.getAppliedOutput());
      }
  }

  private void RaiseClimbers(){
      leftClimber.set(climberSpeed);
      rightClimber.set(climberSpeed);
      if (debug) {

          // output value to smart dashboard
          SmartDashboard.putNumber("Current Left Climber Motor Value", leftClimber.getAppliedOutput());
          SmartDashboard.putNumber("Current Right Climber Motor Value", rightClimber.getAppliedOutput());
      }
  }

  private void LowerClimbers(){
      leftClimber.set(-climberSpeed);
      rightClimber.set(-climberSpeed);
      if (debug) {

          // output value to smart dashboard
          SmartDashboard.putNumber("Current Left Climber Motor Value", leftClimber.getAppliedOutput());
          SmartDashboard.putNumber("Current Right Climber Motor Value", rightClimber.getAppliedOutput());
      }
  }

  private void StopClimbers(){
      leftClimber.set(0);
      rightClimber.set(0);
      if (debug) {

          // output value to smart dashboard
          SmartDashboard.putNumber("Current Left Climber Motor Value", leftClimber.getAppliedOutput());
          SmartDashboard.putNumber("Current Right Climber Motor Value", rightClimber.getAppliedOutput());
      }
  }

  private void timedShooter(double speed, double duration) {
      // Starts shooter motors and schedules it to stop after a set duration
      this.runShooter(speed); // Start the intake
      isShooterRunning = true;
      shooterStartTime = Timer.getFPGATimestamp(); // Current time
      shooterDuration = duration;
  }

    /**
   * Manages the robot's drive system, including motor initialization and driving
   * logic. Supports both field-centric and robot-centric control modes, handling
   * input processing and motor speed calculations to facilitate smooth and
   * responsive movement.
   */

  private void DriveTrainInit() {
      // Initialize each motor with its respective ID from Constants and set them as brushless
      leftFrontMotor = new CANSparkMax(Robot.leftFrontMotorID, MotorType.kBrushless);
      leftBackMotor = new CANSparkMax(Robot.leftBackMotorID, MotorType.kBrushless);
      rightFrontMotor = new CANSparkMax(Robot.rightFrontMotorID, MotorType.kBrushless);
      rightBackMotor = new CANSparkMax(Robot.rightBackMotorID, MotorType.kBrushless);
      leftFrontMotor.setInverted(false);
      leftBackMotor.setInverted(false);
      rightFrontMotor.setInverted(true);
      rightBackMotor.setInverted(true);
  }

  /**
  * Controls the robot's movement based on joystick input and control mode.
  * @param fieldCentricControl Determines if movement is relative to the field or robot.
  * @param yAxisValue Forward/reverse input.
  * @param xAxisValue Left/right input.
  * @param zAxisValue Rotation input.
  * @param ahrs The robot's gyroscope sensor for orientation.
  * @param debug Enables diagnostic output to SmartDashboard.
  */
  private void DrivePerodic(boolean fieldCentricControl,double yAxisValue, double xAxisValue, double zAxisValue, AHRS ahrs, boolean debug) {
      
      if (fieldCentricControl){
          // Drive System Code from https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html

          // Calculate movement based on robot orientation for field-centric control
          double botHeading = ahrs.getRotation2d().getRadians();
          double rotX = xAxisValue * Math.cos(-botHeading) - yAxisValue * Math.sin(-botHeading);
          double rotY = xAxisValue * Math.sin(-botHeading) + yAxisValue * Math.cos(-botHeading);
          double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(zAxisValue), 1);
          leftFrontMotor.set((rotY + rotX + zAxisValue) / denominator);
          leftBackMotor.set((rotY - rotX + zAxisValue) / denominator);
          rightFrontMotor.set((rotY - rotX - zAxisValue) / denominator);
          rightBackMotor.set((rotY + rotX - zAxisValue)  / denominator);

      } else{
          // Directly apply input for robot-centric control
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

    /**
  * Initializes and updates the robot's component simulations for development and testing.
    */
    // Constructor
    private void IntakeArmInit() {
      intakeArm = new CANSparkMax(intakeArmID, MotorType.kBrushless);        
      intakeArm.setInverted(false);
      isMovingArm = false;
      intakeHexEncoder = new DutyCycleEncoder(intakeHexEncoderPWMChannel);
  }

  private void MoveIntakeArmUp(double speed) {
    intakeArm.set(speed);
    if (debug) {

        // Output Motor Values to Smart Dashboard for troubleshooting
        SmartDashboard.putNumber("Intake Arm Motor", intakeArm.getAppliedOutput());
        SmartDashboard.putNumber("Intake Arm Hex Encoder Position", intakeHexEncoder.get());
        SmartDashboard.putNumber("Intake Arm Hex Encoder Absolate Position", intakeHexEncoder.getAbsolutePosition());
    }
  }

  private void MoveIntakeArmDown(double speed) {
    intakeArm.set(-speed);
    if (debug) {

        // Output Motor Values to Smart Dashboard for troubleshooting
        SmartDashboard.putNumber("Intake Arm Motor", intakeArm.getAppliedOutput());
        SmartDashboard.putNumber("Intake Arm Hex Encoder Position", intakeHexEncoder.get());
        SmartDashboard.putNumber("Intake Arm Hex Encoder Absolute Position", intakeHexEncoder.getAbsolutePosition());
    }
  }

  private void GamePieceDetectionPeriodic() {
      isGamePieceLoaded = gamePieceDetectionSwitch.get();  // Pulls the value returned fromt he Game Piece Detection Switch and sets it to isGamePieceLoaded variable
  }

  private void ButtonControllsPeriodic() {

    // Reset the Ahrs when the "Start" button is pressed, and set the LED to blue so the operators know it's busy
    if (xboxMovementController.getStartButtonPressed()) {
      
      if (debug) {
        System.out.println("Resetting navx");
      }
      navx.reset();

    }

    // If Right Trigger is pressed on the interaction controller, run the shooter
    if (xboxInteractionController.getRightBumperPressed()) {
      if (debug) {
        System.out.println("Start Shooter");
      }
      runShooter(shooterSpeed); // Adjust shooterSpeed to your desired speed
    } else if (xboxInteractionController.getRightBumperReleased()) {
      stopShooter();
    }

    // If Left Trigger is pressed on the interaction controller, run the intake
    if (xboxInteractionController.getLeftBumperPressed()) {
      if (debug) {
        System.out.println("Start Intake");
      }
      runIntake(intakeSpeed); // Adjust intakeSpeed to your desired speed
    } else if (xboxInteractionController.getLeftBumperReleased()) {
      stopIntake();
    }

    // Intake Arm Position
    if (xboxInteractionController.getAButtonPressed()) {
      MoveIntakeArmUp(defaultMovementSpeed);
    } else if (xboxInteractionController.getBButtonPressed()) {
      MoveIntakeArmDown(defaultMovementSpeed);
    } 

    // Raising and Lowering Climber 
    if (xboxInteractionController.getYButtonPressed()) {
      RaiseClimbers();
    } else if (xboxInteractionController.getXButtonPressed()){
      LowerClimbers();
    } else if (xboxInteractionController.getYButtonReleased()){
      StopClimbers();
    } else if (xboxInteractionController.getXButtonReleased()){
      StopClimbers();
    }

  }

  private void InteractionPeriodic() {    

    // Periodically called to check and update the state of intake and shooter based on timers
    double currentTime = Timer.getFPGATimestamp();
    // Checks if intake has exceeded its run duration, then stops it
    if (isIntakeRunning && ((currentTime - intakeStartTime) >= intakeDuration)) {
        this.stopIntake();
        isIntakeRunning = false; // Reset intake running flag
    }
    // Checks if shooter has exceeded its run duration, then stops it
    if (isShooterRunning && ((currentTime - shooterStartTime) >= shooterDuration)) {
        this.stopShooter();
        isShooterRunning = false; // Reset shooter running flag
    }

  }
  
  private void IntakeArmPeriodic() {
    if (isMovingArm) {
      // Calculate the control output using the PID controller
      double output = pidController.calculate(intakeHexEncoder.get());

      // Check if the arm is at or near the target position
      if (Math.abs(intakeHexEncoder.get() - targetPosition) < Robot.intakePositionTolerance) {
        intakeArm.set(0.0); // Stop the motor movement
        isMovingArm = false; // Reset the flag
        if (Robot.debug) {
          // output value to smart dashboard
          SmartDashboard.putNumber("Current Intake Arm Motor Value", 0);
        }
      } else {
        pidController.setSetpoint(targetPosition);
        intakeArm.set(output);
        if (Robot.debug) {

          // output value to smart dashboard
          SmartDashboard.putNumber("Current Intake Arm Motor Value", output);
        }
      }
    }
  }

  private void RobotStatePeriodic() {

    // Update LED state based on timers and non-override state
    if (overrideTimer.hasElapsed(1.0)) {
        // If override timer has elapsed, revert to non-override state
        currentColor = nonOverrideState;
    }
    setLEDColor(); // Update LED color
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
    // If debug mode is on, write a line that lets us know what mode we're entering
    if (debug) { System.out.println("Entering robotInit Phase");}
    
    // Call on the constructor methods
    DriveTrainInit();
    InteractionSystemInit();
    IntakeArmInit();

    intakeHexEncoder = new DutyCycleEncoder(0);

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
    movementSpeed = defaultMovementSpeed;

    // Setup gamePieceDetectionSwitch
    gamePieceDetectionSwitch = new DigitalInput(gamePieceDetectionSwitchDIOChannel);

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
     *   - Left Bumper: Run Intake
     *   - Right Bumper: Run Shooter
     *   - A Button: Move Intake Arm Up
     *   - B Button: Move intake Arm Back
     *   - Y Button: Raise Climbers
     *   - X Button: Lower Climbers
     */

    if (isGamePieceLoaded == true){
      setState(2);
    } else {setState(1);}

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
    if (Math.abs(xboxMovementController.getLeftY()) > yDeadZone) {
      yAxisValue = filterY.calculate(-xboxMovementController.getLeftY() * movementSpeed * yModifier); // Remember, Y stick value is reversed
    }

    if (Math.abs(xboxMovementController.getLeftX()) > xDeadZone) {
      xAxisValue = filterX.calculate(xboxMovementController.getLeftX() * movementSpeed * xModifier); 
    }    

    if (Math.abs(xboxInteractionController.getLeftX()) > zDeadZone) // zAxis changes based on if we have two controllers (operators) or not
      {zAxisValue = filterZ.calculate(xboxInteractionController.getLeftX() * zModifier);
    }  
    
    // Call the Peridoic Methods
    ButtonControllsPeriodic();
    IntakeArmPeriodic();
    InteractionPeriodic();
    RobotStatePeriodic();
    GamePieceDetectionPeriodic();

    // If debug mode is on, provide diagnostic information to the smart dashboard
    if (debug) {

      // Output X, Y, and Z values to Smart Dashboard for Troubleshooting
      SmartDashboard.putNumber("Current X Value", xAxisValue);
      SmartDashboard.putNumber("Current Y Value", yAxisValue);
      SmartDashboard.putNumber("Current Z Value", zAxisValue);
      SmartDashboard.putNumber("Current getAbsolutePosition", intakeHexEncoder.getAbsolutePosition());
      SmartDashboard.putNumber("Current intakehexget", intakeHexEncoder.get());
      // Output Ahrs value to Smart Dashboard for troubleshooting
      SmartDashboard.putNumber("Current ahrs Rotation", navx.getRotation2d().getDegrees());

    }

    DrivePerodic(fieldCentricControl, yAxisValue, xAxisValue, zAxisValue, navx, debug);
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
    
    // Initialize the simulation environment here
    REVPhysicsSim.getInstance().addSparkMax(leftFrontMotor, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(leftBackMotor, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(rightFrontMotor, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(rightBackMotor, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(leftShooterRoller, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(rightShooterRoller, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(intakeRoller, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(leftClimber, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(rightClimber, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(intakeArm, DCMotor.getNEO(1));    

    // Additional initialization as needed
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    // Update the simulation state here
    REVPhysicsSim.getInstance().run();
    SmartDashboard.putNumber("Simluated Left Front Motor Power", leftFrontMotor.get());
    SmartDashboard.putNumber("Simluated Left Back Motor Power", leftBackMotor.get());
    SmartDashboard.putNumber("Simluated Right Front Motor Power", rightFrontMotor.get());
    SmartDashboard.putNumber("Simluated Right Back Motor Power", rightBackMotor.get());
    SmartDashboard.putNumber("Simluated Left Shooter Roller Motor Power", leftShooterRoller.get());
    SmartDashboard.putNumber("Simluated Left Shooter Roller Motor Power", rightShooterRoller.get());
    SmartDashboard.putNumber("Simluated Intake Roller Motor Power", intakeRoller.get());
    SmartDashboard.putNumber("Simluated Left Climber Motor Power", leftClimber.get());
    SmartDashboard.putNumber("Simluated Right Climber Motor Power", rightClimber.get());
    SmartDashboard.putNumber("Simluated Intake Pivot Motor Power", intakeArm.get());
  }
}
