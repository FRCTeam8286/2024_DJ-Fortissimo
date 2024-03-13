// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d; //Represents a 2D pose containing translational and rotational elements
import edu.wpi.first.math.geometry.Rotation2d; //Rotates the robot in the 2D Space
import edu.wpi.first.math.geometry.Translation2d; //Translates the robot in the 2D Space
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics; //Makes conversion from ChassisVelocity to WheelSpeeds 
import edu.wpi.first.math.kinematics.MecanumDriveOdometry; //Allows the tracking of the robot's position on a field
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions; // Holds each of the four wheel positions in EncoderCounts
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds; // Holds each of the four wheel speeds in RPM
import edu.wpi.first.math.kinematics.proto.ChassisSpeedsProto;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;


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
  private static final int leftFrontMotorCANID = 10;
  private static final int leftBackMotorCANID = 11;
  private static final int rightFrontMotorCANID = 12;
  private static final int rightBackMotorCANID = 13;
  private static final int leftShooterRollerMotorCANID = 20;
  private static final int rightShooterRollerMotorCANID = 21;
  private static final int intakeArmPivitMotorCANID = 30;
  private static final int intakeRollerMotorCANID = 31;
  private static final int leftClimberMotorCANID = 40;
  private static final int rightClimberMotorCANID = 41;

  /**
  * PWM Mapping:
  * - Blinkin LED Controller: PWM 1
  * - Intake Hex Encoder (Through Bore): PWM 2
  */

  // PWM Channels
  private static final int blinkinPWMChannel = 1;
  private static final int intakeHexEncoderPWMChannel = 2;

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
  private static final double shooterSpeed = 0.65;

  // Intake Speed Variable
  private static final double intakeSpeed = 0.35;

  // Climber Speeds
  private static final double climberSpeed = 0.20;

  // Intake Arm Speed
  private static final double intakeArmSpeed = 0.20;

  // Intake Time
  private static final double intakeTime = 5;

  // Shooter Time
  private static final double shooterTime = 5;

  // Intake Arm Move to/from Intake Position and Speaker Position
  private static final double armIntakeSpeakerPositionTime = 2;

  // Intake Arm Move to/from Intake Position and Amp Position
  private static final double armAmpIntakePositionTime = 0.5;

  // Intake Arm Move to/from Speaker Position and Amp Position
  private static final double armAmpSpeakerPositionTime = 0.5;

  // Define Controller Objects, we'll be associating these with controllers later
  private XboxController xboxMovementController;
  private XboxController xboxInteractionController;

  // AHRS Variable
  private AHRS navx;

  // Limit Switch Variable
  private DigitalInput gamePieceDetectionSwitch;

  // Create objects and variables related to UI choices 
  private SendableChooser<Boolean> controlModeChooser = new SendableChooser<>();
  private SendableChooser<String> autonRoutineChooser = new SendableChooser<>();

  private String defaultRoutine;
  private String testRoutine;

  // These boolean variables are used to determine control options
  private boolean fieldCentricControl;

  // Movement Speed Variable
  private double movementSpeed;

  // isGamePieceLoaded Variable
  private boolean isGamePieceLoaded;  
  
  // Variables related to intake pivoting arm
  private CANSparkMax intakeArm;
  private int IntakeArmPosition = 2; // 0 = Intake Position, 1 = Amp Position, 2 = Speaker Position

  // Creates SlewRateLimiter objects for each axis that limits the rate of change. This value is max change per second. For most imports, the range here is  -1 to 1 
  SlewRateLimiter filterX = new SlewRateLimiter(3); 
  SlewRateLimiter filterY = new SlewRateLimiter(3);
  SlewRateLimiter filterZ = new SlewRateLimiter(3);

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

  private int currentColor = redColor;
  private int nonOverrideState = redColor; // Variable to store the current non-override state

  private double overrideTimer;            // Timer for handling override states

  private int currentState;
  private int currentNonOverrideState;  

  private double autonomousStartTime;
  private double autonomousElapsedTime;

  private CANSparkMax leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;                           // Drive Motors
  private RelativeEncoder leftFrontEncoder, leftBackEncoder, rightFrontEncoder, rightBackEncoder;               // Drive Motor Encoders
  private Translation2d leftFrontTranslation, leftBackTranslation, rightFrontTranslation, rightBackTranslation; // Drive Motors Positions
  private CANSparkMax leftShooterRoller, rightShooterRoller, intakeRoller, leftClimber, rightClimber;           // Interaction Motors
  private boolean isIntakeRunning = false;                                                                      // Intake Running Tracker
  private double intakeStartTime = Timer.getFPGATimestamp();                                                    // Start Time tracker
  private double intakeDuration = 0;
  private boolean isShooterRunning = false;
  private double shooterStartTime = 0;
  private double shooterDuration = 0;
  private double intakeArmStartTime = 0;
  private double intakeArmDuration = 0;
  private boolean isIntakeArmRunning = false;
  private boolean intakeArmDirection;
  private double shooterSpinupTime = 1;

  private MecanumDriveKinematics kinematics;
  private MecanumDriveOdometry odometry;
  private RamseteController ramseteController;

  // Locations of the wheels relative to the robot center.
  private static final Translation2d flModuleOffset = new Translation2d(0.4, 0.4);
  private static final Translation2d frModuleOffset = new Translation2d(0.4, -0.4);
  private static final Translation2d blModuleOffset = new Translation2d(-0.4, 0.4);
  private static final Translation2d brModuleOffset = new Translation2d(-0.4, -0.4);

  // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
  // These characterization values MUST be determined either experimentally or theoretically
  // for *your* robot's drive.
  // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
  // values for your robot.
  public static final double ksVolts = 0.22;
  public static final double kvVoltSecondsPerMeter = 1.98;
  public static final double kaVoltSecondsSquaredPerMeter = 0.2;

  // Example value only - as above, this must be tuned for your drive!
  public static final double kPDriveVel = 8.5;
  
   private void SetLEDColor() {
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
  /**
  * Controls the robot's movement based on joystick input and control mode.
  * @param desiredState Intager that reresents current robot state. 1=no piece loaded, 2=piece loaded, 3=informational state, 4=Attempting Pickup, 5=Attempting Shooter
  */
  private void SetState(int desiredState) {
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
    overrideTimer = Timer.getFPGATimestamp();
  }

  private void clearOverrideState() {
    currentState = currentNonOverrideState;
    overrideTimer = 0;
  } 

  private void RobotStatePeriodic() {

    // Update LED state based on timers and non-override state
    if ((Timer.getFPGATimestamp() - overrideTimer) < 1) {
        // If override timer has elapsed, revert to non-override state
        currentColor = nonOverrideState;
    }   

    if (isGamePieceLoaded == true) {
      nonOverrideState = greenColor;
    } else {
      nonOverrideState = redColor;
    }

    SetLEDColor(); // Update LED color

    if (debug) {

        // Output Motor Values to Smart Dashboard for troubleshooting
        SmartDashboard.putNumber("Robot LED State",currentColor);
        SmartDashboard.putNumber("Robot LED PWM", blinkinLED.get());
    }

  }
    /**
   * Manages the robot's drive system, including motor initialization and driving
   * logic. Supports both field-centric and robot-centric control modes, handling
   * input processing and motor speed calculations to facilitate smooth and
   * responsive movement.
   */

    /**
   * Controls interaction mechanisms like intakes and shooters, offering methods
   * for operation and management of these systems within the 
   */
  

  private void InteractionSystemInit() {
    // Initializes motors for intake and shooter systems as brushless motors
    leftShooterRoller = new CANSparkMax(leftShooterRollerMotorCANID, MotorType.kBrushless);
    rightShooterRoller = new CANSparkMax(rightShooterRollerMotorCANID, MotorType.kBrushless);
    intakeRoller = new CANSparkMax(intakeRollerMotorCANID, MotorType.kBrushless);
    leftClimber = new CANSparkMax(leftClimberMotorCANID, MotorType.kBrushless);
    rightClimber = new CANSparkMax(rightClimberMotorCANID, MotorType.kBrushless);

    // Configure initial motor settings (e.g., inversion)      
    leftShooterRoller.setInverted(true); 
    rightShooterRoller.setInverted(false);
    intakeRoller.setInverted(false);
    
    // Set Idle Modes
    /* 
    intakeArm.setIdleMode(IdleMode.kBrake);
    leftClimber.setIdleMode(IdleMode.kBrake);
    rightClimber.setIdleMode(IdleMode.kBrake);
    leftShooterRoller.setIdleMode(IdleMode.kCoast);
    rightShooterRoller.setIdleMode(IdleMode.kCoast);*/
  }

  private void runIntake(double speed) {
    // Sets intake motor speeds; positive values intake, negative values expel
    SetState(4);
    intakeRoller.set(speed);
    if (debug) {

        // output value to smart dashboard
        SmartDashboard.putNumber("Current Intake Roller Motor Value", intakeRoller.getAppliedOutput());
    }
  }

  private void stopIntake() {
      // Stops the intake motors
      intakeRoller.set(0);
      clearOverrideState();         
      isIntakeRunning = false;   
      if (debug) {

          // output value to smart dashboard
          SmartDashboard.putNumber("Current Intake Roller Motor Value", intakeRoller.getAppliedOutput());
      }
  }

  private void timedIntake(double duration) {
    intakeStartTime = Timer.getFPGATimestamp(); // Record start time for duration tracking
    isIntakeRunning = true; // Flag to track intake state
    intakeDuration = duration;
  }

  private void IntakeRollerPeriodic(){
    if (isShooterRunning == false) {
      if (isGamePieceLoaded == true){
        intakeStartTime = 0;
      }
      if ((Timer.getFPGATimestamp() - intakeStartTime) < intakeDuration) {        
        // Starts intake motors and schedules it to stop after a duration
        runIntake(intakeSpeed); // Start the intake
        isIntakeRunning = true; // Flag to track intake state
      } else {
        stopIntake();
      }    
    }
  }

  private void runShooter(double speed, boolean spinup) {
    // Sets shooter motor speeds; positive for shooting, negative could reverse feed
    SetState(5);      
    leftShooterRoller.set(speed);
    rightShooterRoller.set(speed);
    if (spinup == false) {
      runIntake(-speed);
    }
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
    isShooterRunning = false;
    if (debug) {
      // output value to smart dashboard
      SmartDashboard.putNumber("Current Left Shooter Motor Value", leftShooterRoller.getAppliedOutput());
      SmartDashboard.putNumber("Current right Shooter Motor Value", rightShooterRoller.getAppliedOutput());
    }
  }

  private void TimedShooter(double duration) {
    shooterStartTime = Timer.getFPGATimestamp(); // Record start time for duration tracking
    isShooterRunning = true; // Flag to track intake state
    shooterDuration = duration;
  }

  private void ShooterRollerPeriodic(){
    if ((Timer.getFPGATimestamp() - shooterStartTime) < shooterDuration) {  
      if ((Timer.getFPGATimestamp() - shooterStartTime) > shooterSpinupTime){     //  We want to give the shooter time to spin up 
        // Starts intake motors and schedules it to stop after a duration
        runShooter(shooterSpeed, false); // Start the intake
        isShooterRunning = true; // Flag to track intake state
      } else {
        runShooter(shooterSpeed, true);
      }
    } else {
      stopShooter();
      isShooterRunning = false;
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

  /**
  * Initializes and updates the robot's component simulations for development and testing.
  */
  // Constructor
  private void IntakeArmInit() {
    intakeArm = new CANSparkMax(intakeArmPivitMotorCANID, MotorType.kBrushless);        
    intakeArm.setInverted(false);
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
  private void StopIntakeArm() {
    intakeArm.set(0);
    if (debug) {
      // Output Motor Values to Smart Dashboard for troubleshooting
      SmartDashboard.putNumber("Intake Arm Motor", intakeArm.getAppliedOutput());
      SmartDashboard.putNumber("Intake Arm Hex Encoder Position", intakeHexEncoder.get());
      SmartDashboard.putNumber("Intake Arm Hex Encoder Absolute Position", intakeHexEncoder.getAbsolutePosition());
    }
  }

  private void IntakeArmSpeakerPosition(){
    /*
     * Case 0 = Intake arm is at intake position
     * Case 1 = Intake arm is at amp position
     * Case 2 = Intake arm is in speaker position
     */
    switch(IntakeArmPosition) {
      case 0:
        timedIntakeArmUp(armIntakeSpeakerPositionTime);
        break;
      case 1:
        timedIntakeArmUp(armAmpSpeakerPositionTime);
        break;
      default:
        timedIntakeArmUp(armIntakeSpeakerPositionTime);
        break;
    }
    IntakeArmPosition = 2;
  }

  private void IntakeArmAmpPosition(){

     /*
     * Case 0 = Intake arm is at intake position
     * Case 1 = Intake arm is at amp position
     * Case 2 = Intake arm is in speaker position
     */
    switch(IntakeArmPosition) {
      case 0:
        timedIntakeArmUp(armAmpIntakePositionTime);
        break;
      case 2:
        timedIntakeArmDown(armAmpSpeakerPositionTime);
        break;
      default:
        break;        
    }
    IntakeArmPosition = 1;
  }

  private void IntakeArmIntakePosition(){
     /*
     * Case 0 = Intake arm is at intake position
     * Case 1 = Intake arm is at amp position
     * Case 2 = Intake arm is in speaker position
     */
    switch(IntakeArmPosition) {
      case 1:
        timedIntakeArmDown(armAmpIntakePositionTime);
        break;
      case 2:
        timedIntakeArmDown(armIntakeSpeakerPositionTime);
        break;
      default:
        timedIntakeArmDown(armIntakeSpeakerPositionTime);
        break;        
    }
    IntakeArmPosition = 0;
  }

  private void timedIntakeArmUp(double duration) {
    intakeArmStartTime = Timer.getFPGATimestamp(); // Record start time for duration tracking
    isIntakeArmRunning = true; // Flag to track intake state
    intakeArmDuration = duration;
    intakeArmDirection = true;
  }

  private void timedIntakeArmDown(double duration) {
    intakeArmStartTime = Timer.getFPGATimestamp(); // Record start time for duration tracking
    isIntakeArmRunning = true; // Flag to track intake state
    intakeArmDuration = duration;
    intakeArmDirection = false;
  }

  private void IntakeArmPeriodic(){
    if ((Timer.getFPGATimestamp() - intakeArmStartTime) < intakeArmDuration) {        
      // Starts intake motors and schedules it to stop after a duration
      if (intakeArmDirection==true){
        MoveIntakeArmUp(intakeArmSpeed); // Start the intake
        isIntakeArmRunning = true; // Flag to track intake state
      } else {
        MoveIntakeArmDown(intakeArmSpeed); // Start the intake
        isIntakeArmRunning = true; // Flag to track intake state
      }
    } else {
      StopIntakeArm();
      isIntakeRunning = false;
    }
    if (debug) {

      // Output Motor Values to Smart Dashboard for troubleshooting
      String intakearmPositionName;
      switch (IntakeArmPosition) {
        case 1:
          intakearmPositionName = "Amp";
          break;
        case 2:
          intakearmPositionName = "Speaker";
          break;
        default:
          intakearmPositionName = "Intake";
          break;
      }
      SmartDashboard.putString("Intake Arm Position", intakearmPositionName);
    }
  }

  private void GamePieceDetectionPeriodic() {
    isGamePieceLoaded = !gamePieceDetectionSwitch.get();  // Pulls the value returned fromt he Game Piece Detection Switch and sets it to isGamePieceLoaded variable
    if (debug) {
      SmartDashboard.putBoolean("Gamepiece Limit Switch", isGamePieceLoaded);
    }
  }

  private void ButtonControllsPeriodic() {    
    // Define Controller Inputs

    /**
     * Button Mapping:
     * - XboxMovementController (Port 0):
     *   - Left Y Axis: Drive forward/reverse
     *   - Left X Axis: Strafe left/right
     *   - Right X Axis: Rotate
     *   - Start Button: Calibrate Ahrs
     *   - Right Bumper: Increase movement speed
     *   - Left Bumper: Decrease movement speed
     *
     * - XboxInteractionController (Port 1):
     *   - Left Trig: Run Intake
     *   - Right Trig: Run Shooter
     *   - A Button: Intake Position Speaker
     *   - B Button: Intake Position Intake
     *   - Start BUtton: Intake Position Amp
     *   - Y Button: Raise Climbers
     *   - X Button: Lower Climbers
     */

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
    // Reset the Ahrs when the "Start" button is pressed, and set the LED to blue so the operators know it's busy
    if (xboxMovementController.getStartButtonPressed()) {      
      if (debug) {
        System.out.println("Resetting navx");
      }
      navx.reset();
    }
    // If Right Trigger is pressed on the interaction controller, run the shooter
    if (xboxInteractionController.getRightTriggerAxis() > 0.2) {
      if (debug) {
        System.out.println("Start Shooter");
      }
      TimedShooter(3); // Adjust shooterSpeed to your desired speed
    }
    // If Left Trigger is pressed on the interaction controller, run the intake
    if (xboxInteractionController.getLeftTriggerAxis() > 0.2) {
      if (debug) {
        System.out.println("Start Intake");
      }
      timedIntake(5); // Adjust intakeSpeed to your desired speed
    }
    // Intake Arm Position
    if (xboxInteractionController.getAButtonPressed()) {
      IntakeArmSpeakerPosition();
    } else if (xboxInteractionController.getBButtonPressed()) {
      IntakeArmIntakePosition();
    } else if (xboxInteractionController.getStartButtonPressed()) {
      IntakeArmAmpPosition();
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
    ShooterRollerPeriodic();
    IntakeRollerPeriodic();
    IntakeArmPeriodic();
    GamePieceDetectionPeriodic();
    if (debug) {
        // Output Motor Values to Smart Dashboard for troubleshooting
        SmartDashboard.putNumber("Current Timer Value",Timer.getFPGATimestamp());
    }
  }

  private void DriveTrainInit() {
      // Initialize each motor with its respective ID from Constants and set them as brushless
      leftFrontMotor = new CANSparkMax(Robot.leftFrontMotorCANID, MotorType.kBrushless);
      leftBackMotor = new CANSparkMax(Robot.leftBackMotorCANID, MotorType.kBrushless);
      rightFrontMotor = new CANSparkMax(Robot.rightFrontMotorCANID, MotorType.kBrushless);
      rightBackMotor = new CANSparkMax(Robot.rightBackMotorCANID, MotorType.kBrushless);
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
  private void DrivePerodic(boolean fieldCentricControl,double yAxisValue, double xAxisValue, double zAxisValue, AHRS ahrs) {      
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

  private void defaultAutonomousTimedRoutine() {
    /*
     * Run Shooter and wait 3 seconds
     * Move to intake position
     * Run intake and move Backwards for 2 seconds and stop if game piece is loaded
     * Move forwards for 2 seconds
     * Run shooter
     */
    if (autonomousElapsedTime < 0.1) {                                                              // Until 3 Seconds
      TimedShooter(3);                                                                   // Run Shooter
    } else if (autonomousElapsedTime > 4 && autonomousElapsedTime < 4.1) {                                                       // at 3 Seconds
      IntakeArmIntakePosition();                                                                  // Move to Intake Arm Position
    } else if (autonomousElapsedTime >=5 && autonomousElapsedTime < 8) {                          // Between 5 and 8 Seconds
      DrivePerodic(true, .065, 0, 0, navx);                                                        // Move Backwards
      timedIntake(3);                                                                            // Run Intake
      if (isGamePieceLoaded == true) {                                                            // IF Game Piece is loaded
        DrivePerodic(true, 0, 0, 0, navx);                                                       // Stop Moving
        IntakeArmSpeakerPosition();                                                               // Move to Speaker Arm Position
      }
    } else if (autonomousElapsedTime >=8 && autonomousElapsedTime < 10) {                         // Between 8 and 10 Seconds
      DrivePerodic(true, -.07, 0, 0, navx);                                                           // Move Forwards
      if (autonomousElapsedTime >= 9 && autonomousElapsedTime < 9.1){
      TimedShooter(3);
      }
    }else if (autonomousElapsedTime >= 10 && autonomousElapsedTime < 13) {                       // Between 10 and 13 Seconds
      DrivePerodic(true, 0, 0, 0, navx);                                                         // Stop Moving
                                                                              // Run Shooter
    } else if (autonomousElapsedTime > 15) {
      DrivePerodic(true, .0, 0, 0, navx);
    }else if (autonomousElapsedTime >= 13 && autonomousElapsedTime < 15) {
      DrivePerodic(true, .17, 0, 0, navx);
    }
  }

  private void testAutonomousTimedRoutine() {
    if (autonomousElapsedTime < 0.1) { 
      IntakeArmSpeakerPosition();
    }else if (autonomousElapsedTime > 1 && autonomousElapsedTime < 1.1) {
      timedIntake(3); 
    }else if (autonomousElapsedTime < 15 ) {
      DrivePerodic(true, -.07, 0, 0, navx);
    } 
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

    // Initiate Xbox Controllers
    xboxMovementController = new XboxController(0);  // Replace 0 with the port number of your movement Xbox controller
    xboxInteractionController = new XboxController(1);  // Replace 1 with the port number of your interaction Xbox controller

    // Add options to the controlModeChooser
    controlModeChooser.addOption("Field-Centric Control", true);
    controlModeChooser.addOption("Robot-Centric Control", false);

    // Add otpions to the Autonomus Chooser
    autonRoutineChooser.addOption("Default Routine",defaultRoutine);
    autonRoutineChooser.addOption("Test Routine",testRoutine);

    // Put the choosers on the SmartDashboard
    SmartDashboard.putData("Control Mode Chooser", controlModeChooser);
    SmartDashboard.putData("Autonomus Routine Chooser", autonRoutineChooser);

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
  public void robotPeriodic() {
    if (isGamePieceLoaded == true){
      SetState(2);
    } else {SetState(1);}
  }

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
    autonomousStartTime = Timer.getFPGATimestamp();
  }

   @Override
  public void autonomousPeriodic() {

    // Update Elapsed Time
    autonomousElapsedTime = Timer.getFPGATimestamp() - autonomousStartTime;
    
    // Update Periodic Methods
    InteractionPeriodic();
    RobotStatePeriodic();

    // Execute the corresponding autonomous routine
    if (autonRoutineChooser.getSelected() == testRoutine) {
      defaultAutonomousTimedRoutine();
    } else {
      testAutonomousTimedRoutine();
    }
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

    if (Math.abs(xboxMovementController.getRightX()) > zDeadZone) // zAxis changes based on if we have two controllers (operators) or not
      {zAxisValue = filterZ.calculate(xboxMovementController.getRightX() * zModifier);
    }  
    
    // Call the Periodic Methods
    ButtonControllsPeriodic();
    InteractionPeriodic();
    RobotStatePeriodic();

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
    DrivePerodic(fieldCentricControl, yAxisValue, xAxisValue, zAxisValue, navx);
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
    SmartDashboard.putNumber("Simluated Right Shooter Roller Motor Power", rightShooterRoller.get());
    SmartDashboard.putNumber("Simluated Intake Roller Motor Power", intakeRoller.get());
    SmartDashboard.putNumber("Simluated Left Climber Motor Power", leftClimber.get());
    SmartDashboard.putNumber("Simluated Right Climber Motor Power", rightClimber.get());
    SmartDashboard.putNumber("Simluated Intake Pivot Motor Power", intakeArm.get());
  }
}
