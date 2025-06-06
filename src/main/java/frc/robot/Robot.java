// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


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
import edu.wpi.first.cameraserver.CameraServer;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;

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
  */

  // PWM Channels
  private static final int blinkinPWMChannel = 1;

  // DIO Channels
  private static final int gamePieceDetectionSwitchDIOChannel = 1;

  // Movement Modifiers
  private static final double yModifier = 1;
  private static final double xModifier = 1;
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
  private static final double climberSpeed = 0.60;

  // Intake Arm Speed
  private static final double intakeArmSpeed = 0.60;

  // Intake Time
  private static final double intakeTime = 5;

  // Shooter Time
  private static final double shooterTime = 0.75;

  // Intake Arm Move to/from Intake Position and Speaker Position
  private static final double armIntakeSpeakerPositionTime = 0.55;

  // Intake Arm Move to/from Intake Position and Amp Position
  private static final double armAmpIntakePositionTime = 0.25;

  // Intake Arm Move to/from Speaker Position and Amp Position
  private static final double armAmpSpeakerPositionTime = 0.30;

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

  // These boolean variables are used to determine control options
  private boolean fieldCentricControl;

  // Movement Speed Variable
  private double movementSpeed;

  // isGamePieceLoaded Variable
  private boolean isGamePieceLoaded = false;  
  private boolean isGamePieceLimitSwitchTriggured = false;

  private static final String DefaultAuto = "1 - Front Speaker Position - 12 Point Routine";
  private static final String secondRoutine = "2 - Any Valid Speaker Position - 7 Point Routine ";
  private static final String thirdRoutine = "3 - Any Valid Speaker Position - 5 Point Routine";
  private static final String forthRoutine = "4 - (untested) Left? Speaker Position - 12 Point alternative Routine";
  private static final String fifthRoutine = "5 - (untested) Right? Speaker Position- 12 Point Alternate Routine";
  private static final String sixthRoutine = "6 - Front Speaker Position- 17 Point Left Note Routine ";
  private static final String seventhRoutine = "7 - Front Speaker Position- 22 Point Routine";
  private static final String eighthRoutine = "8 - Front Speaker Position - 17 Point Right Note Routine";
  private static final String ninthRoutine = "9 - Blue Alliance - Amp Side - Hurricane";

  // Creates SlewRateLimiter objects for each axis that limits the rate of change. This value is max change per second. For most imports, the range here is  -1 to 1 
  SlewRateLimiter filterX = new SlewRateLimiter(3); 
  SlewRateLimiter filterY = new SlewRateLimiter(3);
  SlewRateLimiter filterZ = new SlewRateLimiter(3);

  // Setup LED PWM Outputs
  public static final double ledRed = 0.61;
  public static final double ledOrange = 0.65;
  public static final double ledYellow = 0.69;
  public static final double ledGreen = 0.77;
  public static final double ledBlue = 0.87;
  public static final double ledBlack = 0.99;
  public static final double ledPattern = 0.49;

  private PWMSparkMax blinkinLED = new PWMSparkMax(Robot.blinkinPWMChannel);

  private double autonomousStartTime;
  private double autonomousElapsedTime;

  // Objects representing Motors and their encoders
  private CANSparkMax leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
  private RelativeEncoder leftFrontEncoder, leftBackEncoder, rightFrontEncoder, rightBackEncoder;
  private CANSparkMax leftShooterRoller, rightShooterRoller, intakeRoller, leftClimber, rightClimber;
  private RelativeEncoder leftShooterRollerEncoder, rightShooterRollerEncoder, intakeRollerEncoder, leftClimberEncoder, rightClimberEncoder;
  
  
  // Intake Arm Objects
  private CANSparkMax intakeArm;
  private RelativeEncoder intakeArmEncoder;
  private int IntakeArmPosition = 2; // 0 = Intake Position, 1 = Amp Position, 2 = Speaker Position

  private boolean isIntakeRunning = false; // Intake Running Tracker
  private double intakeStartTime = Timer.getFPGATimestamp(); // Start Time tracker
  private double intakeDuration = 0;

  private boolean isShooterRunning = false;  
  private double shooterStartTime = 0;
  private double shooterDuration = 0;

  private double intakeArmStartTime = 0;
  private double intakeArmDuration = 0;
  private boolean isIntakeArmRunning = false;
  private boolean intakeArmDirection;

  private int autonPhase = 0;  
  private boolean autonInitPhase = true;

  private double shooterSpinupTime = .25;

  ArrayList<Double> phaseStartTimes = new ArrayList<Double>(); // Create an Array List that we can add times without having to make a variable for each

  private double navxZeroStartTime = 0;
  private static double navxZeroIndicatorTime = 1;

  private void SetLEDColor(int desiredColor) {    
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
  switch (desiredColor) {
      case 1:
          blinkinLED.set(ledRed);
          break;
      case 2:
          blinkinLED.set(ledGreen);
          break;
      case 3:
          blinkinLED.set(ledBlue);
          break;
      case 4:
          blinkinLED.set(ledYellow);
          break;
      case 5:
          blinkinLED.set(ledOrange);
          break;
      default:
          blinkinLED.set(ledPattern);
          break;
    }
  }

  private void timedNavxZeroIndecator() {
    // This should each time the NavX is zeroed to set a timer for the LED status
    navxZeroStartTime = Timer.getFPGATimestamp();
  }

  private void LEDColorPeriodic() {
    // This should run every cycle to ensure the LED is set to display the correct color
    if (isShooterRunning == true){
      SetLEDColor(5);
    } else if (isIntakeRunning == true) {
      SetLEDColor(4);
    } else if ((Timer.getFPGATimestamp() - navxZeroStartTime) < navxZeroIndicatorTime) {
      SetLEDColor(3);
    } else if (isGamePieceLoaded || isGamePieceLimitSwitchTriggured) {
      SetLEDColor(2);
    } else {
      SetLEDColor(1);
    }
  }

  private void InteractionSystemInit() {
    // Initializes motors for intake and shooter systems as brushless motors
    leftShooterRoller = new CANSparkMax(leftShooterRollerMotorCANID, MotorType.kBrushless);
    rightShooterRoller = new CANSparkMax(rightShooterRollerMotorCANID, MotorType.kBrushless);
    intakeRoller = new CANSparkMax(intakeRollerMotorCANID, MotorType.kBrushless);
    leftClimber = new CANSparkMax(leftClimberMotorCANID, MotorType.kBrushless);
    rightClimber = new CANSparkMax(rightClimberMotorCANID, MotorType.kBrushless);
    leftClimberEncoder = leftClimber.getEncoder();
    rightClimberEncoder = rightClimber.getEncoder();

    intakeArm = new CANSparkMax(intakeArmPivitMotorCANID, MotorType.kBrushless);        
  
    // Configure initial motor settings (e.g., inversion)      
    leftShooterRoller.setInverted(true); 
    rightShooterRoller.setInverted(false);
    intakeRoller.setInverted(false);
    leftClimber.setInverted(true);
    rightClimber.setInverted(false);
    intakeArm.setInverted(false);
      
    // Set Idle Modes
    intakeArm.setIdleMode(IdleMode.kCoast);
    leftClimber.setIdleMode(IdleMode.kBrake);
    rightClimber.setIdleMode(IdleMode.kBrake);
  }

  private void runIntake(double speed) {
    // Sets intake motor speeds; positive values intake, negative values expel
    intakeRoller.set(speed);
  }

  private void stopIntake() {
      // Stops the intake motors
      intakeRoller.set(0);    
      isIntakeRunning = false;   
  }

  private void timedIntake(double duration) {
    // This should run once, each time the intake system is requested.
    intakeStartTime = Timer.getFPGATimestamp(); // Record start time for duration tracking
    isIntakeRunning = true; // Flag to track intake state
    intakeDuration = duration;
  }

  private void IntakeRollerPeriodic(){
    // This should run every cycle to ensure the intake roller is or isn't running as expected
    
    SmartDashboard.putNumber("Intake Motor Applied Output", intakeRoller.getAppliedOutput());
    if (isShooterRunning == false) {
      if (isGamePieceLimitSwitchTriggured == true){
        intakeStartTime = 0;
        isGamePieceLoaded = true;
      }

      if ((Timer.getFPGATimestamp() - intakeStartTime) < intakeDuration) {        
        // Starts intake motors and schedules it to stop after a duration
        runIntake(intakeSpeed); // Start the intake
        runShooter(-intakeSpeed, false);
        isIntakeRunning = true; // Flag to track intake state
      } else {
        stopIntake();
      }    
    }
  }

  private void runShooter(double speed, boolean spinup) {
    // Sets shooter motor speeds; positive for shooting, negative could reverse feed
    leftShooterRoller.set(speed);
    rightShooterRoller.set(speed);
    if (spinup == false) {
      runIntake(-speed);
      isGamePieceLoaded = false;
    }
    if (debug) {
      // output value to smart dashboard
      SmartDashboard.putNumber("Current Left Shooter Motor Value", leftShooterRoller.getAppliedOutput());
      SmartDashboard.putNumber("Current right Shooter Motor Value", rightShooterRoller.getAppliedOutput());
    }
  }

  private void stopShooter() {
    // Stops the shooter motors
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
    // this should run once, every time the shooter system is needed
    shooterStartTime = Timer.getFPGATimestamp(); // Record start time for duration tracking
    isShooterRunning = true; // Flag to track intake state
    shooterDuration = duration;
  }

  private void ShooterRollerPeriodic(){
    // This should run every cycle to ensure the shooter is or isn't running as expected
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

  private void MoveClimbers(double speed){
    leftClimber.set(speed);
    rightClimber.set(speed);
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
    // Intake Arm one time code
    intakeArmEncoder = intakeArm.getEncoder();
  }

  private void MoveIntakeArmUp(double speed) {
    intakeArm.set(speed);
    if (debug) {
      // Output Motor Values to Smart Dashboard for troubleshooting
      SmartDashboard.putNumber("Intake Arm Motor", intakeArm.getAppliedOutput());
    }
  }

  private void MoveIntakeArmDown(double speed) {
    intakeArm.set(-speed);
    if (debug) {
      // Output Motor Values to Smart Dashboard for troubleshooting
      SmartDashboard.putNumber("Intake Arm Motor", intakeArm.getAppliedOutput());
    }
  }
  private void StopIntakeArm() {
    intakeArm.set(0);
    if (debug) {
      // Output Motor Values to Smart Dashboard for troubleshooting
      SmartDashboard.putNumber("Intake Arm Motor", intakeArm.getAppliedOutput());
    }
  }

  private void IntakeArmSpeakerPosition(){
    /*
     * Case 0 = Intake arm is at intake position
     * Case 1 = Intake arm is at amp position
     * Case 2 = Intake arm is in speaker position
     */    
    intakeArm.setIdleMode(IdleMode.kCoast);
    switch(IntakeArmPosition) {
      case 0:
        timedIntakeArmUp(armIntakeSpeakerPositionTime);
        break;
      case 1:
        timedIntakeArmUp(armAmpSpeakerPositionTime);
        break;
      default:
        // timedIntakeArmUp(armIntakeSpeakerPositionTime);
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
    intakeArm.setIdleMode(IdleMode.kBrake);
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
    intakeArm.setIdleMode(IdleMode.kCoast);
    switch(IntakeArmPosition) {
      case 1:
        timedIntakeArmDown(armAmpIntakePositionTime);
        break;
      case 2:
        timedIntakeArmDown(armIntakeSpeakerPositionTime);
        break;
      default:
        // timedIntakeArmDown(armIntakeSpeakerPositionTime);
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
    SmartDashboard.putNumber("Intake Arm Value", intakeArmEncoder.getPosition());
    // Adjust Intake Arm Position based on readings from Enocder
    /*
    if (intakeArmEncoder.getPosition() > 0.25) {
      IntakeArmPosition = 2;
    } else if (intakeArmEncoder.getPosition() < 0.10) {
      IntakeArmPosition = 0;
    }
    */

    // This should run every cycle to ensure the intake arm is or isn't running as expected
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
    // This should run every cycle to check the game piece
    isGamePieceLimitSwitchTriggured = !gamePieceDetectionSwitch.get();  // Pulls the value returned fromt he Game Piece Detection Switch and sets it to isGamePieceLimitSwitchTriggured variable
    if (debug) {
      SmartDashboard.putBoolean("Gamepiece Limit Switch", isGamePieceLimitSwitchTriggured);
    }
  }

  private void ButtonControllsPeriodic() {    
    // This should run every teleop cycle to ensure the controls are being captured

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
     *   - Y Button: Move Climbers
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
      timedNavxZeroIndecator();
      navx.reset();
    }

    // If Right Trigger is pressed on the interaction controller, run the shooter
    if (xboxInteractionController.getRightTriggerAxis() > 0.2) {
      if (debug) {
        System.out.println("Start Shooter");
      }
      TimedShooter(shooterTime); // Adjust shooterSpeed to your desired speed
    }

    // If Left Trigger is pressed on the interaction controller, run the intake
    if (xboxInteractionController.getLeftTriggerAxis() > 0.2) {
      if (debug) {
        System.out.println("Start Intake");
      }
      timedIntake(intakeTime); // Adjust intakeSpeed to your desired speed
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
      if (xboxInteractionController.getPOV() == 0) {
        if ((leftClimberEncoder.getPosition() > -207 && rightClimberEncoder.getPosition() > -220)){
          MoveClimbers(-climberSpeed);
        }
      } else if (xboxInteractionController.getPOV() == 180 ) {
        if ((leftClimberEncoder.getPosition() < -10 && rightClimberEncoder.getPosition() < -10)){
          MoveClimbers(climberSpeed);
        }
      } else {
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
      leftFrontMotor.setIdleMode(IdleMode.kCoast);
      leftBackMotor.setIdleMode(IdleMode.kCoast);
      rightFrontMotor.setIdleMode(IdleMode.kCoast);
      rightBackMotor.setIdleMode(IdleMode.kCoast);
  }

  /**
  * Controls the robot's movement based on joystick input and control mode.
  * @param fieldCentric Determines if movement is relative to the field or robot.
  * @param yAxisValue Forward/reverse input.
  * @param xAxisValue Left/right input.
  * @param zAxisValue Rotation input.
  * @param ahrs The robot's gyroscope sensor for orientation.
  * @param debug Enables diagnostic output to SmartDashboard.
  */
  private void DrivePerodic(boolean fieldCentric,double yAxisValue, double xAxisValue, double zAxisValue, AHRS ahrs) {      
      if (fieldCentric){
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
     * 12 Point Routine from Center Position
     * =====================================
     * 
     * Shoot Loaded note
     * Move arm to intake position
     * Move twards the next game piece
      * Stop moving as soon as game piece is loaded
     * Move Intake Arm into Speaker Position
     * Drive forward
     * Run Shooter
     * Move Back
     */
    if (debug) { SmartDashboard.putString("Autonomous Routine", "defaultAutonomousTimedRoutine");}
    if (autonomousElapsedTime > 1 && autonomousElapsedTime < 1.1) {
      TimedShooter(shooterTime);
    } else if (autonomousElapsedTime > 1.75 && autonomousElapsedTime < 1.80) {
      IntakeArmIntakePosition();
    } else if (autonomousElapsedTime >=2 && autonomousElapsedTime < 3.5) {
      if (!isGamePieceLimitSwitchTriggured) {
        DrivePerodic(true, .13, 0, 0, navx);
        timedIntake(0.25);
      }
      if (isGamePieceLimitSwitchTriggured) {
        DrivePerodic(true, 0, 0, 0, navx); 
      }
    } if (autonomousElapsedTime >=3.5 && autonomousElapsedTime < 3.6){
      IntakeArmSpeakerPosition();
    } if (autonomousElapsedTime >=5 && autonomousElapsedTime < 7) {
      DrivePerodic(true, -.14, 0, 0, navx);
    } if (autonomousElapsedTime >= 7.25 && autonomousElapsedTime < 8){
      DrivePerodic(true, 0, 0, 0, navx);
      TimedShooter(shooterTime);
    } else if (autonomousElapsedTime >= 8.75 && autonomousElapsedTime < 15) {
      DrivePerodic(true, .17, 0, 0, navx);
    }else if (autonomousElapsedTime > 15) {
      DrivePerodic(true, .0, 0, 0, navx);
    }
  }

  private void secondAutonomousTimedRoutine() {
    /*
     * 2 Point Routine from any Position
     * =================================
     * 
     * Move back for 5 seconds
     * Stop
     */
    if (debug) { SmartDashboard.putString("Autonomous Routine", "secondAutonomousTimedRoutine");}
    if (autonomousElapsedTime > 1 && autonomousElapsedTime < 1.1) {
      TimedShooter(shooterTime);
    } else if (autonomousElapsedTime >=2 && autonomousElapsedTime < 5) {
      if (!isGamePieceLimitSwitchTriggured) {
        DrivePerodic(true, .13, 0, 0, navx);
      }
    }
  }

  private void thirdAutonomousTimedRoutine() {
    /*
     * 5 Point Routine from any Speaker Position
     * =========================================
     * 
     * Shoot Loaded note
     */
    if (debug) { SmartDashboard.putString("Autonomous Routine", "thirdAutonomousTimedRoutine");}
    if (autonomousElapsedTime > 1 && autonomousElapsedTime < 1.2) {
      TimedShooter(shooterTime);
    } 
  }

  private void forthAutonomousTimedRoutine() {
    if (debug) { SmartDashboard.putString("Autonomous Routine", "forthAutonomousTimedRoutine");}
    switch (autonPhase){
      case 0:
        if (autonInitPhase) {
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");}
          TimedShooter(shooterDuration); // Run Shooter
          autonInitPhase = false; // Get out of Init Phase
        }
        if (Timer.getFPGATimestamp() - phaseStartTimes.get(autonPhase) > shooterDuration) { // If this has been running longer than the shooter duration            
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
          if (debug) { System.out.println("Done");}
        }
        break; // Ensure execution stops here if this case is processed
      case 1:
        if (autonInitPhase) {
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");}
          IntakeArmIntakePosition();
          autonInitPhase = false;
        }
        DrivePerodic(true, .30, .30, .30, navx); // Move Diagnal and rotate
        if (navx.getRotation2d().getDegrees() > 50) {
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 2:
        if (autonInitPhase) {
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");}
          autonInitPhase = false;
        }
        DrivePerodic(true, .30, .30, 0, navx); // Move Diagnal 
        timedIntake(intakeTime);
        if (isGamePieceLimitSwitchTriggured == true) {
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 3:
        if (autonInitPhase) {
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");}
          IntakeArmSpeakerPosition();
          autonInitPhase = false;
        }
        if (Timer.getFPGATimestamp() - phaseStartTimes.get(autonPhase) > 0.75) {
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 4:
        if (autonInitPhase) {
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");}
          autonInitPhase = false;
        }
        DrivePerodic(true, -.30, -.30, -.30, navx); // Move Diagnal and rotate
        if (navx.getRotation2d().getDegrees() < 10) {
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 5:
        if (autonInitPhase) {
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");}
          autonInitPhase = false;
        }
        DrivePerodic(true, -.30, -.30, 0, navx); // Move Diagnal
        if (phaseStartTimes.get(3) - phaseStartTimes.get(2) > Timer.getFPGATimestamp() - phaseStartTimes.get(5)) { // if the time phase 2 took is greater than the time this phase has taken
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 6:
        if (autonInitPhase) {
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");}
          TimedShooter(shooterDuration);
          DrivePerodic(true, 0, 0, 0, navx); // Stop Moving
          autonInitPhase = false;
        }
        if (Timer.getFPGATimestamp() - phaseStartTimes.get(6) > shooterDuration){
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 7:
        if (autonInitPhase) {
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");}
          autonInitPhase = false;
        }
        DrivePerodic(true, .30, .30, .30, navx); // Move Diagnal and rotate
        if (navx.getRotation2d().getDegrees() > 50) {
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 8:
        if (autonInitPhase) {
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");}
          DrivePerodic(true, 0, 0, 0, navx); // Stop Moving
        }
        navx.reset();
        break; // Ensure execution stops here if this case is processed
    }
  }

  private void fifthAutonomousTimedRoutine() {
    if (debug) { SmartDashboard.putString("Autonomous Routine", "fifthAutonomousTimedRoutine");}
    switch (autonPhase){
      case 0:
        if (autonInitPhase) {
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");}
          TimedShooter(shooterDuration); // Run Shooter
          autonInitPhase = false; // Get out of Init Phase
        }
        if (Timer.getFPGATimestamp() - phaseStartTimes.get(autonPhase) > shooterDuration) { // If this has been running longer than the shooter duration            
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
          if (debug) { System.out.println("Done");}
        }
        break; // Ensure execution stops here if this case is processed
      case 1:
        if (autonInitPhase) {
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");}
          IntakeArmIntakePosition();
          autonInitPhase = false;
        }
        DrivePerodic(true, .30, -.30, -.30, navx); // Move Diagnal and rotate
        if (navx.getRotation2d().getDegrees() < -50) {
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 2:
        if (autonInitPhase) {
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");}
          autonInitPhase = false;
        }
        DrivePerodic(true, .30, .30, 0, navx); // Move Diagnal 
        timedIntake(intakeTime);
        if (isGamePieceLimitSwitchTriggured == true) {
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 3:
        if (autonInitPhase) {
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");}
          IntakeArmSpeakerPosition();
          autonInitPhase = false;
        }
        if (Timer.getFPGATimestamp() - phaseStartTimes.get(autonPhase) > 0.75) {
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 4:
        if (autonInitPhase) {
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");}
          autonInitPhase = false;
        }
        DrivePerodic(true, -.30, .30, .30, navx); // Move Diagnal and rotate
        if (navx.getRotation2d().getDegrees() > 10) {
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 5:
        if (autonInitPhase) {
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");}
          autonInitPhase = false;
        }
        DrivePerodic(true, -.30, .30, 0, navx); // Move Diagnal
        if (phaseStartTimes.get(3) - phaseStartTimes.get(2) > Timer.getFPGATimestamp() - phaseStartTimes.get(5)) { // if the time phase 2 took is greater than the time this phase has taken
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 6:
        if (autonInitPhase) {
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");}
          TimedShooter(shooterDuration);
          DrivePerodic(true, 0, 0, 0, navx); // Stop Moving
          autonInitPhase = false;
        }
        if (Timer.getFPGATimestamp() - phaseStartTimes.get(6) > shooterDuration){
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 7:
        if (autonInitPhase) {
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");}
          autonInitPhase = false;
        }
        DrivePerodic(true, .30, -.30, -.30, navx); // Move Diagnal and rotate
        if (navx.getRotation2d().getDegrees() < 50) {
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 8:
        if (autonInitPhase) {
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");}
          DrivePerodic(true, 0, 0, 0, navx); // Stop Moving
        }
        navx.reset();
        break; // Ensure execution stops here if this case is processed
    }
  }

  private void sixthAutonomousTimedRoutine() {
    double autonSpeed = 0.15;
    if (debug) { SmartDashboard.putString("Autonomous Routine", "sixthAutonomousTimedRoutine");}
    switch (autonPhase){
      case 0: // Phase Number
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          TimedShooter(shooterTime);
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   
        if (Timer.getFPGATimestamp() - phaseStartTimes.get(0) > shooterDuration) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 1: 
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          IntakeArmIntakePosition();
          DrivePerodic(true, autonSpeed, 0, .0, navx); 
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   
        runIntake(intakeSpeed);
        if (isGamePieceLimitSwitchTriggured) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 2:
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          IntakeArmSpeakerPosition();
          DrivePerodic(true, -autonSpeed, 0, .0, navx); 
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   
        if ((Timer.getFPGATimestamp() - phaseStartTimes.get(2)) > ((phaseStartTimes.get(2) - phaseStartTimes.get(1)))) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 3:
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          DrivePerodic(true, 0, 0, .0, navx);
          TimedShooter(shooterTime);
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   
        System.out.println("Time So far: " + (Timer.getFPGATimestamp() - phaseStartTimes.get(0)) + " Seconds");
        if ((Timer.getFPGATimestamp() - phaseStartTimes.get(3)) > shooterDuration) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 4:
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          DrivePerodic(true, 0.86*autonSpeed, -0.5*autonSpeed, -.15, navx); 
          IntakeArmIntakePosition();
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   \
        if (navx.getRotation2d().getDegrees() > 38) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 5:
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          DrivePerodic(true, 0.86*autonSpeed, -0.5*autonSpeed, 0, navx);
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   
        runIntake(intakeSpeed);
        if (isGamePieceLimitSwitchTriggured) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 6:
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          stopIntake();
          IntakeArmSpeakerPosition();
          DrivePerodic(true, -0.86*autonSpeed, 0.5*autonSpeed, 0, navx);
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   
        if (((Timer.getFPGATimestamp() - phaseStartTimes.get(6))-0.25) > (phaseStartTimes.get(6) - phaseStartTimes.get(5))) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 7:
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          DrivePerodic(true, -0.86*autonSpeed, 0.5*autonSpeed, 0.15, navx);
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   
        if (navx.getRotation2d().getDegrees() < 0) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 8:
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          DrivePerodic(true, 0, 0, 0, navx);
          TimedShooter(shooterTime);
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically 
        if (Timer.getFPGATimestamp() - phaseStartTimes.get(8) > 0.75) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 9: 
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          autonInitPhase = false; // Get out of Init Phase
          DrivePerodic(true, autonSpeed, 0, 0, navx);
        }
        // Stuff to do periodically   
        System.out.println("Time So far: " + (Timer.getFPGATimestamp() - phaseStartTimes.get(0)) + " Seconds");
        if (Timer.getFPGATimestamp() - phaseStartTimes.get(8) > 0.2) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 10:
        DrivePerodic(true, autonSpeed, 0, 0, navx);
        break;
    }
  }
  private void seventhAutonomousTimedRoutine() {
    double autonSpeed = 0.20;
    if (debug) { SmartDashboard.putString("Autonomous Routine", "seventhAutonomousTimedRoutine");}
    switch (autonPhase){
      case 0: 
        /*
         * Phase 0: Runs Shooter until it's been as long as the shooter takes to run
         */
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          TimedShooter(shooterTime);
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   
        if (Timer.getFPGATimestamp() - phaseStartTimes.get(0) > shooterDuration) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 1: 
        /*
         * Phase 1: back up and run intake
         */
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          IntakeArmIntakePosition();
          DrivePerodic(true, autonSpeed, 0, .0, navx); 
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically 
        runIntake(intakeSpeed);
        if (isGamePieceLimitSwitchTriggured) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 2:
        /*
         * Phase 2: Drive Forward for as long as it took to drive backwards in the previous phase
         */
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          IntakeArmSpeakerPosition();
          DrivePerodic(true, -autonSpeed, 0, .0, navx); 
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   
        if ((Timer.getFPGATimestamp() - phaseStartTimes.get(2)) > ((phaseStartTimes.get(2) - phaseStartTimes.get(1)))) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 3:
        /*
         * Phase 3: Stop moving and run shooter
         */
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          DrivePerodic(true, 0, 0, .0, navx);
          TimedShooter(shooterTime);
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   
        System.out.println("Time So far: " + (Timer.getFPGATimestamp() - phaseStartTimes.get(0)) + " Seconds");
        if ((Timer.getFPGATimestamp() - phaseStartTimes.get(3)) > shooterDuration) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 4:
        /*
         * Phase 4: Put intake arm into intake position, 
         *          move diagnally and rotate until robot 
         *          is at 34 degrees. 
         */
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          DrivePerodic(true, 0.86*autonSpeed, -0.5*autonSpeed, -.15, navx); 
          IntakeArmIntakePosition();
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   \
        if (navx.getRotation2d().getDegrees() > 38) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 5:
        /*
         * Phase 5: Move diagnally, run intake, 
         *          and stop once a game piece is loaded
         */
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          DrivePerodic(true, 0.86*autonSpeed, -0.5*autonSpeed, 0, navx);
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically
        runIntake(intakeSpeed);
        if (isGamePieceLimitSwitchTriggured) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 6:
        /*
         * Phase 6: Bring intake arm into speaker position, 
         *          drive back diagnally for as long as it 
         *          took in the previous phase
         */
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          stopIntake();
          IntakeArmSpeakerPosition();
          DrivePerodic(true, -0.86*autonSpeed, 0.5*autonSpeed, 0, navx);
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   
        if (((Timer.getFPGATimestamp() - phaseStartTimes.get(6))-0.5) > (phaseStartTimes.get(6) - phaseStartTimes.get(5))) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 7:
        /*
         * Phase 7: move back diagnally and rotate until 
         *          we are at less than 0 degrees
         */
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          DrivePerodic(true, -0.86*autonSpeed, 0.5*autonSpeed, 0.15, navx);
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   
        if (navx.getRotation2d().getDegrees() < 0) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 8:
        /*
         * Phase 8: Stop moving and run shooter 
         *          for the shooter duration
         */
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          DrivePerodic(true, 0, 0, 0, navx);
          TimedShooter(shooterTime);
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically 
        if (Timer.getFPGATimestamp() - phaseStartTimes.get(8) > 0.75) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed        
      case 9:
        /*
         * Phase 9: Bring intake arm out to intake position and move diagnally 
         * twards the other piece, while rotating until we hit -34 degress
         */
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          DrivePerodic(true, 0.86*autonSpeed, 0.5*autonSpeed, .15, navx); 
          IntakeArmIntakePosition();
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   \
        if (navx.getRotation2d().getDegrees() < -38) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 10:
        /*
         * Phase 10: just move diagnally and run intake
        *            until a game piece is loaded
         */
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          DrivePerodic(true, 0.86*autonSpeed, 0.5*autonSpeed, 0, navx);
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   
        runIntake(intakeSpeed);
        if (isGamePieceLimitSwitchTriggured) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 11:
         /*
         * Phase 11: Stop intake, put arm in speaker position
         *           drive back to speaker diagnally
         *           and move on once it's been as long as 
         *           it look in the last phase
         */
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          stopIntake();
          IntakeArmSpeakerPosition();
          DrivePerodic(true, -0.86*autonSpeed, -0.5*autonSpeed, 0, navx);
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   
        if (((Timer.getFPGATimestamp() - phaseStartTimes.get(11))-0.25) > (phaseStartTimes.get(11) - phaseStartTimes.get(10))) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 12:
         /*
         * Phase 12: Rotate and move diagnally until 
         *           the robot is past 0 degrees
         */
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          DrivePerodic(true, -0.86*autonSpeed, -0.5*autonSpeed, -0.15, navx);
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   
        if (navx.getRotation2d().getDegrees() > -5) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 13:
         /*
         * Phase 13: Stop moving, run shooter and
         *           move on once it's done
         */
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          DrivePerodic(true, 0, 0, 0, navx);
          TimedShooter(shooterTime);
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically 
        if (Timer.getFPGATimestamp() - phaseStartTimes.get(13) > 0.75) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 14:
         /*
         * Phase 14: move back
         */
        DrivePerodic(true,autonSpeed,0,0,navx);
        break;
    }
  }
  private void eighthAutonomousTimedRoutine() {
    double autonSpeed = 0.15;
    if (debug) { SmartDashboard.putString("Autonomous Routine", "eighthAutonomousTimedRoutine");}
    switch (autonPhase){
      case 0: // Phase Number
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          TimedShooter(shooterTime);
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   
        if (Timer.getFPGATimestamp() - phaseStartTimes.get(0) > shooterDuration) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 1: 
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          IntakeArmIntakePosition();
          DrivePerodic(true, autonSpeed, 0, .0, navx); 
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   
        runIntake(intakeSpeed);
        if (isGamePieceLimitSwitchTriggured) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 2:
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          IntakeArmSpeakerPosition();
          DrivePerodic(true, -autonSpeed, 0, .0, navx); 
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   
        if ((Timer.getFPGATimestamp() - phaseStartTimes.get(2)) > ((phaseStartTimes.get(2) - phaseStartTimes.get(1)))) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 3:
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          DrivePerodic(true, 0, 0, .0, navx);
          TimedShooter(shooterTime);
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   
        System.out.println("Time So far: " + (Timer.getFPGATimestamp() - phaseStartTimes.get(0)) + " Seconds");
        if ((Timer.getFPGATimestamp() - phaseStartTimes.get(3)) > shooterDuration) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 4:
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          DrivePerodic(true, 0.86*autonSpeed, 0.5*autonSpeed, .15, navx); 
          IntakeArmIntakePosition();
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   \
        if (navx.getRotation2d().getDegrees() < -38) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 5:
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          DrivePerodic(true, 0.86*autonSpeed, 0.5*autonSpeed, 0, navx);
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   
        runIntake(intakeSpeed);
        if (isGamePieceLimitSwitchTriggured) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 6:
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          stopIntake();
          IntakeArmSpeakerPosition();
          DrivePerodic(true, -0.86*autonSpeed, -0.5*autonSpeed, 0, navx);
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   
        if (((Timer.getFPGATimestamp() - phaseStartTimes.get(6))-0.25) > (phaseStartTimes.get(6) - phaseStartTimes.get(5))) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 7:
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          DrivePerodic(true, -0.86*autonSpeed, -0.5*autonSpeed, -0.15, navx);
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   
        if (navx.getRotation2d().getDegrees() > 0) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 8:
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          DrivePerodic(true, 0, 0, 0, navx);
          TimedShooter(shooterTime);
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically 
        if (Timer.getFPGATimestamp() - phaseStartTimes.get(8) > 0.75) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 9: 
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          autonInitPhase = false; // Get out of Init Phase
          DrivePerodic(true, autonSpeed, 0, 0, navx);
        }
        // Stuff to do periodically   
        System.out.println("Time So far: " + (Timer.getFPGATimestamp() - phaseStartTimes.get(0)) + " Seconds");
        if (Timer.getFPGATimestamp() - phaseStartTimes.get(8) > 0.2) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 10:
        DrivePerodic(true, autonSpeed, 0, 0, navx);
        break;
    }
  }
  private void ninthAutonomousTimedRoutine() {
    double autonSpeed = 0.15;
    if (debug) { SmartDashboard.putString("Autonomous Routine", "ninthAutonomousTimedRoutine");}
    switch (autonPhase){
      case 0: 
        /*
         * Phase 0: Runs Shooter until it's been as long as the shooter takes to run
         */
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          TimedShooter(shooterTime);
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   
        if (Timer.getFPGATimestamp() - phaseStartTimes.get(autonPhase)) > shooterDuration) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 1: 
        /*
         * Phase 1: back up
         */
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          DrivePerodic(true, autonSpeed, 0, 0, navx); 
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically 
        if ((Timer.getFPGATimestamp() - phaseStartTimes.get(autonPhase)) > 0.5) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 2: // Phase Number
        /*
         * Phase 2: Drive Sideways
         */
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          DrivePerodic(true, autonSpeed*0.5, autonSpeed*-0.81, 0, navx); 
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically 
        if ((Timer.getFPGATimestamp() - phaseStartTimes.get(autonPhase)) > 0.7) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 3: // Phase Number
        /*
         * Phase 3: Stop for .4 seconds
         */
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          leftFrontMotor.setIdleMode(IdleMode.kBrake);
          leftBackMotor.setIdleMode(IdleMode.kBrake);
          rightFrontMotor.setIdleMode(IdleMode.kBrake);
          rightBackMotor.setIdleMode(IdleMode.kBrake);
          DrivePerodic(true, 0, 0, 0, navx); 
          autonInitPhase = false; // Get out of Init Phase
        }
        if ((Timer.getFPGATimestamp() - phaseStartTimes.get(autonPhase)) > 0.4) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 4: // Phase Number
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          leftFrontMotor.setIdleMode(IdleMode.kCoast);
          leftBackMotor.setIdleMode(IdleMode.kCoast);
          rightFrontMotor.setIdleMode(IdleMode.kCoast);
          rightBackMotor.setIdleMode(IdleMode.kCoast);
          DrivePerodic(true, autonSpeed*0.5, autonSpeed*0.81, 0.5, navx); 
          autonInitPhase = false; // Get out of Init Phase
        }
        if ((Timer.getFPGATimestamp() - phaseStartTimes.get(autonPhase)) > 1.2) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 5: // Phase Number
        phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
        if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
        DrivePerodic(true, 0, 0, 0, navx); 
        autonInitPhase = false; // Get out of Init Phase
        break; // Ensure execution stops here if this case is processed
    }
  }
  private void templateAutonomousRoutine() {
    if (debug) { SmartDashboard.putString("Autonomous Routine", "templateAutonomousTimedRoutine");}
    switch (autonPhase){
      case 0: // Phase Number
        if (autonInitPhase) { // Setup if statement for one time auton
          phaseStartTimes.add(Timer.getFPGATimestamp()); // Set Timer for phase
          if (debug) { System.out.println("Entering Phase "+autonPhase+" of Routine");} // Let us know which phase we're on
          autonInitPhase = false; // Get out of Init Phase
        }
        // Stuff to do periodically   
        System.out.println("Time So far: " + (Timer.getFPGATimestamp() - phaseStartTimes.get(0)) + " Seconds");
        if (true) { // Condition to move into next phase
          autonInitPhase = true; // Switch back to Init
          autonPhase++; // Move to the next Phase
        }
        break; // Ensure execution stops here if this case is processed
      case 1: 
        System.out.println("Phase 1");
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
    CameraServer.startAutomaticCapture("Intake Cam", 0);
    // Call on the constructor methods
    DriveTrainInit();
    InteractionSystemInit();
    IntakeArmInit();
    StopClimbers();
    // Initiate Xbox Controllers
    xboxMovementController = new XboxController(0);  // Replace 0 with the port number of your movement Xbox controller
    xboxInteractionController = new XboxController(1);  // Replace 1 with the port number of your interaction Xbox controller

    // Add options to the controlModeChooser
    controlModeChooser.addOption("Field-Centric Control", true);
    controlModeChooser.addOption("Robot-Centric Control", false);

    // Add otpions to the Autonomus Chooser
    autonRoutineChooser.addOption("1 - Front Speaker Position - 12 Point Routine",DefaultAuto);
    autonRoutineChooser.addOption("2 - Any Valid Speaker Position - 7 Point Routine ",secondRoutine);
    autonRoutineChooser.addOption("3 - Any Valid Speaker Position - 5 Point Routine",thirdRoutine);
    autonRoutineChooser.addOption("4 - (untested) Left? Speaker Position - 12 Point alternative Routine",forthRoutine);
    autonRoutineChooser.addOption("5 - (untested) Right? Speaker Position- 12 Point Alternate Routine",fifthRoutine);
    autonRoutineChooser.addOption("6 - Front Speaker Position- 17 Point Left Note Routine ",sixthRoutine);
    autonRoutineChooser.addOption("7 - Front Speaker Position- 22 Point Routine",seventhRoutine);
    autonRoutineChooser.addOption("8 - Front Speaker Position - 17 Point Right Note Routine",eighthRoutine);
    autonRoutineChooser.addOption("9 - Blue Alliance - Amp Side - Hurricane",ninthRoutine);

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
    // If statement to see if our Mode Choser outputs worked, and if not, have some fall back values (Mostly for Simulation Mode)
    if (controlModeChooser.getSelected() != null) {      
      // Set variables fieldCentricControl and twoControllerMode to options selected on interactive chooser by the operators
      fieldCentricControl = controlModeChooser.getSelected();
    } else {
      // Handle the case where one or both values are null (simulation mode). Also log message because that is probably interesting to see
      fieldCentricControl = true;      
    }
    
    if (debug) {
      SmartDashboard.putNumber("Left Climber Encoder Position", leftClimberEncoder.getPosition());
      SmartDashboard.putNumber("Right Climber Encoder Position", rightClimberEncoder.getPosition());
      SmartDashboard.putBoolean("Field Centric Control", fieldCentricControl);
    }

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
    LEDColorPeriodic();
    
    if (debug) { SmartDashboard.putNumber("Autonomous Phase", autonPhase);}

    // Execute the corresponding autonomous routine
    switch (autonRoutineChooser.getSelected()) {
      case secondRoutine:
        secondAutonomousTimedRoutine();
        break;
      case thirdRoutine:
        thirdAutonomousTimedRoutine();
        break;
      case forthRoutine:
        forthAutonomousTimedRoutine();
        break;
      case fifthRoutine:
        fifthAutonomousTimedRoutine();
        break;
      case sixthRoutine:
        sixthAutonomousTimedRoutine();
        break;
      case seventhRoutine:
        seventhAutonomousTimedRoutine();
        break;
      case eighthRoutine:
        eighthAutonomousTimedRoutine();
        break;
      case ninthRoutine:
        ninthAutonomousTimedRoutine();
        break;
      default:
        defaultAutonomousTimedRoutine();
        break;
    }
  }
  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // If debug mode is on, write a line that lets us know what mode we're entering
    if (debug) { System.out.println("Entering teleopInit Phase");}    
    leftFrontMotor.setIdleMode(IdleMode.kCoast);
    leftBackMotor.setIdleMode(IdleMode.kCoast);
    rightFrontMotor.setIdleMode(IdleMode.kCoast);
    rightBackMotor.setIdleMode(IdleMode.kCoast);
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
    LEDColorPeriodic();

    // If debug mode is on, provide diagnostic information to the smart dashboard
    if (debug) {
      // Output X, Y, and Z values to Smart Dashboard for Troubleshooting
      SmartDashboard.putNumber("Current X Value", xAxisValue);
      SmartDashboard.putNumber("Current Y Value", yAxisValue);
      SmartDashboard.putNumber("Current Z Value", zAxisValue);
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
  public void disabledPeriodic() {
  }

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
