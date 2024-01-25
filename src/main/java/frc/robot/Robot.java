// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private Joystick controller;
  private CANSparkMax frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor;
  private MecanumDrive mecanumDrive;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);


  controller = new Joystick(0);  


  frontLeftMotor = new CANSparkMax(12, MotorType.kBrushless);
  rearLeftMotor = new CANSparkMax(13, MotorType.kBrushless);
  frontRightMotor = new CANSparkMax(4, MotorType.kBrushless);
  rearRightMotor = new CANSparkMax(5, MotorType.kBrushless);


  mecanumDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);


  }


 
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {// Drive with mecanum drive
   
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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }


  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }


  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}


  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
     mecanumDrive.driveCartesian(0.1, 0.1, 0.1);


        controller.getX();  // X axis of the joystick for lateral movement
        controller.getY(); // Y axis of the joystick for forward/backward movement (negative to align with standard orientation)
        controller.getZ();  // Rotation controlled by the Z axis of the joystick
                            // Optional gyro angle, set to 0 if not using gyro
  ;
  double yAxisValue = 0;
  double xAxisValue = 0;
  double zAxisValue = 0;
  double controllerDeadZone = 0.2;
  if (Math.abs(controller.getY()) > controllerDeadZone){
    yAxisValue = -controller.getY();
  }
  if (Math.abs(controller.getX()) > controllerDeadZone){
    xAxisValue = controller.getX();


  }
 if (Math.abs(controller.getZ()) > controllerDeadZone){
    zAxisValue = controller.getZ();
 }
  double denominator = Math.max(Math.abs(yAxisValue + xAxisValue + zAxisValue),1);
  double frontLeftPower = (yAxisValue + xAxisValue + zAxisValue) / denominator;
  double backLeftPower = (yAxisValue - xAxisValue + zAxisValue) / denominator;
  double frontRightPower = (yAxisValue - xAxisValue - zAxisValue) / denominator;
  double backRightPower = (yAxisValue + xAxisValue - zAxisValue) / denominator;


    double multiplier = (-controller.getThrottle() + 1) / 2;


frontLeftMotor.set(frontLeftPower * multiplier);
rearLeftMotor.set(backLeftPower * multiplier);
frontRightMotor.set(-frontRightPower * multiplier);
rearRightMotor.set(-backRightPower * multiplier);


// Example: Control an additional mechanism (e.g., an arm) with a button
if (controller.getRawButton(1)) {
    // Code to control the mechanism when button 1 is pressed
}


// Example: Control another mechanism (e.g., intake) with a different button
if (controller.getRawButton(2)) {
    // Code to control the mechanism when button 2 is pressed
}}


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


