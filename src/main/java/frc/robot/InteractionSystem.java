package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class InteractionSystem {
    /**
     * Controls interaction mechanisms like intakes and shooters, offering methods
     * for operation and management of these systems within the robot.
     */
    private CANSparkMax leftShooterRoller, rightShooterRoller, intakeRoller, leftClimber, rightClimber;
    private boolean isIntakeRunning = false;
    private double intakeStartTime = 0;
    private double intakeDuration = 0;
    private boolean isShooterRunning = false;
    private double shooterStartTime = 0;
    private double shooterDuration = 0;
    private StateManager StateManager;
    

    public InteractionSystem(StateManager StateManager) {
        // Initializes motors for intake and shooter systems as brushless motors
        leftShooterRoller = new CANSparkMax(Constants.leftShooterRoller, MotorType.kBrushless);
        rightShooterRoller = new CANSparkMax(Constants.rightShooterRoller, MotorType.kBrushless);
        intakeRoller = new CANSparkMax(Constants.intakeRollerID, MotorType.kBrushless);
        leftClimber = new CANSparkMax(Constants.leftClimberMotorID, MotorType.kBrushless);
        rightClimber = new CANSparkMax(Constants.rightClimberMotorID, MotorType.kBrushless);

        // Configure initial motor settings (e.g., inversion)
        configureMotors();
        this.StateManager = StateManager; // Assign passed StateManager object to field
    }

    private void configureMotors() {
         // Sets the direction of motor rotation to match physical setup
        leftShooterRoller.setInverted(true); 
        rightShooterRoller.setInverted(false);
        intakeRoller.setInverted(false);
    }

    public void runIntake(double speed) {
        // Sets intake motor speeds; positive values intake, negative values expel
        StateManager.setState(4);
        intakeRoller.set(speed);
        System.out.println(speed);
        if (Constants.debug) {

            // output value to smart dashboard
            SmartDashboard.putNumber("Current Intake Roller Motor Value", intakeRoller.getAppliedOutput());
        }
    }

    public void stopIntake() {
        // Stops the intake motors
        intakeRoller.set(0);
        if (Constants.debug) {

            // output value to smart dashboard
            SmartDashboard.putNumber("Current Intake Roller Motor Value", intakeRoller.getAppliedOutput());
        }
        StateManager.clearOverrideState();
    }

    public void timedIntake(double speed, double duration) {
        // Starts intake motors and schedules it to stop after a duration
        this.runIntake(speed); // Start the intake
        isIntakeRunning = true; // Flag to track intake state
        intakeStartTime = Timer.getFPGATimestamp(); // Record start time for duration tracking
        intakeDuration = duration; // Set how long the intake should run
    }

    public void runShooter(double speed) {
         // Sets shooter motor speeds; positive for shooting, negative could reverse feed
        StateManager.setState(5);
        leftShooterRoller.set(speed);
        rightShooterRoller.set(speed);
        intakeRoller.set(-speed);
        if (Constants.debug) {

            // output value to smart dashboard
            SmartDashboard.putNumber("Current Left Shooter Motor Value", leftShooterRoller.getAppliedOutput());
            SmartDashboard.putNumber("Current right Shooter Motor Value", rightShooterRoller.getAppliedOutput());
        }
    }

    public void stopShooter() {
        // Stops the shooter motors
        StateManager.clearOverrideState();
        leftShooterRoller.set(0);
        rightShooterRoller.set(0);
        intakeRoller.set(0);
        if (Constants.debug) {

            // output value to smart dashboard
            SmartDashboard.putNumber("Current Left Shooter Motor Value", leftShooterRoller.getAppliedOutput());
            SmartDashboard.putNumber("Current right Shooter Motor Value", rightShooterRoller.getAppliedOutput());
        }
    }

    public void raiseArms(){
        leftClimber.set(0.1);
        rightClimber.set(0.1);
        if (Constants.debug) {

            // output value to smart dashboard
            SmartDashboard.putNumber("Current Left Climber Motor Value", leftClimber.getAppliedOutput());
            SmartDashboard.putNumber("Current Right Climber Motor Value", rightClimber.getAppliedOutput());
        }
    }

    public void lowerArms(){
        leftClimber.set(-0.1);
        rightClimber.set(-0.1);
        if (Constants.debug) {

            // output value to smart dashboard
            SmartDashboard.putNumber("Current Left Climber Motor Value", leftClimber.getAppliedOutput());
            SmartDashboard.putNumber("Current Right Climber Motor Value", rightClimber.getAppliedOutput());
        }
    }

    public void stopArms(){
        leftClimber.set(0);
        rightClimber.set(0);
        if (Constants.debug) {

            // output value to smart dashboard
            SmartDashboard.putNumber("Current Left Climber Motor Value", leftClimber.getAppliedOutput());
            SmartDashboard.putNumber("Current Right Climber Motor Value", rightClimber.getAppliedOutput());
        }
    }

    public void timedShooter(double speed, double duration) {
        // Starts shooter motors and schedules it to stop after a set duration
        this.runShooter(speed); // Start the intake
        isShooterRunning = true;
        shooterStartTime = Timer.getFPGATimestamp(); // Current time
        shooterDuration = duration;
    }


    // Call this method from the Robot class's periodic methods to update intake and shooter states
    public void update() {
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
}
