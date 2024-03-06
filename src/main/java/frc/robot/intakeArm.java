package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class intakeArm {    
    private PIDController pidController;
    private CANSparkMax intakeArm;
    private boolean isMovingArm;
    private double targetPosition;    

    // Constructor
    public intakeArm() {
        intakeArm = new CANSparkMax(Constants.intakeArmID, MotorType.kBrushless);        
        intakeArm.setInverted(false);
        // Setup PID Controller
        double kP = 0.1;
        double kI = 0.0;
        double kD = 0.0;
        pidController = new PIDController(kP, kI, kD);
        isMovingArm = false;
        targetPosition = 0.0;
    }

    public void setTargetPosition(double position) {
        targetPosition = position;
        isMovingArm = true;
    }

    public void moveToIntakePosition(DutyCycleEncoder intakeHexEncoder) { 
        targetPosition = Constants.intakePosition;
        intakeController(intakeHexEncoder, Constants.intakePosition);     
    }

    public void moveToAmpPosition(DutyCycleEncoder intakeHexEncoder) {
        targetPosition = Constants.ampPosition;
        intakeController(intakeHexEncoder, Constants.ampPosition);  
    }

    public void moveToSpeakerPosition(DutyCycleEncoder intakeHexEncoder) {
        targetPosition = Constants.speakerPosition;
        intakeController(intakeHexEncoder, Constants.speakerPosition);  
    }

    private void intakeController(DutyCycleEncoder intakeHexEncoder, double desiredLocation) {
        pidController.setSetpoint(desiredLocation);
        // Calculate the control output using the PID controller
        double output = pidController.calculate(intakeHexEncoder.get());

        // Check if the arm is at or near the target position
        if (Math.abs(intakeHexEncoder.get() - desiredLocation) < Constants.intakePositionTolerance) {
            intakeArm.set(0.0); // Stop the motor movement
        } else {
            pidController.setSetpoint(desiredLocation);
            intakeArm.set(output);
        }
    }

    public void update(DutyCycleEncoder intakeHexEncoder) {
        if (isMovingArm) {
            // Calculate the control output using the PID controller
            double output = pidController.calculate(intakeHexEncoder.get());

            // Check if the arm is at or near the target position
            if (Math.abs(intakeHexEncoder.get() - targetPosition) < Constants.intakePositionTolerance) {
                intakeArm.set(0.0); // Stop the motor movement
                isMovingArm = false; // Reset the flag
                if (Constants.debug) {
                    // output value to smart dashboard
                    SmartDashboard.putNumber("Current Intake Arm Motor Value", 0);
                }
            } else {
                pidController.setSetpoint(targetPosition);
                intakeArm.set(output);
                if (Constants.debug) {

                    // output value to smart dashboard
                    SmartDashboard.putNumber("Current Intake Arm Motor Value", output);
                }
            }
        }
    }
}
