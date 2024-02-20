package frc.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

public class DriveTrain {

    /**
     * Manages the robot's drive system, including motor initialization and driving
     * logic. Supports both field-centric and robot-centric control modes, handling
     * input processing and motor speed calculations to facilitate smooth and
     * responsive movement.
     */

    private CANSparkMax leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;

    public DriveTrain() {
        // Initialize each motor with its respective ID from Constants and set them as brushless
        leftFrontMotor = new CANSparkMax(Constants.leftFrontMotorID, MotorType.kBrushless);
        leftBackMotor = new CANSparkMax(Constants.leftBackMotorID, MotorType.kBrushless);
        rightFrontMotor = new CANSparkMax(Constants.rightFrontMotorID, MotorType.kBrushless);
        rightBackMotor = new CANSparkMax(Constants.rightBackMotorID, MotorType.kBrushless);

        configureMotors(); // Call method to configure initial motor settings
    }

    private void configureMotors() {
        // Set motor inversion to match physical drive train configuration
        leftFrontMotor.setInverted(false);
        leftBackMotor.setInverted(false);
        rightFrontMotor.setInverted(true);
        rightBackMotor.setInverted(true);
    }

    // Provide access to each motor, useful for diagnostics or advanced control
    public CANSparkMax getLeftFrontMotor() {
        return leftFrontMotor;
    }

    public CANSparkMax getLeftBackMotor() {
        return leftBackMotor;
    }

    public CANSparkMax getRightFrontMotor() {
        return rightFrontMotor;
    }

    public CANSparkMax getRightBackMotor() {
        return rightBackMotor;
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
    public void drive(boolean fieldCentricControl,double yAxisValue, double xAxisValue, double zAxisValue, AHRS ahrs, boolean debug) {
        
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
}