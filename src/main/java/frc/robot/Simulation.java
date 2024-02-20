package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.REVPhysicsSim;

public class Simulation {
    /**
    * Initializes and updates the robot's component simulations for development and testing.
     */

    private DriveTrain driveTrain;
    public Simulation(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
    }

    public void init() {
        // Initialize the simulation environment here
        REVPhysicsSim.getInstance().addSparkMax(driveTrain.getLeftFrontMotor(), DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(driveTrain.getLeftBackMotor(), DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(driveTrain.getRightFrontMotor(), DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(driveTrain.getRightBackMotor(), DCMotor.getNEO(1));

        // Additional initialization as needed
    }

    public void periodic() {
        // Update the simulation state here
        REVPhysicsSim.getInstance().run();
        SmartDashboard.putNumber("Simluated Left Front Motor Power", driveTrain.getLeftFrontMotor().get());
        SmartDashboard.putNumber("Simluated Left Back Motor Power", driveTrain.getLeftBackMotor().get());
        SmartDashboard.putNumber("Simluated Right Front Motor Power", driveTrain.getRightFrontMotor().get());
        SmartDashboard.putNumber("Simluated Right Back Motor Power", driveTrain.getRightBackMotor().get());
    }
}