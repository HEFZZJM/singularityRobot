package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.structure.mechanisms.LinearMechanism;
import frc.lib.structure.mechanisms.SetPoint;
import frc.lib.structure.motors.KrakenSimIO;
import frc.lib.structure.motors.MotorConfig;
import frc.lib.structure.motors.MotorInputs;
import frc.lib.structure.physics.physicalProperties;

public class LinearMechanismTestSubsystem extends SubsystemBase {
    private final KrakenSimIO motorIO;
    private final LinearMechanism<?, ?, ?> linearMechanism;
    private final MotorConfig config;

    // Example linear stage: vertical elevator (Z is up)
    private static final double CARRIAGE_MASS = 5.0; // kg
    private static final double AXIS_LENGTH = 1.0; // m (for logging only)

    public LinearMechanismTestSubsystem() {
        // Motor config
        config = new MotorConfig("ElevatorTestMotor", 3, "rio");
        config.kP = 4.0;
        config.kI = 0.0;
        config.kD = 1.0;
        config.maxVel = 1.5; // m/s
        config.maxAcc = 3.0; // m/s^2
        config.gearRatio = 12.0;
        config.kT = 0; // scale FF->current in sim path
        config.reversed = false;
        config.isBreak = true;
        config.supplyCurrentLimitEnabled = true;
        config.supplyCurrentLimit = 40.0;
        config.statorCurrentLimitEnabled = true;
        config.statorCurrentLimit = 40.0;
        config.updateFrequency = 100;
        config.isInnerSyncronized = true;

        motorIO = new KrakenSimIO(config);

        // Physical props: mass, CG at half of travel for simplicity, no rotational inertia
        org.ejml.simple.SimpleMatrix cg = new org.ejml.simple.SimpleMatrix(3, 1);
        cg.set(0, 0, 0.0);
        cg.set(1, 0, 0.0);
        cg.set(2, 0, AXIS_LENGTH / 2.0);

        org.ejml.simple.SimpleMatrix moi = new org.ejml.simple.SimpleMatrix(3, 3);
        moi.zero();

        physicalProperties physics =
                new physicalProperties(CARRIAGE_MASS, cg, moi, java.util.Optional.empty());

        // Axis: world Z axis (vertical)
        org.ejml.simple.SimpleMatrix axis = new org.ejml.simple.SimpleMatrix(3, 1);
        axis.set(0, 0, 0.0);
        axis.set(1, 0, 0.0);
        axis.set(2, 0, 1.0);

        // Start point at origin
        org.ejml.simple.SimpleMatrix startPoint = new org.ejml.simple.SimpleMatrix(3, 1);
        startPoint.zero();

        linearMechanism = new LinearMechanism<>("TestElevator", physics, axis, startPoint);
        linearMechanism.registerMotor(motorIO, config);

        Logger.recordOutput("LinearMechanismTest/Setup/Mass", CARRIAGE_MASS);
        Logger.recordOutput("LinearMechanismTest/Setup/AxesLength", AXIS_LENGTH);
        Logger.recordOutput("LinearMechanismTest/Setup/GearRatio", config.gearRatio);
    }

    @Override
    public void periodic() {
        linearMechanism.updateMechanismState();
        linearMechanism.executeControl();

        Logger.recordOutput("LinearMechanismTest/Mechanism/Position",
                linearMechanism.getCurrentPosition());
        Logger.recordOutput("LinearMechanismTest/Mechanism/Velocity",
                linearMechanism.getVelocity());
        Logger.recordOutput("LinearMechanismTest/Mechanism/Acceleration",
                linearMechanism.getAcceleration());
        Logger.recordOutput("LinearMechanismTest/Mechanism/AtTarget", linearMechanism.isAtTarget());

        if (linearMechanism.getMotorCount() > 0) {
            MotorInputs mi = linearMechanism.getMotorInputs(0);
            Logger.recordOutput("LinearMechanismTest/Motor/Position", mi.position);
            Logger.recordOutput("LinearMechanismTest/Motor/Velocity", mi.velocity);
            Logger.recordOutput("LinearMechanismTest/Motor/Current", mi.current);
            Logger.recordOutput("LinearMechanismTest/Motor/Temperature", mi.temperature);
            // Compare setpoint and measurement
            var sp = linearMechanism.getCurrentSetpoint();
            if (sp != null) {
                Logger.recordOutput("LinearMechanismTest/Control/PositionError",
                        sp.position - mi.position);
            }
        }
    }

    public void setTarget(double position, double velocity, double acceleration) {
        SetPoint sp = new SetPoint(position, velocity, acceleration, Double.NaN);
        linearMechanism.setTargetSetpoint(sp);
        Logger.recordOutput("LinearMechanismTest/Setpoint/Position", position);
        Logger.recordOutput("LinearMechanismTest/Setpoint/Velocity", velocity);
        Logger.recordOutput("LinearMechanismTest/Setpoint/Acceleration", acceleration);
    }

    public double getPosition() {
        return linearMechanism.getCurrentPosition();
    }

    public double getVelocity() {
        return linearMechanism.getVelocity();
    }

    public boolean isAtTarget() {
        return linearMechanism.isAtTarget();
    }

    public void stop() {
        linearMechanism.emergencyStop();
    }
}


