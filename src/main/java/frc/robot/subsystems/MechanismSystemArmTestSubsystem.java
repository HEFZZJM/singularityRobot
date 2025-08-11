package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.structure.mechanisms.MechanismSystem;
import frc.lib.structure.mechanisms.RotatingMechanism;
import frc.lib.structure.mechanisms.SetPoint;
import frc.lib.structure.motors.KrakenSimIO;
import frc.lib.structure.motors.MotorConfig;
import frc.lib.structure.motors.MotorInputs;
import frc.lib.structure.physics.physicalProperties;

public class MechanismSystemArmTestSubsystem extends SubsystemBase {
    private final MechanismSystem system;
    private final RotatingMechanism<?, ?, ?> arm;
    private final KrakenSimIO motorIO;
    private final MotorConfig motorConfig;
    private SetPoint currentSetpoint;

    private static final String ARM_NAME = "Arm";

    public MechanismSystemArmTestSubsystem() {
        // Motor configuration
        motorConfig = new MotorConfig("SysArmMotor", 4, "rio");
        motorConfig.kP = 5.0;
        motorConfig.kI = 0.0;
        motorConfig.kD = 2.0;
        motorConfig.maxVel = 10.0; // rad/s
        motorConfig.maxAcc = 20.0; // rad/s^2
        motorConfig.gearRatio = 10.0;
        motorConfig.kT = 0; // map torque FF to current in sim
        motorConfig.reversed = false;
        motorConfig.isBreak = true;
        motorConfig.supplyCurrentLimitEnabled = true;
        motorConfig.supplyCurrentLimit = 40.0;
        motorConfig.statorCurrentLimitEnabled = true;
        motorConfig.statorCurrentLimit = 40.0;
        motorConfig.updateFrequency = 100;
        motorConfig.isInnerSyncronized = true;

        motorIO = new KrakenSimIO(motorConfig);

        // Physical properties: arm rotates about Y-axis (planar XZ)
        double armLength = 0.5; // m
        double mass = 2.0; // kg
        double moiY = mass * armLength * armLength / 3.0;

        SimpleMatrix cg = new SimpleMatrix(3, 1);
        cg.set(0, 0, armLength / 2.0);
        cg.set(1, 0, 0.0);
        cg.set(2, 0, 0.0);

        SimpleMatrix moi = new SimpleMatrix(3, 3);
        moi.zero();
        moi.set(1, 1, moiY);

        physicalProperties physics =
                new physicalProperties(mass, cg, moi, java.util.Optional.empty());

        // Rotation axis = Y
        SimpleMatrix rotAxis = new SimpleMatrix(3, 1);
        rotAxis.set(0, 0, 0.0);
        rotAxis.set(1, 0, 1.0);
        rotAxis.set(2, 0, 0.0);

        SimpleMatrix pivot = new SimpleMatrix(3, 1);
        pivot.zero();

        arm = new RotatingMechanism<>(ARM_NAME, physics, rotAxis, pivot);
        arm.registerMotor(motorIO, motorConfig);

        // Mechanism system with single arm
        system = new MechanismSystem("SingleArmSystem");
        system.addMechanism(arm);

        Logger.recordOutput("MSArmTest/Setup/ArmLength", armLength);
        Logger.recordOutput("MSArmTest/Setup/Mass", mass);
        Logger.recordOutput("MSArmTest/Setup/MOI_Y", moiY);
        Logger.recordOutput("MSArmTest/Setup/GearRatio", motorConfig.gearRatio);
    }

    @Override
    public void periodic() {
        // Update motor feedback for logging
        arm.updateMechanismState();

        if (currentSetpoint != null) {
            // Build setpoint map and delegate to MechanismSystem for FF + motor outputs
            Map<String, SetPoint> sps = new HashMap<>();
            sps.put(ARM_NAME, currentSetpoint);
            system.applySetpointsWithSystemFeedforward(sps);

            // Optional logs for visibility
            Map<String, SimpleMatrix> ffMap = system.calculateSystemFeedforward(sps);
            SimpleMatrix armFF = ffMap.getOrDefault(ARM_NAME, new SimpleMatrix(3, 1));
            SimpleMatrix axisUnit = normalized(arm.getRotationAxis());
            double axisTorque = axisUnit.get(0, 0) * armFF.get(0, 0)
                    + axisUnit.get(1, 0) * armFF.get(1, 0) + axisUnit.get(2, 0) * armFF.get(2, 0);
            Logger.recordOutput("MSArmTest/FF/Vec/X", armFF.get(0, 0));
            Logger.recordOutput("MSArmTest/FF/Vec/Y", armFF.get(1, 0));
            Logger.recordOutput("MSArmTest/FF/Vec/Z", armFF.get(2, 0));
            Logger.recordOutput("MSArmTest/FF/AxisTorque", axisTorque);
            Logger.recordOutput("MSArmTest/Setpoint/Pos", currentSetpoint.position);
            Logger.recordOutput("MSArmTest/Setpoint/Vel", currentSetpoint.velocity);
            Logger.recordOutput("MSArmTest/Setpoint/Acc", currentSetpoint.acceleration);
        }

        // Log motor state
        if (arm.getMotorCount() > 0) {
            MotorInputs mi = arm.getMotorInputs(0);
            Logger.recordOutput("MSArmTest/Motor/Position", mi.position);
            Logger.recordOutput("MSArmTest/Motor/Velocity", mi.velocity);
            Logger.recordOutput("MSArmTest/Motor/Current", mi.current);
            Logger.recordOutput("MSArmTest/Motor/Temperature", mi.temperature);
        }
    }

    public void setTargetAngle(double angle, double velocity, double acceleration) {
        currentSetpoint = new SetPoint(angle, velocity, acceleration, Double.NaN);
    }

    // Follow a SetpointGroup snapshot at time t by delegating to MechanismSystem
    public void followGroupAtTime(frc.lib.structure.mechanisms.SetpointGroup group,
            double timeSec) {
        system.followSetpointGroupAtTime(group, timeSec);
    }

    public double getAngle() {
        return arm.getCurrentAngle();
    }

    public double getAngularVelocity() {
        return arm.getAngularVelocity();
    }

    public boolean isAtTarget() {
        return arm.isAtTarget();
    }

    public void stop() {
        arm.emergencyStop();
    }

    private static SimpleMatrix normalized(SimpleMatrix v) {
        double x = v.get(0, 0), y = v.get(1, 0), z = v.get(2, 0);
        double n = Math.sqrt(x * x + y * y + z * z);
        if (n == 0.0)
            return new SimpleMatrix(3, 1, true, 0.0, 0.0, 1.0);
        return new SimpleMatrix(3, 1, true, x / n, y / n, z / n);
    }
}


