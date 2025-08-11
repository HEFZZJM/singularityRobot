package frc.lib.structure.mechanisms;

import java.util.ArrayList;
import java.util.List;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;
import frc.lib.structure.configBase;
import frc.lib.structure.inputBase;
import frc.lib.structure.requestBase;
import frc.lib.structure.motors.MotorConfig;
import frc.lib.structure.motors.MotorInputs;
import frc.lib.structure.physics.physicalProperties;

/**
 * Rotating mechanism class for describing rotating structures like arms Extends Mechanism,
 * specialized for rotating motion
 */
public class RotatingMechanism<Tconfig extends configBase, Tinputs extends inputBase, Trequest extends requestBase>
        extends Mechanism<Tconfig, Tinputs, Trequest> {

    private double currentAngle; // Current angle (radians)
    private double angularVelocity; // Angular velocity (rad/s)
    private double angularAcceleration; // Angular acceleration (rad/s²)

    // Rotation axis information
    private SimpleMatrix rotationAxis; // Rotation axis vector
    private SimpleMatrix pivotPoint; // Rotation center point

    public RotatingMechanism(String name, physicalProperties properties, SimpleMatrix rotationAxis,
            SimpleMatrix pivotPoint) {
        super(name, properties);
        this.rotationAxis = rotationAxis;
        this.pivotPoint = pivotPoint;
        this.currentAngle = 0.0;
        this.angularVelocity = 0.0;
        this.angularAcceleration = 0.0;
    }

    /**
     * Set current angle
     * 
     * @param angle Angle (radians)
     */
    public void setCurrentAngle(double angle) {
        this.currentAngle = angle;
    }

    /**
     * Set angular velocity
     * 
     * @param angularVelocity Angular velocity (rad/s)
     */
    public void setAngularVelocity(double angularVelocity) {
        this.angularVelocity = angularVelocity;
    }

    /**
     * Set angular acceleration
     * 
     * @param angularAcceleration Angular acceleration (rad/s²)
     */
    public void setAngularAcceleration(double angularAcceleration) {
        this.angularAcceleration = angularAcceleration;
    }

    /**
     * Get current angle
     * 
     * @return Current angle (radians)
     */
    public double getCurrentAngle() {
        return currentAngle;
    }

    /**
     * Get angular velocity
     * 
     * @return Angular velocity (rad/s)
     */
    public double getAngularVelocity() {
        return angularVelocity;
    }

    /**
     * Get angular acceleration
     * 
     * @return Angular acceleration (rad/s²)
     */
    public double getAngularAcceleration() {
        return angularAcceleration;
    }

    /**
     * Get rotation axis
     * 
     * @return Rotation axis vector
     */
    public SimpleMatrix getRotationAxis() {
        return rotationAxis;
    }

    /**
     * Get rotation center point
     * 
     * @return Rotation center point
     */
    public SimpleMatrix getPivotPoint() {
        return pivotPoint;
    }

    @Override
    public void setConfig(Tconfig config) {
        // Implement rotating mechanism specific configuration
        super.setConfig(config);
    }

    @Override
    protected void updateStateFromMotorInputs() {
        // Update rotating mechanism state from motor inputs (average of all motors)
        if (getMotorCount() > 0) {
            double avgAngle = 0.0;
            double avgAngularVelocity = 0.0;
            double avgAngularAcceleration = 0.0;

            for (MotorInputs inputs : getMotorInputs()) {
                avgAngle += inputs.position;
                avgAngularVelocity += inputs.velocity;
                avgAngularAcceleration += inputs.acceleration;
            }

            avgAngle /= getMotorCount();
            avgAngularVelocity /= getMotorCount();
            avgAngularAcceleration /= getMotorCount();

            setCurrentAngle(avgAngle);
            setAngularVelocity(avgAngularVelocity);
            setAngularAcceleration(avgAngularAcceleration);
        }
    }

    /**
     * Calculate feedforward torque for rotating mechanism Includes gravity torque, inertia torque,
     * etc.
     */
    @Override
    public SimpleMatrix getFeedforward(SimpleMatrix noninertialFrame) {
        // Prefer target setpoint values for feedforward to reflect desired motion
        double ffAngle = (currentSetpoint != null) ? currentSetpoint.position : currentAngle;
        double ffAngularVelocity =
                (currentSetpoint != null) ? currentSetpoint.velocity : angularVelocity;
        double ffAngularAcceleration =
                (currentSetpoint != null) ? currentSetpoint.acceleration : angularAcceleration;

        // Calculate gravity torque (depends on angle) with effective gravity g - a_noninertial
        SimpleMatrix gravityTorque = calculateGravityTorque(ffAngle, noninertialFrame);

        // Calculate inertia torque (depends on angular acceleration)
        SimpleMatrix inertiaTorque = calculateInertiaTorque(ffAngularAcceleration);

        // Calculate coriolis/gyroscopic torque (depends on angular velocity)
        SimpleMatrix coriolisTorque = calculateCoriolisTorque(ffAngularVelocity);

        // Log inputs and components
        String base = "FF/" + getName();
        Logger.recordOutput(base + "/Inputs/Angle", ffAngle);
        Logger.recordOutput(base + "/Inputs/AngularVelocity", ffAngularVelocity);
        Logger.recordOutput(base + "/Inputs/AngularAcceleration", ffAngularAcceleration);

        Logger.recordOutput(base + "/Gravity/X", gravityTorque.get(0, 0));
        Logger.recordOutput(base + "/Gravity/Y", gravityTorque.get(1, 0));
        Logger.recordOutput(base + "/Gravity/Z", gravityTorque.get(2, 0));

        Logger.recordOutput(base + "/Inertia/X", inertiaTorque.get(0, 0));
        Logger.recordOutput(base + "/Inertia/Y", inertiaTorque.get(1, 0));
        Logger.recordOutput(base + "/Inertia/Z", inertiaTorque.get(2, 0));

        Logger.recordOutput(base + "/Gyro/X", coriolisTorque.get(0, 0));
        Logger.recordOutput(base + "/Gyro/Y", coriolisTorque.get(1, 0));
        Logger.recordOutput(base + "/Gyro/Z", coriolisTorque.get(2, 0));

        // Return total feedforward torque vector (3D)
        SimpleMatrix total = gravityTorque.plus(inertiaTorque).plus(coriolisTorque);
        Logger.recordOutput(base + "/Total/X", total.get(0, 0));
        Logger.recordOutput(base + "/Total/Y", total.get(1, 0));
        Logger.recordOutput(base + "/Total/Z", total.get(2, 0));
        return total;
    }

    /**
     * Calculate gravity torque
     */
    private SimpleMatrix calculateGravityTorque(double angleRadians,
            SimpleMatrix nonInertialAccel) {
        // Local CG relative to pivot at zero angle
        SimpleMatrix rLocal = getPhysicalProperties().CG.minus(pivotPoint);

        // Normalize rotation axis
        SimpleMatrix axisUnit = normalized(rotationAxis);

        // Rotate rLocal around axis by current/target angle to world frame
        SimpleMatrix rWorld = rotateAroundAxis(rLocal, axisUnit, angleRadians);

        // Effective gravity vector (world): geff = g - a_base (nonInertialAccel encodes a_base)
        double ax = (nonInertialAccel != null && nonInertialAccel.getNumRows() >= 3)
                ? nonInertialAccel.get(0, 0)
                : 0.0;
        double ay = (nonInertialAccel != null && nonInertialAccel.getNumRows() >= 3)
                ? nonInertialAccel.get(1, 0)
                : 0.0;
        double az = (nonInertialAccel != null && nonInertialAccel.getNumRows() >= 3)
                ? nonInertialAccel.get(2, 0)
                : 0.0;
        // g = (0,0,-9.81). geff = ( -ax, -ay, -9.81 - az )
        SimpleMatrix geff = new SimpleMatrix(3, 1, true, -ax, -ay, -9.81 - az);
        SimpleMatrix gravity = geff.scale(getPhysicalProperties().mass);

        // Torque due to gravity: τ = r × F
        return crossProduct(rWorld, gravity);
    }

    /**
     * Calculate inertia torque
     */
    private SimpleMatrix calculateInertiaTorque(double angularAccelerationRadPerSec2) {
        // τ = I * α (α along rotation axis)
        SimpleMatrix moi = getPhysicalProperties().MOI;
        SimpleMatrix axisUnit = normalized(rotationAxis);
        SimpleMatrix alpha = axisUnit.scale(angularAccelerationRadPerSec2);
        return moi.mult(alpha);
    }

    /**
     * Calculate coriolis torque
     */
    private SimpleMatrix calculateCoriolisTorque(double angularVelocityRadPerSec) {
        // Gyroscopic torque = ω × (I * ω)
        SimpleMatrix moi = getPhysicalProperties().MOI;
        SimpleMatrix axisUnit = normalized(rotationAxis);
        SimpleMatrix omega = axisUnit.scale(angularVelocityRadPerSec);
        SimpleMatrix Iw = moi.mult(omega);
        return crossProduct(omega, Iw);
    }

    /**
     * Vector cross product calculation
     */
    private SimpleMatrix crossProduct(SimpleMatrix a, SimpleMatrix b) {
        double ax = a.get(0, 0);
        double ay = a.get(1, 0);
        double az = a.get(2, 0);

        double bx = b.get(0, 0);
        double by = b.get(1, 0);
        double bz = b.get(2, 0);

        return new SimpleMatrix(3, 1, true, ay * bz - az * by, az * bx - ax * bz,
                ax * by - ay * bx);
    }

    private SimpleMatrix normalized(SimpleMatrix v) {
        double vx = v.get(0, 0);
        double vy = v.get(1, 0);
        double vz = v.get(2, 0);
        double norm = Math.sqrt(vx * vx + vy * vy + vz * vz);
        if (norm == 0.0) {
            return new SimpleMatrix(3, 1, true, 0.0, 0.0, 1.0);
        }
        return new SimpleMatrix(3, 1, true, vx / norm, vy / norm, vz / norm);
    }

    private SimpleMatrix rotateAroundAxis(SimpleMatrix v, SimpleMatrix axisUnit, double angle) {
        // Rodrigues' rotation formula: v_rot = v*cosθ + (k × v)*sinθ + k*(k·v)*(1-cosθ)
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);

        SimpleMatrix term1 = v.scale(cos);
        SimpleMatrix kxv = crossProduct(axisUnit, v);
        SimpleMatrix term2 = kxv.scale(sin);
        double kdotv = axisUnit.get(0, 0) * v.get(0, 0) + axisUnit.get(1, 0) * v.get(1, 0)
                + axisUnit.get(2, 0) * v.get(2, 0);
        SimpleMatrix term3 = axisUnit.scale(kdotv * (1.0 - cos));
        return term1.plus(term2).plus(term3);
    }

    /**
     * Override feedforward distribution for rotating mechanisms Consider gear ratios and motor
     * efficiency for better distribution
     */
    @Override
    protected List<Double> distributeFeedforwardAmongMotors(SimpleMatrix totalFeedforward) {
        List<Double> motorFeedforwards = new ArrayList<>();

        if (getMotorCount() == 0) {
            return motorFeedforwards;
        }

        // Project torque vector onto rotation axis to get signed scalar torque
        SimpleMatrix axisUnit = normalized(rotationAxis);
        double axisTorque = 0.0;
        if (totalFeedforward != null && totalFeedforward.getNumRows() >= 3) {
            axisTorque = axisUnit.get(0, 0) * totalFeedforward.get(0, 0)
                    + axisUnit.get(1, 0) * totalFeedforward.get(1, 0)
                    + axisUnit.get(2, 0) * totalFeedforward.get(2, 0);
        }
        Logger.recordOutput("FF/" + getName() + "/AxisTorque", axisTorque);

        // Calculate total gear ratio and efficiency
        double totalGearRatio = 0.0;

        for (MotorConfig config : motorConfigs) {
            totalGearRatio += config.gearRatio;
        }

        // Distribute torque based on gear ratio and efficiency
        for (int i = 0; i < getMotorCount(); i++) {
            MotorConfig config = motorConfigs.get(i);
            double motorRatio = config.gearRatio / totalGearRatio;
            double motorEfficiency = 0.85; // Individual motor efficiency

            // Torque = total torque * motor ratio / motor efficiency
            double motorTorque = axisTorque * motorRatio / motorEfficiency;
            motorFeedforwards.add(motorTorque);
            String motorName = config.name != null ? config.name : (getName() + "/motor" + i);
            Logger.recordOutput("FF/" + getName() + "/Motor/" + motorName + "/Feedforward",
                    motorTorque);
        }

        return motorFeedforwards;
    }
}
