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
 * Linear mechanism class for describing linear motion structures like elevators Extends Mechanism,
 * specialized for linear motion
 */
public class LinearMechanism<Tconfig extends configBase, Tinputs extends inputBase, Trequest extends requestBase>
        extends Mechanism<Tconfig, Tinputs, Trequest> {

    private double currentPosition; // Current position (meters)
    private double velocity; // Velocity (m/s)
    private double acceleration; // Acceleration (m/s²)

    // Motion axis information
    private SimpleMatrix motionAxis; // Motion axis vector
    private SimpleMatrix startPoint; // Starting point

    public LinearMechanism(String name, physicalProperties properties, SimpleMatrix motionAxis,
            SimpleMatrix startPoint) {
        super(name, properties);
        this.motionAxis = motionAxis;
        this.startPoint = startPoint;
        this.currentPosition = 0.0;
        this.velocity = 0.0;
        this.acceleration = 0.0;
    }

    /**
     * Set current position
     * 
     * @param position Position (meters)
     */
    public void setCurrentPosition(double position) {
        this.currentPosition = position;
    }

    /**
     * Set velocity
     * 
     * @param velocity Velocity (m/s)
     */
    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    /**
     * Set acceleration
     * 
     * @param acceleration Acceleration (m/s²)
     */
    public void setAcceleration(double acceleration) {
        this.acceleration = acceleration;
    }

    /**
     * Get current position
     * 
     * @return Current position (meters)
     */
    public double getCurrentPosition() {
        return currentPosition;
    }

    /**
     * Get velocity
     * 
     * @return Velocity (m/s)
     */
    public double getVelocity() {
        return velocity;
    }

    /**
     * Get acceleration
     * 
     * @return Acceleration (m/s²)
     */
    public double getAcceleration() {
        return acceleration;
    }

    /**
     * Get motion axis
     * 
     * @return Motion axis vector
     */
    public SimpleMatrix getMotionAxis() {
        return motionAxis;
    }

    /**
     * Get starting point
     * 
     * @return Starting point
     */
    public SimpleMatrix getStartPoint() {
        return startPoint;
    }

    /**
     * Get current center of mass position
     * 
     * @return Center of mass position
     */
    public SimpleMatrix getCurrentCenterOfMass() {
        // Center of mass = start point + current position * motion axis direction
        return startPoint.plus(motionAxis.scale(currentPosition));
    }

    @Override
    public void setConfig(Tconfig config) {
        // Implement linear mechanism specific configuration
        super.setConfig(config);
    }

    @Override
    protected void updateStateFromMotorInputs() {
        // Update linear mechanism state from motor inputs (average of all motors)
        if (getMotorCount() > 0) {
            double avgPosition = 0.0;
            double avgVelocity = 0.0;
            double avgAcceleration = 0.0;

            for (MotorInputs inputs : getMotorInputs()) {
                avgPosition += inputs.position;
                avgVelocity += inputs.velocity;
                avgAcceleration += inputs.acceleration;
            }

            avgPosition /= getMotorCount();
            avgVelocity /= getMotorCount();
            avgAcceleration /= getMotorCount();

            setCurrentPosition(avgPosition);
            setVelocity(avgVelocity);
            setAcceleration(avgAcceleration);
        }
    }

    /**
     * Calculate feedforward force for linear mechanism Includes gravity, inertia, friction, etc.
     */
    @Override
    public SimpleMatrix getFeedforward(SimpleMatrix noninertialFrame) {
        // Prefer target setpoint for FF
        double ffPosition = (currentSetpoint != null) ? currentSetpoint.position : currentPosition;
        double ffVelocity = (currentSetpoint != null) ? currentSetpoint.velocity : velocity;
        double ffAcceleration =
                (currentSetpoint != null) ? currentSetpoint.acceleration : acceleration;

        // Calculate components
        // Effective gravity along axis uses geff = g - a_base (noninertialFrame encodes a_base)
        SimpleMatrix gravity = calculateGravityAlongAxis(noninertialFrame);
        SimpleMatrix inertia = calculateInertiaAlongAxis(ffAcceleration);
        SimpleMatrix friction = calculateFrictionAlongAxis(ffVelocity);

        // Log inputs and components
        String base = "FF/" + getName();
        Logger.recordOutput(base + "/Inputs/Position", ffPosition);
        Logger.recordOutput(base + "/Inputs/Velocity", ffVelocity);
        Logger.recordOutput(base + "/Inputs/Acceleration", ffAcceleration);

        Logger.recordOutput(base + "/Gravity/X", gravity.get(0, 0));
        Logger.recordOutput(base + "/Gravity/Y", gravity.get(1, 0));
        Logger.recordOutput(base + "/Gravity/Z", gravity.get(2, 0));

        Logger.recordOutput(base + "/Inertia/X", inertia.get(0, 0));
        Logger.recordOutput(base + "/Inertia/Y", inertia.get(1, 0));
        Logger.recordOutput(base + "/Inertia/Z", inertia.get(2, 0));

        Logger.recordOutput(base + "/Friction/X", friction.get(0, 0));
        Logger.recordOutput(base + "/Friction/Y", friction.get(1, 0));
        Logger.recordOutput(base + "/Friction/Z", friction.get(2, 0));

        SimpleMatrix total = gravity.plus(inertia).plus(friction);
        Logger.recordOutput(base + "/Total/X", total.get(0, 0));
        Logger.recordOutput(base + "/Total/Y", total.get(1, 0));
        Logger.recordOutput(base + "/Total/Z", total.get(2, 0));
        return total;
    }

    /**
     * Calculate gravity
     */
    private SimpleMatrix calculateGravityAlongAxis(SimpleMatrix noninertialFrame) {
        // Effective gravity geff = g - a_base; weight = m * geff
        double ax = (noninertialFrame != null && noninertialFrame.getNumRows() >= 3)
                ? noninertialFrame.get(0, 0)
                : 0.0;
        double ay = (noninertialFrame != null && noninertialFrame.getNumRows() >= 3)
                ? noninertialFrame.get(1, 0)
                : 0.0;
        double az = (noninertialFrame != null && noninertialFrame.getNumRows() >= 3)
                ? noninertialFrame.get(2, 0)
                : 0.0;
        SimpleMatrix geff = new SimpleMatrix(3, 1, true, -ax, -ay, -9.81 - az);
        SimpleMatrix weight = geff.scale(getPhysicalProperties().mass);
        SimpleMatrix axisUnit = normalized(motionAxis);
        // Project weight onto motion axis: F_axis = (w·axis) axis
        double comp = weight.get(0, 0) * axisUnit.get(0, 0) + weight.get(1, 0) * axisUnit.get(1, 0)
                + weight.get(2, 0) * axisUnit.get(2, 0);
        return axisUnit.scale(comp);
    }

    /**
     * Calculate inertia
     */
    private SimpleMatrix calculateInertiaAlongAxis(double accel) {
        // F = m * a along axis
        SimpleMatrix axisUnit = normalized(motionAxis);
        return axisUnit.scale(getPhysicalProperties().mass * accel);
    }

    /**
     * Calculate friction (simplified model)
     */
    private SimpleMatrix calculateFrictionAlongAxis(double vel) {
        // Viscous friction proportional to speed along axis
        double frictionCoefficient = 0.1;
        double frictionMagnitude = frictionCoefficient * Math.abs(vel);
        SimpleMatrix axisUnit = normalized(motionAxis);
        if (vel > 0) {
            return axisUnit.scale(-frictionMagnitude);
        } else if (vel < 0) {
            return axisUnit.scale(frictionMagnitude);
        } else {
            return new SimpleMatrix(3, 1, true, 0.0, 0.0, 0.0);
        }
    }

    private SimpleMatrix normalized(SimpleMatrix v) {
        double x = v.get(0, 0);
        double y = v.get(1, 0);
        double z = v.get(2, 0);
        double n = Math.sqrt(x * x + y * y + z * z);
        if (n == 0.0) {
            return new SimpleMatrix(3, 1, true, 0.0, 0.0, 1.0);
        }
        return new SimpleMatrix(3, 1, true, x / n, y / n, z / n);
    }

    /**
     * Override feedforward distribution for linear mechanisms Consider gear ratios and motor
     * efficiency for better distribution
     */
    @Override
    protected List<Double> distributeFeedforwardAmongMotors(SimpleMatrix totalFeedforward) {
        List<Double> motorFeedforwards = new ArrayList<>();

        if (getMotorCount() == 0) {
            return motorFeedforwards;
        }

        // Project total force onto motion axis to get signed scalar force
        SimpleMatrix axisUnit = normalized(motionAxis);
        double axisForce = 0.0;
        if (totalFeedforward != null && totalFeedforward.getNumRows() >= 3) {
            axisForce = axisUnit.get(0, 0) * totalFeedforward.get(0, 0)
                    + axisUnit.get(1, 0) * totalFeedforward.get(1, 0)
                    + axisUnit.get(2, 0) * totalFeedforward.get(2, 0);
        }
        Logger.recordOutput("FF/" + getName() + "/AxisForce", axisForce);

        // Calculate total gear ratio and efficiency
        double totalGearRatio = 0.0;

        for (MotorConfig config : motorConfigs) {
            totalGearRatio += config.gearRatio;
            // Efficiency is applied per-motor below; no need to sum here
        }

        // Distribute based on gear ratio and efficiency
        for (int i = 0; i < getMotorCount(); i++) {
            MotorConfig config = motorConfigs.get(i);
            double motorRatio = config.gearRatio / totalGearRatio;
            double motorEfficiency = 0.85; // Individual motor efficiency

            // Feedforward per motor: proportionally share required axis force
            double motorFeedforward = axisForce * motorRatio / motorEfficiency;
            motorFeedforwards.add(motorFeedforward);
            String motorName = config.name != null ? config.name : (getName() + "/motor" + i);
            Logger.recordOutput("FF/" + getName() + "/Motor/" + motorName + "/Feedforward",
                    motorFeedforward);
        }

        return motorFeedforwards;
    }
}
