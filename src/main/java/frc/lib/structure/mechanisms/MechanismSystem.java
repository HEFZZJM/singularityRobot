package frc.lib.structure.mechanisms;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;
import frc.lib.structure.configBase;
import frc.lib.structure.inputBase;
import frc.lib.structure.requestBase;
import frc.lib.structure.motors.MotorRequest;

/**
 * Mechanism system manager Manages physical relationships between multiple mechanisms, builds tree
 * structure Calculates feedforward forces for the entire system considering parent-child
 * interactions
 */
public class MechanismSystem {

    // Mechanism mapping: name -> mechanism object
    private Map<String, Mechanism<? extends configBase, ? extends inputBase, ? extends requestBase>> mechanisms;

    // Parent-child relationship mapping: child mechanism -> parent mechanism
    private Map<String, String> parentChildRelations;

    // Child mechanisms mapping: parent mechanism -> list of children
    private Map<String, List<String>> childrenMap;

    // System root node
    private String rootMechanism;

    // System name
    private String systemName;

    public MechanismSystem(String systemName) {
        this.systemName = systemName;
        this.mechanisms = new ConcurrentHashMap<>();
        this.parentChildRelations = new ConcurrentHashMap<>();
        this.childrenMap = new ConcurrentHashMap<>();
        this.rootMechanism = null;
    }

    /**
     * Add mechanism to system
     */
    public void addMechanism(
            Mechanism<? extends configBase, ? extends inputBase, ? extends requestBase> mechanism) {
        mechanisms.put(mechanism.getName(), mechanism);

        // If it's the first mechanism, set as root node
        if (rootMechanism == null) {
            rootMechanism = mechanism.getName();
        }
    }

    /**
     * Set parent-child relationship
     */
    public void setParentChildRelation(String parentName, String childName) {
        if (!mechanisms.containsKey(parentName) || !mechanisms.containsKey(childName)) {
            throw new IllegalArgumentException("Parent or child mechanism not found");
        }

        parentChildRelations.put(childName, parentName);

        // Update children map
        childrenMap.computeIfAbsent(parentName, k -> new ArrayList<>()).add(childName);
    }

    /**
     * Get mechanism
     */
    public Mechanism<? extends configBase, ? extends inputBase, ? extends requestBase> getMechanism(
            String name) {
        return mechanisms.get(name);
    }

    /**
     * Get all mechanisms
     */
    public Map<String, Mechanism<? extends configBase, ? extends inputBase, ? extends requestBase>> getAllMechanisms() {
        return new HashMap<>(mechanisms);
    }

    /**
     * Get child mechanisms
     */
    public List<String> getChildren(String parentName) {
        return childrenMap.getOrDefault(parentName, new ArrayList<>());
    }

    /**
     * Get parent mechanism
     */
    public String getParent(String childName) {
        return parentChildRelations.get(childName);
    }

    /**
     * Update all mechanism states from motor feedback
     */
    public void updateAllMechanismStates() {
        for (Mechanism<? extends configBase, ? extends inputBase, ? extends requestBase> mechanism : mechanisms
                .values()) {
            mechanism.updateMechanismState();
        }
    }

    /**
     * Execute control for all mechanisms
     */
    public void executeAllMechanismControl() {
        for (Mechanism<? extends configBase, ? extends inputBase, ? extends requestBase> mechanism : mechanisms
                .values()) {
            mechanism.executeControl();
        }
    }

    /**
     * Set setpoints for multiple mechanisms
     */
    public void setMechanismSetpoints(Map<String, SetPoint> setpoints) {
        for (Map.Entry<String, SetPoint> entry : setpoints.entrySet()) {
            Mechanism<? extends configBase, ? extends inputBase, ? extends requestBase> mechanism =
                    mechanisms.get(entry.getKey());
            if (mechanism != null) {
                mechanism.setTargetSetpoint(entry.getValue());
            }
        }
    }

    /**
     * Check if all mechanisms are at target
     */
    public boolean areAllMechanismsAtTarget() {
        for (Mechanism<? extends configBase, ? extends inputBase, ? extends requestBase> mechanism : mechanisms
                .values()) {
            if (!mechanism.isAtTarget()) {
                return false;
            }
        }
        return true;
    }

    /**
     * Emergency stop all mechanisms
     */
    public void emergencyStopAllMechanisms() {
        for (Mechanism<? extends configBase, ? extends inputBase, ? extends requestBase> mechanism : mechanisms
                .values()) {
            mechanism.emergencyStop();
        }
    }

    /**
     * Calculate feedforward forces for the entire system considering parent-child interactions This
     * method implements true tree-based feedforward calculation
     */
    public Map<String, SimpleMatrix> calculateSystemFeedforward(Map<String, SetPoint> setpoints) {
        Map<String, SimpleMatrix> feedforwardForces = new HashMap<>();
        Map<String, MechanismState> mechanismStates = new HashMap<>();

        // First pass: Update all mechanism states based on setpoints
        updateAllMechanismStatesFromSetpoints(setpoints, mechanismStates);

        // Second pass: Calculate feedforward from root to leaves (top-down)
        calculateTopDownFeedforward(rootMechanism, setpoints, feedforwardForces, mechanismStates,
                new SimpleMatrix(3, 1));

        // Third pass: Calculate reaction forces from leaves to root (bottom-up)
        calculateBottomUpReactionForces(rootMechanism, feedforwardForces, mechanismStates);

        return feedforwardForces;
    }

    /**
     * Follow a SetpointGroup at a given time (seconds): - Selects the appropriate setpoint snapshot
     * for the given time - Sets each mechanism's setpoint - Computes system-level feedforward
     * (considering parent-child interactions) - Distributes and sends feedforward to motors with
     * the position/velocity/acceleration setpoints
     */
    public void followSetpointGroupAtTime(SetpointGroup group, double timeSeconds) {
        if (group == null || group.size() == 0) {
            return;
        }

        // Find index at or before timeSeconds
        int idx = 0;
        for (int i = 0; i < group.size(); i++) {
            if (group.getTimeAtIndex(i) <= timeSeconds) {
                idx = i;
            } else {
                break;
            }
        }

        Map<String, SetPoint> setpoints = group.getSetpointsAtIndex(idx);
        Logger.recordOutput("MSFF/" + systemName + "/PlaybackTime", timeSeconds);
        Logger.recordOutput("MSFF/" + systemName + "/PlaybackIndex", idx);
        applySetpointsWithSystemFeedforward(setpoints);
    }

    /**
     * Applies setpoints and sends system-level feedforward motor requests.
     */
    public void applySetpointsWithSystemFeedforward(Map<String, SetPoint> setpoints) {
        if (setpoints == null || setpoints.isEmpty())
            return;

        // Set setpoints to mechanisms
        setMechanismSetpoints(setpoints);

        // Calculate system FF
        Map<String, SimpleMatrix> ffForces = calculateSystemFeedforward(setpoints);

        // Send to motors per mechanism using each mechanism's distribution logic
        for (Map.Entry<String, SetPoint> entry : setpoints.entrySet()) {
            String mechName = entry.getKey();
            SetPoint sp = entry.getValue();
            Mechanism<? extends configBase, ? extends inputBase, ? extends requestBase> mech =
                    mechanisms.get(mechName);
            if (mech == null)
                continue;

            SimpleMatrix totalFF = ffForces.getOrDefault(mechName, new SimpleMatrix(3, 1));
            // Log total FF vector and axis projection
            String base = "MSFF/" + systemName + "/" + mechName;
            if (totalFF.getNumRows() >= 3) {
                Logger.recordOutput(base + "/Vec/X", totalFF.get(0, 0));
                Logger.recordOutput(base + "/Vec/Y", totalFF.get(1, 0));
                Logger.recordOutput(base + "/Vec/Z", totalFF.get(2, 0));
            }
            if (mech instanceof RotatingMechanism<?, ?, ?> rot) {
                SimpleMatrix axis = rot.getRotationAxis();
                SimpleMatrix axisU = normalizeAxis(axis);
                double axisTorque = dot3(totalFF, axisU);
                Logger.recordOutput(base + "/AxisTorque", axisTorque);
            } else if (mech instanceof LinearMechanism<?, ?, ?> lin) {
                SimpleMatrix axis = lin.getMotionAxis();
                SimpleMatrix axisU = normalizeAxis(axis);
                double axisForce = dot3(totalFF, axisU);
                Logger.recordOutput(base + "/AxisForce", axisForce);
            }
            // Use mechanism-specific distribution (gear ratios/efficiency) to split among motors
            List<Double> motorFFs = mech.distributeFeedforwardAmongMotors(totalFF);

            // Send to each motor
            for (int i = 0; i < mech.motorIOs.size(); i++) {
                double ff = (i < motorFFs.size()) ? motorFFs.get(i) : 0.0;
                MotorRequest req =
                        new MotorRequest().withPosition(sp.position).withVelocity(sp.velocity)
                                .withAcceleration(sp.acceleration).withFeedforward(ff);
                mech.motorIOs.get(i).set(req);
            }
        }
    }

    private static SimpleMatrix normalizeAxis(SimpleMatrix v) {
        double x = v.get(0, 0), y = v.get(1, 0), z = v.get(2, 0);
        double n = Math.sqrt(x * x + y * y + z * z);
        if (n == 0.0)
            return new SimpleMatrix(3, 1, true, 0.0, 0.0, 1.0);
        return new SimpleMatrix(3, 1, true, x / n, y / n, z / n);
    }

    private static double dot3(SimpleMatrix a, SimpleMatrix b) {
        return a.get(0, 0) * b.get(0, 0) + a.get(1, 0) * b.get(1, 0) + a.get(2, 0) * b.get(2, 0);
    }

    private static SimpleMatrix cross3(SimpleMatrix a, SimpleMatrix b) {
        double ax = a.get(0, 0), ay = a.get(1, 0), az = a.get(2, 0);
        double bx = b.get(0, 0), by = b.get(1, 0), bz = b.get(2, 0);
        return new SimpleMatrix(3, 1, true, ay * bz - az * by, az * bx - ax * bz,
                ax * by - ay * bx);
    }

    private SimpleMatrix getChildWorldCG(MechanismState childState) {
        if (childState.type == MechanismType.LINEAR) {
            LinearMechanism<?, ?, ?> lin = (LinearMechanism<?, ?, ?>) childState.mechanism;
            return lin.getCurrentCenterOfMass();
        } else {
            RotatingMechanism<?, ?, ?> rot = (RotatingMechanism<?, ?, ?>) childState.mechanism;
            SimpleMatrix rLocal = rot.getPhysicalProperties().CG.minus(rot.getPivotPoint());
            SimpleMatrix axisU = normalizeAxis(rot.getRotationAxis());
            // Rodrigues rotation to world by current angle
            double angle = childState.position;
            double c = Math.cos(angle), s = Math.sin(angle);
            SimpleMatrix term1 = rLocal.scale(c);
            SimpleMatrix kxv = cross3(axisU, rLocal);
            SimpleMatrix term2 = kxv.scale(s);
            double kdotv = dot3(axisU, rLocal);
            SimpleMatrix term3 = axisU.scale(kdotv * (1.0 - c));
            SimpleMatrix rWorld = term1.plus(term2).plus(term3);
            return rWorld.plus(rot.getPivotPoint());
        }
    }

    /**
     * Update all mechanism states based on setpoints
     */
    private void updateAllMechanismStatesFromSetpoints(Map<String, SetPoint> setpoints,
            Map<String, MechanismState> mechanismStates) {
        for (Map.Entry<String, SetPoint> entry : setpoints.entrySet()) {
            String mechanismName = entry.getKey();
            SetPoint setpoint = entry.getValue();
            Mechanism<? extends configBase, ? extends inputBase, ? extends requestBase> mechanism =
                    mechanisms.get(mechanismName);

            if (mechanism != null) {
                MechanismState state = new MechanismState();
                state.mechanism = mechanism;
                state.setpoint = setpoint;

                // Update mechanism state based on type
                if (mechanism instanceof LinearMechanism) {
                    LinearMechanism<?, ?, ?> linearMech = (LinearMechanism<?, ?, ?>) mechanism;
                    linearMech.setCurrentPosition(setpoint.position);
                    linearMech.setVelocity(setpoint.velocity);
                    linearMech.setAcceleration(setpoint.acceleration);

                    state.position = setpoint.position;
                    state.velocity = setpoint.velocity;
                    state.acceleration = setpoint.acceleration;
                    state.type = MechanismType.LINEAR;
                } else if (mechanism instanceof RotatingMechanism) {
                    RotatingMechanism<?, ?, ?> rotatingMech =
                            (RotatingMechanism<?, ?, ?>) mechanism;
                    rotatingMech.setCurrentAngle(setpoint.position);
                    rotatingMech.setAngularVelocity(setpoint.velocity);
                    rotatingMech.setAngularAcceleration(setpoint.acceleration);

                    state.position = setpoint.position;
                    state.velocity = setpoint.velocity;
                    state.acceleration = setpoint.acceleration;
                    state.type = MechanismType.ROTATING;
                }

                mechanismStates.put(mechanismName, state);
            }
        }
    }

    /**
     * Top-down feedforward calculation (parent motion affects children)
     */
    private void calculateTopDownFeedforward(String mechanismName, Map<String, SetPoint> setpoints,
            Map<String, SimpleMatrix> feedforwardForces,
            Map<String, MechanismState> mechanismStates, SimpleMatrix parentMotion) {

        Mechanism<? extends configBase, ? extends inputBase, ? extends requestBase> mechanism =
                mechanisms.get(mechanismName);

        if (mechanism == null) {
            return;
        }

        MechanismState state = mechanismStates.get(mechanismName);
        if (state == null) {
            return;
        }

        // Compose base linear acceleration (a_base) in world frame that this mechanism experiences
        // Start from parent's a_base, then add this mechanism的线性轴向加速度
        SimpleMatrix nonInertialAccel = parentMotion;
        if (state.type == MechanismType.LINEAR) {
            LinearMechanism<?, ?, ?> lin = (LinearMechanism<?, ?, ?>) state.mechanism;
            SimpleMatrix axisU = normalizeAxis(lin.getMotionAxis());
            // a_base(child) = a_base(parent) + a_self * axis
            nonInertialAccel = parentMotion.plus(axisU.scale(state.acceleration));
        }

        // Calculate base feedforward for this mechanism given current non-inertial acceleration
        SimpleMatrix baseFeedforward = mechanism.getFeedforward(nonInertialAccel);

        // Add parent motion effects to children, including rotational kinematics if parent rotates
        List<String> children = getChildren(mechanismName);
        for (String childName : children) {
            MechanismState childState = mechanismStates.get(childName);
            if (childState != null) {
                SimpleMatrix childAccel = nonInertialAccel;
                if (state.type == MechanismType.ROTATING) {
                    RotatingMechanism<?, ?, ?> rot = (RotatingMechanism<?, ?, ?>) state.mechanism;
                    SimpleMatrix axisU = normalizeAxis(rot.getRotationAxis());
                    // Parent angular kinematics
                    double w = state.velocity; // rad/s
                    double alpha = state.acceleration; // rad/s^2
                    SimpleMatrix omega = axisU.scale(w);
                    SimpleMatrix alphav = axisU.scale(alpha);
                    // Child CG position in world
                    SimpleMatrix childCG = getChildWorldCG(childState);
                    SimpleMatrix rc = childCG.minus(rot.getPivotPoint());
                    // Remove axial component (lever arm perpendicular to axis)
                    double proj = dot3(axisU, rc);
                    SimpleMatrix rcPerp = rc.minus(axisU.scale(proj));
                    // a_add = ω×(ω×r) + α×r
                    SimpleMatrix aCent = cross3(omega, cross3(omega, rcPerp));
                    SimpleMatrix aTan = cross3(alphav, rcPerp);
                    childAccel = childAccel.plus(aCent).plus(aTan);
                }

                calculateTopDownFeedforward(childName, setpoints, feedforwardForces,
                        mechanismStates, childAccel);
            }
        }

        // Store the base feedforward (will be modified in bottom-up pass)
        feedforwardForces.put(mechanismName, baseFeedforward);
    }

    /**
     * Bottom-up reaction force calculation (children affect parent)
     */
    private SimpleMatrix calculateBottomUpReactionForces(String mechanismName,
            Map<String, SimpleMatrix> feedforwardForces,
            Map<String, MechanismState> mechanismStates) {

        Mechanism<? extends configBase, ? extends inputBase, ? extends requestBase> mechanism =
                mechanisms.get(mechanismName);

        if (mechanism == null) {
            return new SimpleMatrix(3, 1);
        }

        MechanismState state = mechanismStates.get(mechanismName);
        if (state == null) {
            return new SimpleMatrix(3, 1);
        }

        // Get base feedforward for this mechanism
        SimpleMatrix totalFeedforward =
                feedforwardForces.getOrDefault(mechanismName, new SimpleMatrix(3, 1));

        // Calculate reaction forces from all children
        List<String> children = getChildren(mechanismName);
        SimpleMatrix totalReactionForce = new SimpleMatrix(3, 1);

        for (String childName : children) {
            MechanismState childState = mechanismStates.get(childName);
            if (childState != null) {
                // Recursively get child's total force (including its children's reactions)
                SimpleMatrix childTotalForce = calculateBottomUpReactionForces(childName,
                        feedforwardForces, mechanismStates);

                // Calculate reaction force that child exerts on parent
                SimpleMatrix reactionForce =
                        calculateReactionForce(state, childState, childTotalForce);
                totalReactionForce = totalReactionForce.plus(reactionForce);
            }
        }

        // Add reaction forces to this mechanism's feedforward
        totalFeedforward = totalFeedforward.plus(totalReactionForce);
        feedforwardForces.put(mechanismName, totalFeedforward);

        return totalFeedforward;
    }

    /**
     * Calculate how parent motion affects child motion
     */
    private SimpleMatrix calculateChildMotionEffect(MechanismState parentState,
            MechanismState childState, SimpleMatrix parentMotion) {

        // This is a simplified calculation - in reality, this would involve
        // proper kinematic transformations based on the physical connection

        if (parentState.type == MechanismType.LINEAR && childState.type == MechanismType.LINEAR) {
            // Pass through non-inertial acceleration
            return parentMotion;
        } else if (parentState.type == MechanismType.LINEAR
                && childState.type == MechanismType.ROTATING) {
            // Pass through non-inertial acceleration (child will use it)
            return parentMotion;
        } else if (parentState.type == MechanismType.ROTATING
                && childState.type == MechanismType.LINEAR) {
            // Rotating parent imparts a_add to linear child; already handled in top-down pass
            return parentMotion;
        } else {
            // Rotating parent affects rotating child: rely on a_add via top-down; no extra here
            return parentMotion;
        }
    }

    /**
     * Calculate reaction force that child exerts on parent
     */
    private SimpleMatrix calculateReactionForce(MechanismState parentState,
            MechanismState childState, SimpleMatrix childForce) {

        // This calculates the reaction force that a child mechanism exerts on its parent
        // based on Newton's third law and the physical connection geometry

        if (parentState.type == MechanismType.LINEAR && childState.type == MechanismType.LINEAR) {
            // Direct force transmission
            return childForce.scale(-1.0); // Reaction force is opposite
        } else if (parentState.type == MechanismType.LINEAR
                && childState.type == MechanismType.ROTATING) {
            // Project child's total force along parent's axis as reaction
            LinearMechanism<?, ?, ?> parentLin = (LinearMechanism<?, ?, ?>) parentState.mechanism;
            SimpleMatrix parentAxisU = normalizeAxis(parentLin.getMotionAxis());
            double comp = dot3(childForce, parentAxisU);
            return parentAxisU.scale(-comp);
        } else if (parentState.type == MechanismType.ROTATING
                && childState.type == MechanismType.LINEAR) {
            // Linear child exerts force on rotating parent: τ = r × F at parent's pivot
            RotatingMechanism<?, ?, ?> parentRot =
                    (RotatingMechanism<?, ?, ?>) parentState.mechanism;
            SimpleMatrix childCG = getChildWorldCG(childState);
            SimpleMatrix rc = childCG.minus(parentRot.getPivotPoint());
            SimpleMatrix torque = cross3(rc, childForce);
            return torque;
        } else {
            // Rotating child exerts torque on rotating parent: combine torques directly
            return childForce.scale(-1.0);
        }
    }

    /**
     * Get system name
     */
    public String getSystemName() {
        return systemName;
    }

    /**
     * Get root mechanism name
     */
    public String getRootMechanism() {
        return rootMechanism;
    }

    /**
     * Helper class to store mechanism state information
     */
    private static class MechanismState {
        Mechanism<? extends configBase, ? extends inputBase, ? extends requestBase> mechanism;
        SetPoint setpoint;
        double position;
        double velocity;
        double acceleration;
        MechanismType type;
    }

    /**
     * Mechanism type enumeration
     */
    private enum MechanismType {
        LINEAR, ROTATING
    }
}
