package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.structure.mechanisms.LinearMechanism;
import frc.lib.structure.mechanisms.MechanismSystem;
import frc.lib.structure.mechanisms.RotatingMechanism;
import frc.lib.structure.mechanisms.SetpointGroup;
import frc.lib.structure.motors.KrakenSimIO;
import frc.lib.structure.motors.MotorConfig;
import frc.lib.structure.motors.MotorInputs;
import frc.lib.structure.physics.physicalProperties;

public class ElevatorArmSystemTestSubsystem extends SubsystemBase {
    private final MechanismSystem system;
    private final LinearMechanism<?, ?, ?> elevator;
    private final RotatingMechanism<?, ?, ?> arm;

    private final KrakenSimIO elevatorMotor;
    private final KrakenSimIO armMotor;

    private final MotorConfig elevatorConfig;
    private final MotorConfig armConfig;

    private static final String ELEVATOR_NAME = "Elevator";
    private static final String ARM_NAME = "Arm";

    public ElevatorArmSystemTestSubsystem() {
        // Elevator motor & config
        elevatorConfig = new MotorConfig("ElevatorSysMotor", 5, "rio");
        elevatorConfig.kP = 4.0;
        elevatorConfig.kI = 0.0;
        elevatorConfig.kD = 1.0;
        elevatorConfig.maxVel = 1.5; // m/s
        elevatorConfig.maxAcc = 3.0; // m/s^2
        elevatorConfig.gearRatio = 12.0;
        elevatorConfig.kT = 0; // enable FF -> current in sim
        elevatorConfig.reversed = false;
        elevatorConfig.isBreak = true;
        elevatorConfig.supplyCurrentLimitEnabled = true;
        elevatorConfig.supplyCurrentLimit = 40.0;
        elevatorConfig.statorCurrentLimitEnabled = true;
        elevatorConfig.statorCurrentLimit = 40.0;
        elevatorConfig.updateFrequency = 100;
        elevatorConfig.isInnerSyncronized = true;
        elevatorMotor = new KrakenSimIO(elevatorConfig);

        // Arm motor & config
        armConfig = new MotorConfig("ArmSysMotor", 6, "rio");
        armConfig.kP = 5.0;
        armConfig.kI = 0.0;
        armConfig.kD = 2.0;
        armConfig.maxVel = 2.0; // rad/s
        armConfig.maxAcc = 4.0; // rad/s^2
        armConfig.gearRatio = 10.0;
        armConfig.kT = 0;
        armConfig.reversed = false;
        armConfig.isBreak = true;
        armConfig.supplyCurrentLimitEnabled = true;
        armConfig.supplyCurrentLimit = 40.0;
        armConfig.statorCurrentLimitEnabled = true;
        armConfig.statorCurrentLimit = 40.0;
        armConfig.updateFrequency = 100;
        armConfig.isInnerSyncronized = true;
        armMotor = new KrakenSimIO(armConfig);

        // Elevator physical properties (vertical Z axis)
        double carriageMass = 5.0;
        double axisLength = 1.0; // for CG placement only
        SimpleMatrix elevCG = new SimpleMatrix(3, 1);
        elevCG.set(0, 0, 0.0);
        elevCG.set(1, 0, 0.0);
        elevCG.set(2, 0, axisLength / 2.0);
        SimpleMatrix elevMOI = new SimpleMatrix(3, 3);
        elevMOI.zero();
        physicalProperties elevPhys =
                new physicalProperties(carriageMass, elevCG, elevMOI, java.util.Optional.empty());
        SimpleMatrix elevAxis = new SimpleMatrix(3, 1);
        elevAxis.set(0, 0, 0.0);
        elevAxis.set(1, 0, 0.0);
        elevAxis.set(2, 0, 1.0);
        SimpleMatrix elevStart = new SimpleMatrix(3, 1);
        elevStart.zero();

        elevator = new LinearMechanism<>(ELEVATOR_NAME, elevPhys, elevAxis, elevStart);
        elevator.registerMotor(elevatorMotor, elevatorConfig);

        // Arm physical properties (arm rotates about Y-axis, planar XZ)
        double armLength = 0.5;
        double armMass = 2.0;
        double moiY = armMass * armLength * armLength / 3.0;
        SimpleMatrix armCG = new SimpleMatrix(3, 1);
        armCG.set(0, 0, armLength / 2.0);
        armCG.set(1, 0, 0.0);
        armCG.set(2, 0, 0.0);
        SimpleMatrix armMOI = new SimpleMatrix(3, 3);
        armMOI.zero();
        armMOI.set(1, 1, moiY);
        physicalProperties armPhys =
                new physicalProperties(armMass, armCG, armMOI, java.util.Optional.empty());
        SimpleMatrix rotAxis = new SimpleMatrix(3, 1);
        rotAxis.set(0, 0, 0.0);
        rotAxis.set(1, 0, 1.0);
        rotAxis.set(2, 0, 0.0);
        SimpleMatrix pivot = new SimpleMatrix(3, 1);
        pivot.zero();

        arm = new RotatingMechanism<>(ARM_NAME, armPhys, rotAxis, pivot);
        arm.registerMotor(armMotor, armConfig);

        // Build mechanism system and link parent-child (Elevator -> Arm)
        system = new MechanismSystem("ElevatorArmSystem");
        system.addMechanism(elevator);
        system.addMechanism(arm);
        system.setParentChildRelation(ELEVATOR_NAME, ARM_NAME);

        Logger.recordOutput("ElevArmTest/Setup/CarriageMass", carriageMass);
        Logger.recordOutput("ElevArmTest/Setup/ArmLength", armLength);
    }

    @Override
    public void periodic() {
        // Update all mechanism states from motor inputs
        system.updateAllMechanismStates();

        // Log states
        Logger.recordOutput("ElevArmTest/Elevator/Position",
                ((LinearMechanism<?, ?, ?>) elevator).getCurrentPosition());
        Logger.recordOutput("ElevArmTest/Elevator/Velocity",
                ((LinearMechanism<?, ?, ?>) elevator).getVelocity());
        Logger.recordOutput("ElevArmTest/Arm/Angle",
                ((RotatingMechanism<?, ?, ?>) arm).getCurrentAngle());
        Logger.recordOutput("ElevArmTest/Arm/Velocity",
                ((RotatingMechanism<?, ?, ?>) arm).getAngularVelocity());

        if (elevator.getMotorCount() > 0) {
            MotorInputs mi = elevator.getMotorInputs(0);
            Logger.recordOutput("ElevArmTest/ElevatorMotor/Position", mi.position);
            Logger.recordOutput("ElevArmTest/ElevatorMotor/Velocity", mi.velocity);
            Logger.recordOutput("ElevArmTest/ElevatorMotor/Current", mi.current);
        }
        if (arm.getMotorCount() > 0) {
            MotorInputs mi = arm.getMotorInputs(0);
            Logger.recordOutput("ElevArmTest/ArmMotor/Position", mi.position);
            Logger.recordOutput("ElevArmTest/ArmMotor/Velocity", mi.velocity);
            Logger.recordOutput("ElevArmTest/ArmMotor/Current", mi.current);
        }
    }

    // Delegate playback to MechanismSystem
    public void followGroupAtTime(SetpointGroup group, double timeSec) {
        system.followSetpointGroupAtTime(group, timeSec);
    }

    public Map<String, Double> getCurrentPositions() {
        Map<String, Double> map = new HashMap<>();
        map.put(ELEVATOR_NAME, ((LinearMechanism<?, ?, ?>) elevator).getCurrentPosition());
        map.put(ARM_NAME, ((RotatingMechanism<?, ?, ?>) arm).getCurrentAngle());
        return map;
    }

    public Map<String, Double> getCurrentVelocities() {
        Map<String, Double> map = new HashMap<>();
        map.put(ELEVATOR_NAME, ((LinearMechanism<?, ?, ?>) elevator).getVelocity());
        map.put(ARM_NAME, ((RotatingMechanism<?, ?, ?>) arm).getAngularVelocity());
        return map;
    }
}


