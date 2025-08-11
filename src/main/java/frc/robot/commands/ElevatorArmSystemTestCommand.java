package frc.robot.commands;

import java.util.LinkedHashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.structure.mechanisms.SetpointGroup;
import frc.lib.structure.mechanisms.SetpointGroupGenerator;
import frc.lib.structure.mechanisms.SetpointGroupGenerator.Constraints;
import frc.lib.structure.mechanisms.SetpointGroupGenerator.MechanismState;
import frc.robot.subsystems.ElevatorArmSystemTestSubsystem;

public class ElevatorArmSystemTestCommand extends Command {
    private final ElevatorArmSystemTestSubsystem subsystem;

    private static final double ELEV_MAX_VEL = 1.0; // m/s
    private static final double ELEV_MAX_ACC = 2.0; // m/s^2
    private static final double ARM_MAX_VEL = 1.5; // rad/s
    private static final double ARM_MAX_ACC = 3.0; // rad/s^2

    private static final double[][] WAYPOINTS = new double[][] {
            // {elevator_pos_m, arm_angle_rad}
            {0.0, 0.0}, {0.6, Math.PI / 2}, {0., 0}};

    private boolean active = false;
    private int idx = 0;
    private double t0;
    private SetpointGroup group;

    public ElevatorArmSystemTestCommand(ElevatorArmSystemTestSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        active = false;
        idx = 0;
        Logger.recordOutput("ElevArmTestCommand/Initialized", true);
    }

    @Override
    public void execute() {
        double now = System.currentTimeMillis() / 1000.0;

        if (!active && idx < WAYPOINTS.length) {
            // Current state
            Map<String, Double> pos = subsystem.getCurrentPositions();
            Map<String, Double> vel = subsystem.getCurrentVelocities();
            Map<String, MechanismState> current = new LinkedHashMap<>();
            current.put("Elevator", new MechanismState(pos.get("Elevator"), vel.get("Elevator")));
            current.put("Arm", new MechanismState(pos.get("Arm"), vel.get("Arm")));

            // Target state
            Map<String, MechanismState> target = new LinkedHashMap<>();
            target.put("Elevator", new MechanismState(WAYPOINTS[idx][0], 0.0));
            target.put("Arm", new MechanismState(WAYPOINTS[idx][1], 0.0));

            // Constraints
            Map<String, Constraints> constraints = new LinkedHashMap<>();
            constraints.put("Elevator", new Constraints(ELEV_MAX_VEL, ELEV_MAX_ACC));
            constraints.put("Arm", new Constraints(ARM_MAX_VEL, ARM_MAX_ACC));

            SetpointGroupGenerator gen = new SetpointGroupGenerator();
            group = gen.generate(current, target, constraints, 0.02);
            t0 = now;
            active = true;
            Logger.recordOutput("ElevArmTestCommand/ProfileStarted", true);
            Logger.recordOutput("ElevArmTestCommand/TargetElevator", WAYPOINTS[idx][0]);
            Logger.recordOutput("ElevArmTestCommand/TargetArm", WAYPOINTS[idx][1]);
        }

        if (active && group != null) {
            double t = now - t0;
            subsystem.followGroupAtTime(group, t);
            Logger.recordOutput("ElevArmTestCommand/PlaybackTime", t);
            if (t >= group.getTimes().get(group.size() - 1) - 1e-3) {
                active = false;
                idx++;
                Logger.recordOutput("ElevArmTestCommand/ProfileCompleted", true);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("ElevArmTestCommand/Interrupted", interrupted);
    }

    @Override
    public boolean isFinished() {
        return idx >= WAYPOINTS.length;
    }
}


