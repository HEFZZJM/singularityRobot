package frc.robot.commands;

import java.util.LinkedHashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.structure.mechanisms.SetpointGroup;
import frc.lib.structure.mechanisms.SetpointGroupGenerator;
import frc.lib.structure.mechanisms.SetpointGroupGenerator.Constraints;
import frc.lib.structure.mechanisms.SetpointGroupGenerator.MechanismState;
import frc.robot.subsystems.MechanismSystemArmTestSubsystem;

public class MechanismSystemArmTestCommand extends Command {
    private final MechanismSystemArmTestSubsystem subsystem;

    private static final double MAX_VEL = 2.0; // rad/s
    private static final double MAX_ACC = 4.0; // rad/s^2
    private static final double[] WAYPOINTS = {0.0, Math.PI / 6, -Math.PI / 6, Math.PI / 3, 0.0};

    private boolean profileActive = false;
    private int idx = 0;
    private double t0;
    private SetpointGroup currentGroup;

    public MechanismSystemArmTestCommand(MechanismSystemArmTestSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        profileActive = false;
        idx = 0;
        Logger.recordOutput("MSArmTestCommand/Initialized", true);
    }

    @Override
    public void execute() {
        double now = System.currentTimeMillis() / 1000.0;
        if (!profileActive && idx < WAYPOINTS.length) {
            Map<String, MechanismState> current = new LinkedHashMap<>();
            current.put("Arm",
                    new MechanismState(subsystem.getAngle(), subsystem.getAngularVelocity()));
            Map<String, MechanismState> target = new LinkedHashMap<>();
            target.put("Arm", new MechanismState(WAYPOINTS[idx], 0.0));
            Map<String, Constraints> constraints = new LinkedHashMap<>();
            constraints.put("Arm", new Constraints(MAX_VEL, MAX_ACC));

            SetpointGroupGenerator gen = new SetpointGroupGenerator();
            currentGroup = gen.generate(current, target, constraints, 0.02);
            t0 = now;
            profileActive = true;
            Logger.recordOutput("MSArmTestCommand/ProfileStarted", true);
            Logger.recordOutput("MSArmTestCommand/Target", WAYPOINTS[idx]);
        }

        if (profileActive && currentGroup != null) {
            double t = now - t0;
            subsystem.followGroupAtTime(currentGroup, t);
            Logger.recordOutput("MSArmTestCommand/PlaybackTime", t);

            if (t >= currentGroup.getTimes().get(currentGroup.size() - 1) - 1e-3) {
                profileActive = false;
                idx++;
                Logger.recordOutput("MSArmTestCommand/ProfileCompleted", true);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
        Logger.recordOutput("MSArmTestCommand/Interrupted", interrupted);
    }

    @Override
    public boolean isFinished() {
        return idx >= WAYPOINTS.length;
    }
}


