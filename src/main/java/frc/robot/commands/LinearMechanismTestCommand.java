package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LinearMechanismTestSubsystem;

public class LinearMechanismTestCommand extends Command {
    private final LinearMechanismTestSubsystem subsystem;

    private static final double MAX_VEL = 1.0; // m/s
    private static final double MAX_ACC = 2.0; // m/s^2

    // Waypoints (meters)
    private static final double[] WAYPOINTS = {0.0, 0.3, 0.6, 0.2, 0.5, 0.0};

    private int currentWaypoint = 0;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State startState;
    private TrapezoidProfile.State endState;
    private double profileStartTime;
    private boolean profileActive = false;

    public LinearMechanismTestCommand(LinearMechanismTestSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        currentWaypoint = 0;
        profileActive = false;
        Logger.recordOutput("LinearMechanismTestCommand/Initialized", true);
        Logger.recordOutput("LinearMechanismTestCommand/Waypoints", WAYPOINTS.length);
        Logger.recordOutput("LinearMechanismTestCommand/MaxVel", MAX_VEL);
        Logger.recordOutput("LinearMechanismTestCommand/MaxAcc", MAX_ACC);
    }

    @Override
    public void execute() {
        double now = System.currentTimeMillis() / 1000.0;
        if (!profileActive && currentWaypoint < WAYPOINTS.length) {
            startNewProfile();
        }

        if (profileActive) {
            double t = now - profileStartTime;
            TrapezoidProfile.State s = profile.calculate(t, startState, endState);
            double acc = 0.0;
            if (t > 0.02) {
                TrapezoidProfile.State prev = profile.calculate(t - 0.02, startState, endState);
                acc = (s.velocity - prev.velocity) / 0.02;
            }
            subsystem.setTarget(s.position, s.velocity, acc);

            double totalTime = profile.totalTime();
            Logger.recordOutput("LinearMechanismTestCommand/ProfileTime", t);
            Logger.recordOutput("LinearMechanismTestCommand/TotalTime", totalTime);
            Logger.recordOutput("LinearMechanismTestCommand/Setpoint/Pos", s.position);
            Logger.recordOutput("LinearMechanismTestCommand/Setpoint/Vel", s.velocity);
            Logger.recordOutput("LinearMechanismTestCommand/Setpoint/Acc", acc);

            if (t >= totalTime - 0.01) {
                profileActive = false;
                currentWaypoint++;
                Logger.recordOutput("LinearMechanismTestCommand/ProfileCompleted", true);
            }
        }
    }

    private void startNewProfile() {
        TrapezoidProfile.Constraints c = new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACC);
        profile = new TrapezoidProfile(c);
        startState = new TrapezoidProfile.State(subsystem.getPosition(), subsystem.getVelocity());
        endState = new TrapezoidProfile.State(WAYPOINTS[currentWaypoint], 0.0);
        profileStartTime = System.currentTimeMillis() / 1000.0;
        profileActive = true;
        Logger.recordOutput("LinearMechanismTestCommand/ProfileStarted", true);
        Logger.recordOutput("LinearMechanismTestCommand/Target", WAYPOINTS[currentWaypoint]);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
        Logger.recordOutput("LinearMechanismTestCommand/Interrupted", interrupted);
    }

    @Override
    public boolean isFinished() {
        return currentWaypoint >= WAYPOINTS.length;
    }
}







