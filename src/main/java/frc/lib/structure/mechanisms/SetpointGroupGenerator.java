package frc.lib.structure.mechanisms;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Generates a SetpointGroup for a MechanismSystem from current and target states. For now, supports
 * independent trapezoid profiles per mechanism name provided.
 */
public class SetpointGroupGenerator {

    public static class Constraints {
        public final double maxVel;
        public final double maxAcc;

        public Constraints(double maxVel, double maxAcc) {
            this.maxVel = maxVel;
            this.maxAcc = maxAcc;
        }
    }

    /**
     * State description used for generation per mechanism.
     */
    public static class MechanismState {
        public final double position;
        public final double velocity;

        public MechanismState(double position, double velocity) {
            this.position = position;
            this.velocity = velocity;
        }
    }

    /**
     * Generate time-sampled setpoints from current to target for each mechanism. SamplingPeriod sec
     * resolution.
     */
    public SetpointGroup generate(Map<String, MechanismState> current,
            Map<String, MechanismState> target, Map<String, Constraints> constraints,
            double samplingPeriodSec) {

        // Find maximum trajectory time among mechanisms to align timelines
        double maxTotalTime = 0.0;

        Map<String, TrapezoidProfile> profiles = new LinkedHashMap<>();
        Map<String, TrapezoidProfile.State> starts = new LinkedHashMap<>();
        Map<String, TrapezoidProfile.State> ends = new LinkedHashMap<>();
        Map<String, Double> totalTimes = new LinkedHashMap<>();

        for (String name : target.keySet()) {
            MechanismState cur = current.get(name);
            MechanismState tar = target.get(name);
            Constraints c = constraints.get(name);
            if (cur == null || tar == null || c == null)
                continue;
            TrapezoidProfile.Constraints cons =
                    new TrapezoidProfile.Constraints(c.maxVel, c.maxAcc);
            TrapezoidProfile profile = new TrapezoidProfile(cons);
            TrapezoidProfile.State start = new TrapezoidProfile.State(cur.position, cur.velocity);
            TrapezoidProfile.State end = new TrapezoidProfile.State(tar.position, 0.0);
            // Prime the profile to compute total time for these endpoints
            profile.calculate(0.0, start, end);
            double totalTime = profile.totalTime();

            profiles.put(name, profile);
            starts.put(name, start);
            ends.put(name, end);
            totalTimes.put(name, totalTime);
            maxTotalTime = Math.max(maxTotalTime, totalTime);
        }

        // Sample common timeline
        List<Double> times = new ArrayList<>();
        List<Map<String, SetPoint>> allSetpoints = new ArrayList<>();

        for (double t = 0.0; t <= maxTotalTime + 1e-9; t += samplingPeriodSec) {
            Map<String, SetPoint> spForT = new LinkedHashMap<>();
            for (String name : profiles.keySet()) {
                TrapezoidProfile profile = profiles.get(name);
                TrapezoidProfile.State start = starts.get(name);
                TrapezoidProfile.State end = ends.get(name);

                double totalTime = totalTimes.get(name);
                TrapezoidProfile.State s = profile.calculate(Math.min(t, totalTime), start, end);
                double acc = 0.0;
                if (t >= samplingPeriodSec) {
                    TrapezoidProfile.State prev = profile
                            .calculate(Math.min(t - samplingPeriodSec, totalTime), start, end);
                    acc = (s.velocity - prev.velocity) / samplingPeriodSec;
                }
                spForT.put(name, new SetPoint(s.position, s.velocity, acc, Double.NaN));
            }
            times.add(t);
            allSetpoints.add(spForT);
        }

        return new SetpointGroup(times, allSetpoints);
    }
}


