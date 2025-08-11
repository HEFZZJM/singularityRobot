package frc.lib.structure.mechanisms;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * A time-parameterized group of setpoints for a mechanism system. It stores, for each time t
 * (seconds), a mapping from mechanism name to its SetPoint.
 */
public class SetpointGroup {
    // Times in ascending order
    private final List<Double> timeSeconds;
    // For each timestamp, a map of mechanismName -> setpoint
    private final List<Map<String, SetPoint>> setpointsAtTimes;

    public SetpointGroup(List<Double> timeSeconds, List<Map<String, SetPoint>> setpointsAtTimes) {
        if (timeSeconds.size() != setpointsAtTimes.size()) {
            throw new IllegalArgumentException("timeSeconds and setpointsAtTimes size mismatch");
        }
        this.timeSeconds = List.copyOf(timeSeconds);
        // Deep copy to avoid mutation
        this.setpointsAtTimes = setpointsAtTimes.stream()
                .map(m -> Collections.unmodifiableMap(new LinkedHashMap<>(m))).toList();
    }

    public int size() {
        return timeSeconds.size();
    }

    public List<Double> getTimes() {
        return timeSeconds;
    }

    public List<Map<String, SetPoint>> getSetpoints() {
        return setpointsAtTimes;
    }

    public Map<String, SetPoint> getSetpointsAtIndex(int idx) {
        return setpointsAtTimes.get(idx);
    }

    public double getTimeAtIndex(int idx) {
        return timeSeconds.get(idx);
    }
}




