package frc.lib.structure.mechanisms;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

/**
 * Loads SetpointGroup from a JSON file. JSON format: { "timeline": [ { "t": 0.0, "setpoints": {
 * "Arm": {"pos": 0.0, "vel": 0.0, "acc": 0.0}, "Elev": { ... } } }, { "t": 0.02, "setpoints": { ...
 * } } ] }
 */
public class SetpointGroupJsonLoader {
    private static final Gson GSON = new Gson();

    public SetpointGroup load(File file) throws IOException {
        try (FileReader reader = new FileReader(file)) {
            JsonObject root = JsonParser.parseReader(reader).getAsJsonObject();
            JsonArray timeline = root.getAsJsonArray("timeline");
            List<Double> times = new ArrayList<>();
            List<Map<String, SetPoint>> all = new ArrayList<>();

            for (JsonElement e : timeline) {
                JsonObject entry = e.getAsJsonObject();
                double t = entry.get("t").getAsDouble();
                JsonObject sps = entry.getAsJsonObject("setpoints");

                Map<String, SetPoint> map = new LinkedHashMap<>();
                for (Map.Entry<String, JsonElement> me : sps.entrySet()) {
                    String name = me.getKey();
                    JsonObject sp = me.getValue().getAsJsonObject();
                    double pos = sp.get("pos").getAsDouble();
                    double vel = sp.get("vel").getAsDouble();
                    double acc = sp.get("acc").getAsDouble();
                    map.put(name, new SetPoint(pos, vel, acc, Double.NaN));
                }
                times.add(t);
                all.add(map);
            }

            return new SetpointGroup(times, all);
        }
    }
}


