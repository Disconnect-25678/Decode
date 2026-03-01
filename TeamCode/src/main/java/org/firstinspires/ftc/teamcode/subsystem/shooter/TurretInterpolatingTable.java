package org.firstinspires.ftc.teamcode.subsystem.shooter;

import static java.util.Map.entry;

import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

public class TurretInterpolatingTable {

    private TurretInterpolatingTable() {}

    public static TreeMap<Double, Double> table =
            new TreeMap<>(
                    Map.ofEntries(
                        entry(
                                Double.valueOf(0),
                                Double.valueOf(0)
                        ),
                        entry(
                                Double.valueOf(0.926),
                                Double.valueOf(360 - 295.84702679)
                        ),
                        entry(
                                Double.valueOf(1.29),
                                Double.valueOf(90)
                        ),
                        entry(
                                Double.valueOf(1.641),
                                Double.valueOf(90 + 24.764)
                        ),
                        entry(
                                Double.valueOf(2.553),
                                Double.valueOf(180)
                        ),
                        entry(
                                Double.valueOf(3.485),
                                Double.valueOf(270 - 24.764)
                        ),
                        entry(
                                Double.valueOf(3.845),
                                Double.valueOf(270)
                        ),
                        entry(
                                Double.valueOf(4.096),
                                Double.valueOf(270 + 25.84702679)
                        ),
                        entry(
                                Double.valueOf(5),
                                Double.valueOf(360)
                        )
                    ));

    public static Double get(double encoderValue) {
        Entry<Double, Double> ceil = table.ceilingEntry(encoderValue);
        Entry<Double, Double> floor = table.floorEntry(encoderValue);
        if (ceil == null) return floor.getValue();
        if (floor == null) return ceil.getValue();
        if (ceil.getValue().equals(floor.getValue())) return ceil.getValue();
        return Double.valueOf(
                lerp(
                        floor.getValue().doubleValue(),
                        ceil.getValue().doubleValue(),
                        (encoderValue - floor.getKey().doubleValue()) / (ceil.getKey().doubleValue() - floor.getKey().doubleValue())
                )
        );
    }

    private static double lerp(double y1, double y2, double t) {
        return y1 + (t * (y2 - y1));
    }

}