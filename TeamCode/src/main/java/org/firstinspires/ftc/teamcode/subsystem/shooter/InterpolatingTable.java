package org.firstinspires.ftc.teamcode.subsystem.shooter;

import static java.util.Map.entry;

// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.units.Unit;

import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

public class InterpolatingTable {

    private InterpolatingTable() {}

    public static TreeMap<Double, ShotParameter> table =
            new TreeMap<>(
                    Map.ofEntries(
                            entry(
                                    Double.valueOf(41.796553470206),
                                    new ShotParameter(21, 4000)
                            ),
                            entry(
                                    Double.valueOf(52.6828993327078),
                                    new ShotParameter(25, 4200)
                            ),
                            entry(
                                    Double.valueOf(64.4312152250869),
                                    new ShotParameter(27.5, 4400)
                            ),
                            entry(
                                    Double.valueOf(74.1253074920102),
                                    new ShotParameter(29, 4550)
                            ),
                            entry(
                                    Double.valueOf(79.5281971451608),
                                    new ShotParameter(33, 4800)
                            ),
                            entry(
                                    Double.valueOf(89.625738615174),
                                    new ShotParameter(36, 4925)
                            ),
                            entry(
                                    Double.valueOf(93.5489044189453),
                                    new ShotParameter(37, 4925)
                            ),
                            entry(
                                    Double.valueOf(100.100965940353),
                                    new ShotParameter(38, 4950)
                            )
                    )
            );

//    public static TreeMap<Double, ShotParameter> table =
//            new TreeMap<>(
//                    Map.ofEntries(
//                            entry(
//                                    Double.valueOf(42.63879765177995),
//                                    new ShotParameter(16, 4000)),
//                            entry(
//                                    Double.valueOf(50.80696943477022),
//                                    new ShotParameter(17.5, 4300)
//                            ),
//                            entry(
//                                    Double.valueOf(71.60300367997554),
//                                    new ShotParameter(22, 4350)
//                            ),
////                            entry(
////                                    Double.valueOf(72.0233951970043),
////                                    new ShotParameter(33, 4900)
////                            ),
//                            entry(
//                                    Double.valueOf(80.4723339717294),
//                                    new ShotParameter(25, 4600)
//                            ),
//                            entry(
//                                    Double.valueOf(84.6669275974),
//                                    new ShotParameter(25, 4600)
//                            ),
//                            entry(
//                                    Double.valueOf(96.39638920286106),
//                                    new ShotParameter(26, 4800)
//                            ),
//                            entry(
//                                    Double.valueOf(100.7779904061193),
//                                    new ShotParameter(25, 4800)
//                            ),
//                            entry(
//                                    Double.valueOf(107.90704069273725),
//                                    new ShotParameter(26, 5000)
//                            )
////                            entry(
////                                    Double.valueOf(Math.hypot(60, 60)),
////                                    new ShotParameter(35, 5500))
////                            entry(
////                                    Double.valueOf(150),
////                                    new ShotParameter(32, 6000)),
////                            entry(
////                                    Double.valueOf(200),
////                                    new ShotParameter(45, 6000))
//                    ));

    public static ShotParameter get(double distanceToTarget) {
        Entry<Double, ShotParameter> ceil = table.ceilingEntry(distanceToTarget);
        Entry<Double, ShotParameter> floor = table.floorEntry(distanceToTarget);
        if (ceil == null) return floor.getValue();
        if (floor == null) return ceil.getValue();
        if (ceil.getValue().equals(floor.getValue())) return ceil.getValue();
        return floor
                .getValue()
                .interpolate(
                        ceil.getValue(),
                        (distanceToTarget - floor.getKey()) / (ceil.getKey() - floor.getKey()));
    }
}