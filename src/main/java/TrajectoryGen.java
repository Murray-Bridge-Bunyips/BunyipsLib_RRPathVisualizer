import bunyipslib.external.units.Angle;
import bunyipslib.external.units.Distance;
import bunyipslib.external.units.Measure;
import bunyipslib.external.units.Velocity;

import static bunyipslib.external.units.Units.*;

public class TrajectoryGen extends TrajectoryGenInternal {
    public static Measure<Distance> trackWidth =
            Inches.of(18);
    public static Measure<Velocity<Distance>> maxVel =
            Inches.per(Second).of(30);
    public static Measure<Velocity<Angle>> maxAngularVel =
            Degrees.per(Second).of(180);
    public static Measure<Velocity<Velocity<Distance>>> maxAccel =
            Inches.per(Second).per(Second).of(30);
    public static boolean autoAddTaskBetweenTrajectories = true;

    /**
     * Populate trajectories here using makeTrajectory().
     * <p>
     * Note some features originally found in BunyipsLib including advanced task construction,
     * speed constraints, and certain markers are not supported in this environment. This includes TrajectorySequences and therefore
     * some routes will throw continuity/compilation errors and will not reflect real robot behaviour. Some strategies can be used to mitigate
     * this, such as the autoAddTaskBetweenTrajectories flag.
     * <p>
     * Therefore, some routes may need adjusting/method omission to work in this environment.
     */
    public static void createTrajectories() {

    }
}
