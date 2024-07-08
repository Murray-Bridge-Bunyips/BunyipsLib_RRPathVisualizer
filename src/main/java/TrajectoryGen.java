import bunyipslib.external.units.Angle;
import bunyipslib.external.units.Distance;
import bunyipslib.external.units.Measure;
import bunyipslib.external.units.Velocity;

import static bunyipslib.external.units.Units.*;
import com.acmerobotics.roadrunner.geometry.*;

public class TrajectoryGen extends TrajectoryGenInternal {
    public static Measure<Distance> trackWidth =
            Inches.of(18);
    public static Measure<Velocity<Distance>> maxVel =
            Meters.per(Second).of(1);
    public static Measure<Velocity<Angle>> maxAngularVel =
            Degrees.per(Second).of(90);
    public static Measure<Velocity<Velocity<Distance>>> maxAccel =
            Meters.per(Second).per(Second).of(20);

    /**
     * Populate trajectories here using makeTrajectory().
     * <p>
     * Note some features originally found in BunyipsLib including advanced task construction,
     * speed constraints, and certain markers are not supported in this environment. This includes TrajectorySequences and therefore
     * some routes will throw PathContinuityExceptions/compilation errors and will not reflect real robot behaviour.
     * <p>
     * Therefore, some routes may need adjusting/method omission to work in this environment.
     */
    public static void createTrajectories() {

    }
}
