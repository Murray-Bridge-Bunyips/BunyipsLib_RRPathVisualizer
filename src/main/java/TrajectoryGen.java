import bunyipslib.external.units.Angle;
import bunyipslib.external.units.Distance;
import bunyipslib.external.units.Measure;
import bunyipslib.external.units.Velocity;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import java.util.ArrayList;

import static bunyipslib.external.units.Units.*;

public class TrajectoryGen extends TrajectoryGenInternal {
    public static Measure<Distance> trackWidth =
            Inches.of(18);
    public static Measure<Velocity<Distance>> maxVel =
            Meters.per(Second).of(10);
    public static Measure<Velocity<Angle>> maxAngularVel =
            Degrees.per(Second).of(45);
    public static Measure<Velocity<Velocity<Distance>>> maxAccel =
            Meters.per(Second).per(Second).of(20);
    public static Measure<Velocity<Velocity<Angle>>> maxAngularAccel =
            Degrees.per(Second).per(Second).of(45);

    /**
     * Populate trajectories here using makeTrajectory()
     */
    public static ArrayList<Trajectory> createTrajectories() {
        makeTrajectory().forward(100).addTask();
        return getT();
    }
}
