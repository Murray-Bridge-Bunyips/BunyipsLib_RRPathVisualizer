import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

import java.util.ArrayList;

public class TrajectoryGen extends TrajectoryGenInternal {
    public static double trackWidth = 16.0; // in
    public static DriveConstraints driveConstraints = new DriveConstraints(
            60.0, // maxVel (in/s)
            60.0, // maxAccel (in/s^2)
            0.0, // maxJerk (in/s^3)
            Math.toRadians(270.0), // maxAngVel (rad/s)
            Math.toRadians(270.0), // maxAngAccel (rad/s^2)
            0.0 // maxAngJerk (rad/s^3)
    );

    /**
     * Populate trajectories here using addNewTrajectory()
     */
    public static ArrayList<Trajectory> createTrajectories() {


        return getT();
    }
}
