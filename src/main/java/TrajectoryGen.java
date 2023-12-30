import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;

import java.util.ArrayList;

public class TrajectoryGen extends TrajectoryGenInternal {
    public static MecanumVelocityConstraint velConstraint = new MecanumVelocityConstraint(30, 16);
    public static ProfileAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(30);

    /**
     * Populate trajectories here using addNewTrajectory()
     */
    public static ArrayList<Trajectory> createTrajectories() {


        return getT();
    }
}
