import TrajectoryGen.driveConstraints
import TrajectoryGen.trackWidth
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints

/**
 * Port of BunyipsLib `addNewTrajectory()` features
 * @author Lucas Bubner, 2023
 */
open class TrajectoryGenInternal {
    companion object {
        @JvmStatic
        protected val t = ArrayList<Trajectory>()

        @JvmStatic
        private var lastPose: Pose2d = Pose2d()

        @JvmStatic
        protected fun addNewTrajectory() =
            Builder(lastPose, lastPose.heading, MecanumConstraints(driveConstraints, trackWidth))

        @JvmStatic
        protected fun addNewTrajectory(pose: Pose2d) =
            Builder(pose, pose.heading, MecanumConstraints(driveConstraints, trackWidth))
    }

    class Builder(pose: Pose2d, heading: Double, constraints: MecanumConstraints) {
        private val i: TrajectoryBuilder = TrajectoryBuilder(pose, heading, constraints)

        fun build(): Trajectory {
            val trajectory = i.build()
            lastPose = trajectory.end()
            t.add(trajectory)
            return trajectory
        }

        fun addDisplacementMarker(marker: MarkerCallback) = run {
            i.addDisplacementMarker(marker)
            this
        }

        fun addDisplacementMarker(
            displacement: (Double) -> Double,
            callback: MarkerCallback
        ) = run {
            i.addDisplacementMarker(displacement, callback)
            this
        }

        fun addDisplacementMarker(
            displacement: Double,
            callback: MarkerCallback
        ) = run {
            i.addDisplacementMarker(displacement, callback)
            this
        }

        fun addDisplacementMarker(
            scale: Double,
            offset: Double,
            callback: MarkerCallback
        ) = run {
            i.addDisplacementMarker(scale, offset, callback)
            this
        }

        fun addSpatialMarker(
            point: com.acmerobotics.roadrunner.geometry.Vector2d,
            callback: MarkerCallback
        ) = run {
            i.addSpatialMarker(point, callback)
            this
        }

        fun addTemporalMarker(
            time: (Double) -> Double,
            callback: MarkerCallback
        ) = run {
            i.addTemporalMarker(time, callback)
            this
        }

        fun addTemporalMarker(time: Double, callback: MarkerCallback) =
            run {
                i.addTemporalMarker(time, callback)
                this
            }

        fun addTemporalMarker(
            scale: Double,
            offset: Double,
            callback: MarkerCallback
        ) = run {
            i.addTemporalMarker(scale, offset, callback)
            this
        }

        fun back(distance: Double) = run {
            i.back(distance)
            this
        }

        fun forward(distance: Double) = run {
            i.forward(distance)
            this
        }

        fun lineTo(endPosition: com.acmerobotics.roadrunner.geometry.Vector2d) = run {
            i.lineTo(endPosition)
            this
        }

        fun lineToConstantHeading(endPosition: com.acmerobotics.roadrunner.geometry.Vector2d) = run {
            i.lineToConstantHeading(endPosition)
            this
        }

        fun lineToLinearHeading(endPose: Pose2d) = run {
            i.lineToLinearHeading(endPose)
            this
        }

        fun lineToSplineHeading(endPose: Pose2d) = run {
            i.lineToSplineHeading(endPose)
            this
        }

        fun splineTo(endPosition: com.acmerobotics.roadrunner.geometry.Vector2d, endTangent: Double) = run {
            i.splineTo(endPosition, endTangent)
            this
        }

        fun splineToConstantHeading(
            endPosition: com.acmerobotics.roadrunner.geometry.Vector2d,
            endTangent: Double
        ) = run {
            i.splineToConstantHeading(endPosition, endTangent)
            this
        }

        fun splineToLinearHeading(endPose: Pose2d, endTangent: Double) =
            run {
                i.splineToLinearHeading(endPose, endTangent)
                this
            }

        fun splineToSplineHeading(endPose: Pose2d, endTangent: Double) =
            run {
                i.splineToSplineHeading(endPose, endTangent)
                this
            }

        fun strafeLeft(distance: Double) = run {
            i.strafeLeft(distance)
            this
        }

        fun strafeRight(distance: Double) = run {
            i.strafeRight(distance)
            this
        }

        fun strafeTo(endPosition: com.acmerobotics.roadrunner.geometry.Vector2d) = run {
            i.strafeTo(endPosition)
            this
        }
    }
}