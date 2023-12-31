import TrajectoryGen.accelConstraint
import TrajectoryGen.velConstraint
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.*
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import com.acmerobotics.roadrunner.util.Angle

/**
 * Port of BunyipsLib RoadRunner `addNewTrajectory()` features and a hacky port of TrajectorySequence
 * @author Lucas Bubner, 2023
 */
@Suppress("unused")
open class TrajectoryGenInternal {
    companion object {
        private const val TURN_OFFSET = 0.01

        @JvmStatic
        protected val t = ArrayList<Trajectory>()

        @JvmStatic
        val timeouts = ArrayList<Pair<Int, Double>>()

        @JvmStatic
        private var lastPose: Pose2d = Pose2d()

        @JvmStatic
        protected fun addNewTrajectory() = Builder(lastPose, lastPose.heading, velConstraint, accelConstraint)

        @JvmStatic
        protected fun addNewTrajectory(pose: Pose2d) = Builder(pose, pose.heading, velConstraint, accelConstraint)

        // newTrajectory methods use the base implementation, which represents their normal behaviour of not accepting
        // constraints and methods such as reversed, waitSeconds, turn etc. as they are not supported normally
        @JvmStatic
        protected fun newTrajectory() = TrajectoryBuilder(lastPose, lastPose.heading, velConstraint, accelConstraint)

        @JvmStatic
        protected fun newTrajectory(pose: Pose2d) = TrajectoryBuilder(pose, pose.heading, velConstraint, accelConstraint)
    }

    class Builder(pose: Pose2d, heading: Double, velocityConstraint: TrajectoryVelocityConstraint, accelerationConstraint: TrajectoryAccelerationConstraint) {
        private var i: TrajectoryBuilder = TrajectoryBuilder(pose, heading, velocityConstraint, accelerationConstraint)

        @Suppress("unused_parameter")
        fun setConstraints(
            velConstraint: TrajectoryVelocityConstraint,
            accelConstraint: TrajectoryAccelerationConstraint
        ): Builder {
            println("setConstraints(velConstraint, accelConstraint) called on trajectory ${t.size + 1}, not supported!")
            return this
        }

        fun resetConstraints(): Builder {
            println("resetConstraints() called on trajectory ${t.size + 1}, not supported!")
            return this
        }

        @Suppress("unused_parameter")
        fun setVelConstraint(velConstraint: TrajectoryVelocityConstraint): Builder {
            println("setVelConstraint(velConstraint) called on trajectory ${t.size + 1}, not supported!")
            return this
        }

        fun resetVelConstraint(): Builder {
            println("resetVelConstraint() called on trajectory ${t.size + 1}, not supported!")
            return this
        }

        @Suppress("unused_parameter")
        fun setAccelConstraint(accelConstraint: TrajectoryAccelerationConstraint): Builder {
            println("setAccelConstraint(accelConstraint) called on trajectory ${t.size + 1}, not supported!")
            return this
        }

        fun resetAccelConstraint(): Builder {
            println("resetAccelConstraint() called on trajectory ${t.size + 1}, not supported!")
            return this
        }

        fun setTurnConstraint(maxAngVel: Double, maxAngAccel: Double): Builder {
            println("setTurnConstraint($maxAngVel, $maxAngAccel) called on trajectory ${t.size + 1}, not supported!")
            return this
        }

        fun resetTurnConstraint(): Builder {
            println("resetTurnConstraint() called on trajectory ${t.size + 1}, not supported!")
            return this
        }

        @Suppress("unused_parameter")
        fun UNSTABLE_addTemporalMarkerOffset(offset: Double, callback: MarkerCallback): Builder {
            println("UNSTABLE_addTemporalMarkerOffset($offset, callback) called on trajectory ${t.size + 1}, not supported!")
            return this
        }

        @Suppress("unused_parameter")
        fun UNSTABLE_addDisplacementMarkerOffset(offset: Double, callback: MarkerCallback): Builder {
            println("UNSTABLE_addDisplacementMarkerOffset($offset, callback) called on trajectory ${t.size + 1}, not supported!")
            return this
        }

        fun setTangent(tangent: Double): Builder {
            println("setTangent($tangent) called on trajectory ${t.size + 1}, not supported!")
            return this
        }

        fun setReversed(reversed: Boolean): Builder {
            println("setReversed($reversed) called on trajectory ${t.size + 1}, not supported!")
            return this
        }

        fun turn(angle: Double, maxAngVel: Double, maxAngAccel: Double): Builder {
            // Constraints are not supported, so we will just ignore them
            println("turn($angle, $maxAngVel, $maxAngAccel) called on trajectory ${t.size + 1}, not supported! passed to turn($angle) ...")
            turn(angle)
            return this
        }

        fun build(): Trajectory? {
            try {
                val trajectory = i.build()
                t.add(trajectory)
                lastPose = trajectory.end()
                i = TrajectoryBuilder(lastPose, lastPose.heading, velConstraint, accelConstraint)
                return trajectory
            } catch (e: NoSuchElementException) {
                // Trajectory is empty, will return null to avoid errors
                return null
            }
        }

        fun waitSeconds(seconds: Double): Builder {
            if (seconds < 0.0)
                throw IllegalArgumentException("seconds must be positive")

            // Need to early build to update queue timing
            build()

            timeouts.add(Pair(t.size - 1, seconds))
            if (t.size == 0)
                println("waitSeconds($seconds) is not natively supported, queued start wait")
            else
                println("waitSeconds($seconds) is not natively supported, queued extended idle duration for trajectory ${t.size + 1}")
            return this
        }

        fun turn(angle: Double): Builder {
            // Need to early build and reset to get the end pose and to split the trajectory
            build()

            // We can't turn naturally, so we will instead translate the turn into a lineToLinearHeading mock trajectory
            // This trajectory has a small length to not raise an empty trajectory error, which is a bit of a hack
            // However, this offset will be reversed when the next trajectory is constructed
            val newPose = Pose2d(
                lastPose.vec().plus(Vector2d(TURN_OFFSET, 0.0)),
                Angle.norm(lastPose.heading + angle)
            )

            addNewTrajectory()
                .lineToLinearHeading(newPose)
                .build()

            // Since we have inserted a trajectory with modified heading we need to reset the builder
            // We can remove the offset here as well, effectively reversing any inaccuracies
            i = TrajectoryBuilder(
                Pose2d(newPose.vec().minus(Vector2d(TURN_OFFSET, 0.0)), newPose.heading),
                newPose.heading,
                velConstraint,
                accelConstraint
            )

            println("turn($angle) has been translated into trajectory ${t.size}, as this is not a native method")
            return this
        }

        fun addTrajectory(trajectory: Trajectory): Builder {
            println("addTrajectory(${trajectory.start()} -> ${trajectory.end()}, ${trajectory.duration()} sec) called on trajectory ${t.size + 1}, added as a new trajectory.")
            // We can't attach the trajectory to the sequence, but we can instead add it to the trajectories list
            t.add(trajectory)
            return this
        }

        fun lineTo(endPosition: Vector2d): Builder {
            i.lineTo(endPosition)

            return this
        }

        /**
         * Adds a line segment with constant heading interpolation.
         *
         * @param endPosition end position
         */
        fun lineToConstantHeading(endPosition: Vector2d): Builder {
            i.lineToConstantHeading(endPosition)

            return this
        }

        /**
         * Adds a line segment with linear heading interpolation.
         *
         * @param endPose end pose
         */
        fun lineToLinearHeading(endPose: Pose2d): Builder {
            i.lineToLinearHeading(endPose)

            return this
        }

        /**
         * Adds a line segment with spline heading interpolation.
         *
         * @param endPose end pose
         */
        fun lineToSplineHeading(endPose: Pose2d): Builder {
            i.lineToSplineHeading(endPose)

            return this
        }

        /**
         * Adds a strafe path segment.
         *
         * @param endPosition end position
         */
        fun strafeTo(endPosition: Vector2d): Builder {
            i.strafeTo(endPosition)

            return this
        }

        /**
         * Adds a line straight forward.
         *
         * @param distance distance to travel forward
         */
        fun forward(distance: Double): Builder {
            i.forward(distance)

            return this
        }

        /**
         * Adds a line straight backward.
         *
         * @param distance distance to travel backward
         */
        fun back(distance: Double): Builder {
            i.back(distance)

            return this
        }

        /**
         * Adds a segment that strafes left in the robot reference frame.
         *
         * @param distance distance to strafe left
         */
        fun strafeLeft(distance: Double): Builder {
            i.strafeLeft(distance)

            return this
        }

        /**
         * Adds a segment that strafes right in the robot reference frame.
         *
         * @param distance distance to strafe right
         */
        fun strafeRight(distance: Double): Builder {
            i.strafeRight(distance)

            return this
        }

        /**
         * Adds a spline segment with tangent heading interpolation.
         *
         * @param endPosition end position
         * @param endTangent end tangent
         */
        fun splineTo(endPosition: Vector2d, endTangent: Double): Builder {
            i.splineTo(endPosition, endTangent)

            return this
        }

        /**
         * Adds a spline segment with constant heading interpolation.
         *
         * @param endPosition end position
         * @param endTangent end tangent
         */
        fun splineToConstantHeading(endPosition: Vector2d, endTangent: Double): Builder {
            i.splineToConstantHeading(endPosition, endTangent)

            return this
        }

        /**
         * Adds a spline segment with linear heading interpolation.
         *
         * @param endPose end pose
         * @param endTangent end tangent
         */
        fun splineToLinearHeading(endPose: Pose2d, endTangent: Double): Builder {
            i.splineToLinearHeading(endPose, endTangent)

            return this
        }

        /**
         * Adds a spline segment with spline heading interpolation.
         *
         * @param endPose end pose
         * @param endTangent end tangent
         */
        fun splineToSplineHeading(endPose: Pose2d, endTangent: Double): Builder {
            i.splineToSplineHeading(endPose, endTangent)

            return this
        }

        /**
         * Adds a marker to the trajectory at [time].
         */
        fun addTemporalMarker(time: Double, callback: MarkerCallback): Builder {
            i.addTemporalMarker(time, callback)

            return this
        }

        /**
         * Adds a marker to the trajectory at [scale] * trajectory duration + [offset].
         */
        fun addTemporalMarker(scale: Double, offset: Double, callback: MarkerCallback): Builder {
            i.addTemporalMarker(scale, offset, callback)

            return this
        }

        /**
         * Adds a marker to the trajectory at [time] evaluated with the trajectory duration.
         */
        fun addTemporalMarker(time: (Double) -> Double, callback: MarkerCallback): Builder {
            i.addTemporalMarker(time, callback)

            return this
        }

        /**
         * Adds a marker that will be triggered at the closest trajectory point to [point].
         */
        fun addSpatialMarker(point: Vector2d, callback: MarkerCallback): Builder {
            i.addSpatialMarker(point, callback)

            return this
        }

        /**
         * Adds a marker at the current position of the trajectory.
         */
        fun addDisplacementMarker(callback: MarkerCallback): Builder {
            i.addDisplacementMarker(callback)

            return this
        }

        /**
         * Adds a marker to the trajectory at [displacement].
         */
        fun addDisplacementMarker(displacement: Double, callback: MarkerCallback): Builder {
            i.addDisplacementMarker(displacement, callback)

            return this
        }

        /**
         * Adds a marker to the trajectory at [scale] * path length + [offset].
         */
        fun addDisplacementMarker(scale: Double, offset: Double, callback: MarkerCallback): Builder {
            i.addDisplacementMarker(scale, offset, callback)

            return this
        }

        /**
         * Adds a marker to the trajectory at [displacement] evaluated with path length.
         */
        fun addDisplacementMarker(displacement: (Double) -> Double, callback: MarkerCallback): Builder {
            i.addDisplacementMarker(displacement, callback)

            return this
        }
    }
}
