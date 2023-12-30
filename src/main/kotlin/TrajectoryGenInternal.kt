import TrajectoryGen.accelConstraint
import TrajectoryGen.velConstraint
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.*
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint

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
        protected fun addNewTrajectory() = Builder(lastPose, lastPose.heading, velConstraint, accelConstraint)

        @JvmStatic
        protected fun addNewTrajectory(pose: Pose2d) = Builder(pose, pose.heading, velConstraint, accelConstraint)
    }

    class Builder(pose: Pose2d, heading: Double, velocityConstraint: TrajectoryVelocityConstraint, accelerationConstraint: TrajectoryAccelerationConstraint) {
        private val i: TrajectoryBuilder = TrajectoryBuilder(pose, heading, velocityConstraint, accelerationConstraint)

        fun setTangent(tangent: Double): Builder {
            println("setTangent($tangent) called on trajectory ${t.size + 1}, not supported!")
            return this
        }

        fun setReversed(reversed: Boolean): Builder {
            println("setReversed($reversed) called on trajectory ${t.size + 1}, not supported!")
            return this
        }

        fun setConstraints(
            velConstraint: TrajectoryVelocityConstraint,
            accelConstraint: TrajectoryAccelerationConstraint
        ): Builder {
            println("setConstraints($velConstraint, $accelConstraint) called on trajectory ${t.size + 1}, not supported!")
            return this
        }

        fun resetConstraints(): Builder {
            println("resetConstraints() called on trajectory ${t.size + 1}, not supported!")
            return this
        }

        fun setVelConstraint(velConstraint: TrajectoryVelocityConstraint): Builder {
            println("setVelConstraint($velConstraint) called on trajectory ${t.size + 1}, not supported!")
            return this
        }

        fun resetVelConstraint(): Builder {
            println("resetVelConstraint() called on trajectory ${t.size + 1}, not supported!")
            return this
        }

        fun setAccelConstraint(accelConstraint: TrajectoryAccelerationConstraint): Builder {
            println("setAccelConstraint($accelConstraint) called on trajectory ${t.size + 1}, not supported!")
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

        fun UNSTABLE_addTemporalMarkerOffset(offset: Double, callback: MarkerCallback): Builder {
            println("UNSTABLE_addTemporalMarkerOffset($offset, $callback) called on trajectory ${t.size + 1}, not supported!")
            return this
        }

        fun UNSTABLE_addDisplacementMarkerOffset(offset: Double, callback: MarkerCallback): Builder {
            println("UNSTABLE_addDisplacementMarkerOffset($offset, $callback) called on trajectory ${t.size + 1}, not supported!")
            return this
        }

        fun turn(angle: Double): Builder {
            println("turn($angle) called on trajectory ${t.size + 1}, not supported!")
            return this
        }

        fun turn(angle: Double, maxAngVel: Double, maxAngAccel: Double): Builder {
            println("turn($angle, $maxAngVel, $maxAngAccel) called on trajectory ${t.size + 1}, not supported!")
            return this
        }

        fun waitSeconds(seconds: Double): Builder {
            println("waitSeconds($seconds) called on trajectory ${t.size + 1}, not supported!")
            return this
        }

        fun addTrajectory(trajectory: Trajectory): Builder {
            println("addTrajectory($trajectory) called on trajectory ${t.size + 1}, not supported!")
            return this
        }

        fun build(): Trajectory {
            val trajectory = i.build()
            lastPose = trajectory.end()
            t.add(trajectory)
            return trajectory
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
        fun addTemporalMarker(time: Double, callback: MarkerCallback) =
            addTemporalMarker(0.0, time, callback)

        /**
         * Adds a marker to the trajectory at [scale] * trajectory duration + [offset].
         */
        fun addTemporalMarker(scale: Double, offset: Double, callback: MarkerCallback) =
            addTemporalMarker({ scale * it + offset }, callback)

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
        fun addDisplacementMarker(callback: MarkerCallback) = i.addDisplacementMarker(callback)

        /**
         * Adds a marker to the trajectory at [displacement].
         */
        fun addDisplacementMarker(displacement: Double, callback: MarkerCallback) =
            addDisplacementMarker(0.0, displacement, callback)

        /**
         * Adds a marker to the trajectory at [scale] * path length + [offset].
         */
        fun addDisplacementMarker(scale: Double, offset: Double, callback: MarkerCallback) =
            addDisplacementMarker({ scale * it + offset }, callback)

        /**
         * Adds a marker to the trajectory at [displacement] evaluated with path length.
         */
        fun addDisplacementMarker(displacement: (Double) -> Double, callback: MarkerCallback): Builder {
            i.addDisplacementMarker(displacement, callback)

            return this
        }
    }
}
