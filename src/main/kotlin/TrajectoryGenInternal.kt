import TrajectoryGen.*
import bunyipslib.external.units.*
import bunyipslib.external.units.Units.*
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.*
import com.acmerobotics.roadrunner.util.Angle.norm


/**
 * Port of BunyipsLib RoadRunner `makeTrajectory()` features and a hacky port of TrajectorySequence
 * @author Lucas Bubner, 2023
 */
@Suppress("unused")
open class TrajectoryGenInternal {
    companion object {
        private const val TURN_OFFSET = 0.01

        @JvmStatic
        val t = ArrayList<Trajectory>()

        @JvmStatic
        val timeouts = ArrayList<Pair<Int, Double>>()

        @JvmStatic
        private var lastPose: Pose2d = Pose2d()

        @JvmStatic
        protected fun makeTrajectory() = Builder(
            lastPose,
            lastPose.heading,
            atVelocity(maxVel.magnitude(), maxVel.unit()),
            atAcceleration(maxAccel.magnitude(), maxAccel.unit())
        )

        @JvmStatic
        protected fun makeTrajectory(pose: Pose2d) = Builder(
            pose,
            pose.heading,
            atVelocity(maxVel.inUnit(InchesPerSecond), InchesPerSecond),
            atAcceleration(maxAccel.inUnit(InchesPerSecond.per(Second)), InchesPerSecond.per(Second))
        )

        @JvmStatic
        protected fun makeTrajectory(startPose: Pose2d, inUnit: Distance, angleUnit: Angle): Builder {
            val x = Inches.convertFrom(startPose.x, inUnit)
            val y = Inches.convertFrom(startPose.y, inUnit)
            val r = Radians.convertFrom(startPose.heading, angleUnit)
            return makeTrajectory(Pose2d(x, y, r))
        }

        @JvmStatic
        protected fun atVelocity(translation: Double, unit: Velocity<Distance>): TrajectoryVelocityConstraint {
            return MinVelocityConstraint(
                listOf(
                    AngularVelocityConstraint(maxAngularVel.inUnit(RadiansPerSecond)),
                    MecanumVelocityConstraint(unit.of(translation).inUnit(InchesPerSecond), trackWidth.inUnit(Inches))
                )
            )
        }

        @JvmStatic
        protected fun atAngularVelocity(angularVelocity: Double, unit: Velocity<Angle>): TrajectoryVelocityConstraint {
            return MinVelocityConstraint(
                listOf(
                    AngularVelocityConstraint(unit.of(angularVelocity).inUnit(Radians.per(Second))),
                    MecanumVelocityConstraint(maxVel.inUnit(InchesPerSecond), trackWidth.inUnit(Inches))
                )
            )
        }

        @JvmStatic
        protected fun atVelocities(
            translationalVelocity: Double,
            translationalVelocityUnit: Velocity<Distance>,
            angularVelocity: Double,
            angularVelocityUnit: Velocity<Angle>
        ): TrajectoryVelocityConstraint {
            return MinVelocityConstraint(
                listOf(
                    AngularVelocityConstraint(angularVelocityUnit.of(angularVelocity).inUnit(Radians.per(Second))),
                    MecanumVelocityConstraint(
                        translationalVelocityUnit.of(translationalVelocity).inUnit(Inches.per(Second)),
                        trackWidth.inUnit(Inches)
                    )
                )
            )
        }

        @JvmStatic
        protected fun atAcceleration(
            acceleration: Double,
            unit: Velocity<Velocity<Distance>>
        ): TrajectoryAccelerationConstraint {
            return ProfileAccelerationConstraint(unit.of(acceleration).inUnit(InchesPerSecond.per(Second)))
        }

        @JvmStatic
        protected fun unitPose(pose: Pose2d, distanceUnit: Distance, angleUnit: Angle): Pose2d {
            return Pose2d(
                Inches.convertFrom(pose.x, distanceUnit),
                Inches.convertFrom(pose.y, distanceUnit),
                Radians.convertFrom(pose.heading, angleUnit)
            )
        }

        @JvmStatic
        protected fun unitVec(vector: Vector2d, unit: Distance): Vector2d {
            return Vector2d(
                Inches.convertFrom(vector.x, unit),
                Inches.convertFrom(vector.y, unit)
            )
        }
    }

    class Builder(
        pose: Pose2d,
        heading: Double,
        velocityConstraint: TrajectoryVelocityConstraint,
        accelerationConstraint: TrajectoryAccelerationConstraint
    ) {
        private var i: TrajectoryBuilder = TrajectoryBuilder(pose, heading, velocityConstraint, accelerationConstraint)
        private var r: TrajectoryBuilder = TrajectoryBuilder(pose, heading, velocityConstraint, accelerationConstraint)

        /**
         * Add this trajectory to the queue.
         * This method is the only one left to be implemented from BunyipsLib,
         * as advanced task construction is not supported in this visualiser.
         */
        fun addTask(): Trajectory? {
            try {
                val trajectory = i.build()
                t.add(trajectory)
                lastPose = trajectory.end()
                i = TrajectoryBuilder(
                    lastPose,
                    lastPose.heading,
                    atVelocity(maxVel.magnitude(), maxVel.unit()),
                    atAcceleration(maxAccel.magnitude(), maxAccel.unit())
                )
                return trajectory
            } catch (e: NoSuchElementException) {
                // Trajectory is empty, will return null to avoid errors
                return null
            }
        }

        fun waitFor(time: Measure<Time>): Builder {
            return waitSeconds(time.inUnit(Seconds))
        }

        fun waitFor(time: Double, unit: Time): Builder {
            return waitSeconds(unit.of(time).inUnit(Seconds))
        }

        fun waitSeconds(seconds: Double): Builder {
            if (seconds < 0.0)
                throw IllegalArgumentException("seconds must be positive")

            // Need to early build to update queue timing
            addTask()

            timeouts.add(Pair(t.size - 1, seconds))
            if (t.size == 0)
                println("waitSeconds($seconds) is not natively supported, queued start wait")
            else
                println("waitSeconds($seconds) is not natively supported, queued extended idle duration for trajectory ${t.size + 1}")
            return this
        }

        fun turn(angle: Double, angleUnit: Angle): Builder {
            val a = Radians.convertFrom(angle, angleUnit)
            return turn(a)
        }

        fun turn(angle: Double): Builder {
            // Need to early build and reset to get the end pose and to split the trajectory
            addTask()

            // We can't turn naturally, so we will instead translate the turn into a lineToLinearHeading mock trajectory
            // This trajectory has a small length to not raise an empty trajectory error, which is a bit of a hack
            // However, this offset will be reversed when the next trajectory is constructed
            val newPose = Pose2d(
                lastPose.vec().plus(Vector2d(TURN_OFFSET, 0.0)),
                norm(lastPose.heading + angle)
            )

            makeTrajectory()
                .lineToLinearHeading(newPose)
                .addTask()

            // Since we have inserted a trajectory with modified heading we need to reset the builder
            // We can remove the offset here as well, effectively reversing any inaccuracies
            i = TrajectoryBuilder(
                Pose2d(newPose.vec().minus(Vector2d(TURN_OFFSET, 0.0)), newPose.heading),
                newPose.heading,
                atVelocities(
                    maxVel.magnitude(),
                    maxVel.unit(),
                    maxAngularVel.magnitude(),
                    maxAngularVel.unit()
                ),
                atAcceleration(
                    maxAccel.magnitude(),
                    maxAccel.unit()
                )
            )

            println("turn($angle) has been translated into trajectory ${t.size}, as this is not a native method")
            return this
        }

        fun addTrajectory(trajectory: Trajectory): Builder {
            println("addTrajectory(${trajectory.start()} -> ${trajectory.end()}, ${trajectory.duration()} sec) called on trajectory ${t.size + 1}, added as a new trajectory.")
            // We can't attach the trajectory to the sequence, but we can instead add it to the trajectories list
            t.add(trajectory)
            if (autoAddTaskBetweenTrajectories) addTask()
            return this
        }

        fun lineTo(endPosition: Vector2d): Builder {
            i.lineTo(endPosition)
            if (autoAddTaskBetweenTrajectories) addTask()

            return this
        }

        fun lineTo(endPosition: Vector2d, inUnit: Distance): Builder {
            val x = Inches.convertFrom(endPosition.x, inUnit)
            val y = Inches.convertFrom(endPosition.y, inUnit)
            return lineTo(Vector2d(x, y))
        }

        /**
         * Adds a line segment with constant heading interpolation.
         *
         * @param endPosition end position
         */
        fun lineToConstantHeading(endPosition: Vector2d): Builder {
            i.lineToConstantHeading(endPosition)
            if (autoAddTaskBetweenTrajectories) addTask()

            return this
        }

        fun lineToConstantHeading(endPosition: Vector2d, inUnit: Distance): Builder {
            val x = Inches.convertFrom(endPosition.x, inUnit)
            val y = Inches.convertFrom(endPosition.y, inUnit)
            return lineToConstantHeading(Vector2d(x, y))
        }

        /**
         * Adds a line segment with linear heading interpolation.
         *
         * @param endPose end pose
         */
        fun lineToLinearHeading(endPose: Pose2d): Builder {
            i.lineToLinearHeading(endPose)
            if (autoAddTaskBetweenTrajectories) addTask()

            return this
        }

        fun lineToLinearHeading(endPose: Pose2d, distanceUnit: Distance, angleUnit: Angle): Builder {
            val x = Inches.convertFrom(endPose.x, distanceUnit)
            val y = Inches.convertFrom(endPose.y, distanceUnit)
            val r = Radians.convertFrom(endPose.heading, angleUnit)
            return lineToLinearHeading(Pose2d(x, y, r))
        }

        /**
         * Adds a line segment with spline heading interpolation.
         *
         * @param endPose end pose
         */
        fun lineToSplineHeading(endPose: Pose2d): Builder {
            i.lineToSplineHeading(endPose)
            if (autoAddTaskBetweenTrajectories) addTask()

            return this
        }

        fun lineToSplineHeading(endPose: Pose2d, distanceUnit: Distance, angleUnit: Angle): Builder {
            val x = Inches.convertFrom(endPose.x, distanceUnit)
            val y = Inches.convertFrom(endPose.y, distanceUnit)
            val r = Radians.convertFrom(endPose.heading, angleUnit)
            return lineToSplineHeading(Pose2d(x, y, r))
        }

        /**
         * Adds a strafe path segment.
         *
         * @param endPosition end position
         */
        fun strafeTo(endPosition: Vector2d): Builder {
            i.strafeTo(endPosition)
            if (autoAddTaskBetweenTrajectories) addTask()

            return this
        }

        fun strafeTo(endPosition: Vector2d, inUnit: Distance): Builder {
            val x = Inches.convertFrom(endPosition.x, inUnit)
            val y = Inches.convertFrom(endPosition.y, inUnit)
            return strafeTo(Vector2d(x, y))
        }

        /**
         * Adds a line straight forward.
         *
         * @param distance distance to travel forward
         */
        fun forward(distance: Double): Builder {
            i.forward(distance)
            if (autoAddTaskBetweenTrajectories) addTask()

            return this
        }

        fun forward(distance: Double, inUnit: Distance): Builder {
            val d = Inches.convertFrom(distance, inUnit)
            return forward(d)
        }

        /**
         * Adds a line straight backward.
         *
         * @param distance distance to travel backward
         */
        fun back(distance: Double): Builder {
            i.back(distance)
            if (autoAddTaskBetweenTrajectories) addTask()

            return this
        }

        fun back(distance: Double, inUnit: Distance): Builder {
            val d = Inches.convertFrom(distance, inUnit)
            return back(d)
        }

        /**
         * Adds a segment that strafes left in the robot reference frame.
         *
         * @param distance distance to strafe left
         */
        fun strafeLeft(distance: Double): Builder {
            i.strafeLeft(distance)
            if (autoAddTaskBetweenTrajectories) addTask()

            return this
        }

        fun strafeLeft(distance: Double, inUnit: Distance): Builder {
            val d = Inches.convertFrom(distance, inUnit)
            return strafeLeft(d)
        }

        /**
         * Adds a segment that strafes right in the robot reference frame.
         *
         * @param distance distance to strafe right
         */
        fun strafeRight(distance: Double): Builder {
            i.strafeRight(distance)
            if (autoAddTaskBetweenTrajectories) addTask()

            return this
        }

        fun strafeRight(distance: Double, inUnit: Distance): Builder {
            val d = Inches.convertFrom(distance, inUnit)
            return strafeRight(d)
        }

        /**
         * Adds a spline segment with tangent heading interpolation.
         *
         * @param endPosition end position
         * @param endTangent end tangent
         */
        fun splineTo(endPosition: Vector2d, endTangent: Double): Builder {
            i.splineTo(endPosition, endTangent)
            if (autoAddTaskBetweenTrajectories) addTask()

            return this
        }

        fun splineTo(endPosition: Vector2d, inUnit: Distance, endHeading: Double, angleUnit: Angle): Builder {
            val x = Inches.convertFrom(endPosition.x, inUnit)
            val y = Inches.convertFrom(endPosition.y, inUnit)
            val r = Radians.convertFrom(endHeading, angleUnit)
            return splineTo(Vector2d(x, y), r)
        }

        /**
         * Adds a spline segment with constant heading interpolation.
         *
         * @param endPosition end position
         * @param endTangent end tangent
         */
        fun splineToConstantHeading(endPosition: Vector2d, endTangent: Double): Builder {
            i.splineToConstantHeading(endPosition, endTangent)
            if (autoAddTaskBetweenTrajectories) addTask()

            return this
        }

        fun splineToConstantHeading(
            endPosition: Vector2d,
            inUnit: Distance,
            endHeading: Double,
            angleUnit: Angle
        ): Builder {
            val x = Inches.convertFrom(endPosition.x, inUnit)
            val y = Inches.convertFrom(endPosition.y, inUnit)
            val r = Radians.convertFrom(endHeading, angleUnit)
            return splineToConstantHeading(Vector2d(x, y), r)
        }

        /**
         * Adds a spline segment with linear heading interpolation.
         *
         * @param endPose end pose
         * @param endTangent end tangent
         */
        fun splineToLinearHeading(endPose: Pose2d, endTangent: Double): Builder {
            i.splineToLinearHeading(endPose, endTangent)
            if (autoAddTaskBetweenTrajectories) addTask()

            return this
        }

        fun splineToLinearHeading(
            endPose: Pose2d,
            distanceUnit: Distance,
            angleUnit: Angle,
            endHeading: Double,
            endAngleUnit: Angle
        ): Builder {
            val x = Inches.convertFrom(endPose.x, distanceUnit)
            val y = Inches.convertFrom(endPose.y, distanceUnit)
            val r = Radians.convertFrom(endHeading, angleUnit)
            val r2 = Radians.convertFrom(endPose.heading, endAngleUnit)
            return splineToLinearHeading(Pose2d(x, y, r), r2)
        }

        /**
         * Adds a spline segment with spline heading interpolation.
         *
         * @param endPose end pose
         * @param endTangent end tangent
         */
        fun splineToSplineHeading(endPose: Pose2d, endTangent: Double): Builder {
            i.splineToSplineHeading(endPose, endTangent)
            if (autoAddTaskBetweenTrajectories) addTask()

            return this
        }

        fun splineToSplineHeading(
            endPose: Pose2d,
            distanceUnit: Distance,
            angleUnit: Angle,
            endHeading: Double,
            endAngleUnit: Angle
        ): Builder {
            val x = Inches.convertFrom(endPose.x, distanceUnit)
            val y = Inches.convertFrom(endPose.y, distanceUnit)
            val r = Radians.convertFrom(endHeading, angleUnit)
            val r2 = Radians.convertFrom(endPose.heading, endAngleUnit)
            return splineToSplineHeading(Pose2d(x, y, r), r2)
        }

        // These methods below are taken straight from the builder and are not from the trajectory sequence,
        // therefore they might not reflect the actual implementation for a real robot

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
