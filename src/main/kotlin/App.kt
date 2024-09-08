import javafx.animation.KeyFrame
import javafx.animation.Timeline
import javafx.application.Application
import javafx.event.ActionEvent
import javafx.event.EventHandler
import javafx.scene.Group
import javafx.scene.Scene
import javafx.scene.canvas.Canvas
import javafx.scene.canvas.GraphicsContext
import javafx.scene.image.Image
import javafx.scene.layout.StackPane
import javafx.scene.paint.Color
import javafx.scene.shape.Rectangle
import javafx.stage.Stage
import javafx.util.Duration

class App : Application() {
    init {
        TrajectoryGen.createTrajectories()
    }

    private val robotRect = Rectangle(100.0, 100.0, 10.0, 10.0)
    private val startRect = Rectangle(100.0, 100.0, 10.0, 10.0)
    private val endRect = Rectangle(100.0, 100.0, 10.0, 10.0)

    private var startTime = Double.NaN
    private val trajectories = TrajectoryGenInternal.t
    private val timeouts = TrajectoryGenInternal.timeouts

    private lateinit var fieldImage: Image
    private lateinit var stage: Stage

    private var activeTrajectoryIndex = 0
    private val trajectoryDurations = trajectories.map { it.duration() }
    private var duration = trajectoryDurations.sum() + timeouts.sumByDouble { it.second }
    private val numberOfTrajectories = trajectories.size
    private var lockoutTime = 0.0
    private var firstRun = true

    companion object {
        var WIDTH = 0.0
        var HEIGHT = 0.0
    }

    override fun start(stage: Stage?) {
        this.stage = stage!!
        fieldImage = Image("/field.jpg")

        val root = Group()

        WIDTH = fieldImage.width
        HEIGHT = fieldImage.height
        GraphicsUtil.pixelsPerInch = WIDTH / GraphicsUtil.FIELD_WIDTH
        GraphicsUtil.halfFieldPixels = WIDTH / 2.0

        val canvas = Canvas(WIDTH, HEIGHT)
        val gc = canvas.graphicsContext2D
        val t1 = Timeline(KeyFrame(Duration.millis(10.0), EventHandler<ActionEvent> { run(gc) }))
        t1.cycleCount = Timeline.INDEFINITE

        stage.scene = Scene(
            StackPane(
                root
            )
        )

        root.children.addAll(canvas, startRect, endRect, robotRect)

        stage.title = "RRPathVisualizer"
        stage.isResizable = false

        println("\nTrajectoryGen.createTrajectories() generated $numberOfTrajectories " + if (numberOfTrajectories == 1) "trajectory" else "trajectories")
        for (i in 0 until numberOfTrajectories)
            println("trajectory ${i + 1} duration: ${trajectoryDurations[i]} sec")

        for (t in timeouts)
            println("timeout queued with trajectory ${t.first + 1}, duration: ${t.second} sec")

        if (numberOfTrajectories == 0 && timeouts.isEmpty())
            println("no trajectories generated! add some to TrajectoryGen.createTrajectories()")

        println("total duration: $duration sec")

        stage.show()
        t1.play()
    }

    private fun run(gc: GraphicsContext) {
        if (startTime.isNaN())
            startTime = System.currentTimeMillis() / 1000.0

        GraphicsUtil.gc = gc
        gc.drawImage(fieldImage, 0.0, 0.0)

        gc.lineWidth = GraphicsUtil.LINE_THICKNESS

        gc.globalAlpha = 0.5
        GraphicsUtil.setColor(Color.RED)
        gc.globalAlpha = 1.0

        if (trajectories.isEmpty())
            return

        val trajectory = trajectories[activeTrajectoryIndex]

        var x = 0.0
        for (i in 0 until activeTrajectoryIndex)
            x += trajectoryDurations[i] + timeouts.filter { it.first == i }.sumByDouble { it.second }
        val prevDurations: Double = x

        // Check for first run conditions matching a sleep-first timeout
        if (firstRun && activeTrajectoryIndex == 0 && timeouts.size > 0 && timeouts.first().first == -1) {
            if (lockoutTime == 0.0)
                lockoutTime = System.currentTimeMillis() / 1000.0
            // Lock starting time for first run as we don't have a trajectory to rely on
            startTime = System.currentTimeMillis() / 1000.0

            val totalTrajectories = numberOfTrajectories
            val currentClockTime = System.currentTimeMillis() / 1000.0 - lockoutTime
            val totalTimeout = timeouts.first().second
            val totalDuration = "%.2f".format(this.duration)
            stage.title = "(0/$totalTrajectories) sleeping ${"%.2f".format(currentClockTime)}/$totalTimeout, total ${
                "%.2f".format(currentClockTime)
            }/$totalDuration"

            // Timeout completion handling
            if (lockoutTime + timeouts.first().second < System.currentTimeMillis() / 1000.0)
                firstRun = false
        }

        val time = System.currentTimeMillis() / 1000.0
        val profileTime = time - startTime - prevDurations

        // Append any timeouts to the previous trajectory
        val additionalTime = timeouts.filter { it.first == activeTrajectoryIndex }.sumByDouble { it.second }
        val duration = trajectoryDurations[activeTrajectoryIndex] + additionalTime

        val start = trajectories.first().start()
        val end = trajectories.last().end()
        val current = trajectory[profileTime]

        if (profileTime >= duration) {
            activeTrajectoryIndex++
            if (activeTrajectoryIndex >= numberOfTrajectories) {
                activeTrajectoryIndex = 0
                firstRun = true
                lockoutTime = 0.0
                startTime = time
            }
        }

        trajectories.forEach { GraphicsUtil.drawSampledPath(it.path) }

        GraphicsUtil.updateRobotRect(startRect, start, GraphicsUtil.END_BOX_COLOR, 0.5)
        GraphicsUtil.updateRobotRect(endRect, end, GraphicsUtil.END_BOX_COLOR, 0.5)

        GraphicsUtil.updateRobotRect(robotRect, current, GraphicsUtil.ROBOT_COLOR, 0.75)
        GraphicsUtil.drawRobotVector(current)

        if (firstRun && timeouts.size > 0 && timeouts.first().first == -1)
            return

        val currentIndex = activeTrajectoryIndex + 1
        val totalTrajectories = numberOfTrajectories
        val currentProfileTime = "%.2f".format(profileTime)
        val totalProfileTime =
            "%.2f".format(profileTime + prevDurations + if (timeouts.isNotEmpty() && timeouts.first().first == -1) timeouts.first().second else 0.0)
        val remainingTime = "%.2f".format(profileTime - trajectoryDurations[activeTrajectoryIndex])

        stage.title = if (additionalTime > 0 && profileTime >= trajectoryDurations[activeTrajectoryIndex]) {
            "($currentIndex/$totalTrajectories) sleeping $remainingTime/${"%.2f".format(additionalTime)}, total $totalProfileTime/${
                "%.2f".format(
                    this.duration
                )
            }"
        } else {
            "($currentIndex/$totalTrajectories) current $currentProfileTime/${"%.2f".format(if (additionalTime > 0) duration - additionalTime else duration)}, total $totalProfileTime/${
                "%.2f".format(
                    this.duration
                )
            }"
        }

    }
}

fun main(args: Array<String>) {
    Application.launch(App::class.java, *args)
}