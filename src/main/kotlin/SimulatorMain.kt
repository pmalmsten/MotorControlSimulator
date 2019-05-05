import java.time.LocalDateTime
import java.time.format.DateTimeFormatter
import java.util.concurrent.TimeUnit

val SIMULATION_DURATION_NANOS = TimeUnit.SECONDS.toNanos(6)
val SIMULATION_TICK_NANOS = TimeUnit.MILLISECONDS.toNanos(2)

enum class CSVColumn(val fieldName: String) {
    SIMULATION_TIMESTAMP_SECONDS("Simulation Timestamp (s)"),

    POSITION_DEG("Position (Deg)"),
    VELOCITY_DEG_PER_S("Velocity (Deg/S)"),
    ACCEL_DEG_PER_S2("Acceleration (Deg/S^2"),
    JERK_DEG_PER_S3("Jerk (Deg/S^3)")
}

val CSV_COLUMNS = CSVColumn.values().map { it.fieldName }

fun main(args: Array<String>) {
    val currentTimeFormatted = DateTimeFormatter.ISO_DATE_TIME.format(LocalDateTime.now())

    CSVWriter("simulation_$currentTimeFormatted.csv", CSV_COLUMNS).use {
        runSimulation(it)
    }
}

fun runSimulation(csv: CSVWriter) {
    var pointMassPose = RotationalPointMassPose(0.3, 0.01, AngularPose())
    val motor = DCPermanentMagnetMotor(0.3, 2000.0)

    for (currentTimeNanos in SIMULATION_TICK_NANOS..SIMULATION_DURATION_NANOS step SIMULATION_TICK_NANOS) {

        val motorTorqueNM = motor.computeTorque(
            pointMassPose.pose.velocityDegPerS,
            simulatedDutyCycleAtTime(currentTimeNanos))

        val netTorqueNM = motorTorqueNM -
                simulatedFrictionAtVelocity(pointMassPose.pose.velocityDegPerS)

        pointMassPose = pointMassPose.updatePose(
            netTorqueNM,
            SIMULATION_TICK_NANOS / TimeUnit.SECONDS.toNanos(1).toDouble())

        csv.writeRecord(mapOf(
            CSVColumn.SIMULATION_TIMESTAMP_SECONDS.fieldName to
                    currentTimeNanos / TimeUnit.SECONDS.toNanos(1).toDouble(),

            CSVColumn.POSITION_DEG.fieldName to pointMassPose.pose.positionDegrees,
            CSVColumn.VELOCITY_DEG_PER_S.fieldName to pointMassPose.pose.velocityDegPerS,
            CSVColumn.ACCEL_DEG_PER_S2.fieldName to pointMassPose.pose.accelerationDegPerS2,
            CSVColumn.JERK_DEG_PER_S3.fieldName to pointMassPose.pose.jerkDegPerS3
        ))
    }
}

fun simulatedDutyCycleAtTime(currentTimeNanos: Long): Double {
    return if (currentTimeNanos >= TimeUnit.SECONDS.toNanos(1) &&
            currentTimeNanos <= TimeUnit.SECONDS.toNanos(5))
        1.0
    else
        0.0
}

fun simulatedTorqueAtTime(currentTimeNanos: Long): Double {
    return if (currentTimeNanos >= TimeUnit.SECONDS.toNanos(1) &&
            currentTimeNanos <= TimeUnit.SECONDS.toNanos(5))
        0.2
    else
        0.0
}

fun simulatedFrictionAtVelocity(currentVelocity: Double): Double {
    return currentVelocity * 0.0001
}