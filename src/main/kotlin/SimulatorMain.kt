import java.time.LocalDateTime
import java.time.format.DateTimeFormatter
import java.util.concurrent.TimeUnit

val SIMULATION_DURATION_NANOS = TimeUnit.SECONDS.toNanos(6)
val SIMULATION_TICK_NANOS = TimeUnit.MILLISECONDS.toNanos(2)

enum class CSVColumn(val fieldName: String) {
    SIMULATION_TIMESTAMP_SECONDS("Simulation Timestamp (s)"),

    POSITION_RAD("Position (Rad)"),
    VELOCITY_RAD_PER_S("Velocity (Rad/S)"),
    ACCEL_RAD_PER_S2("Acceleration (Rad/S^2)"),
    NET_TORQUE_NM("Net Torque (NM)"),
    JERK_RAD_PER_S3("Jerk (Rad/S^3)"),
    MOTOR_DUTY_CYCLE("Motor Duty Cycle (%)")
}

val CSV_COLUMNS = CSVColumn.values().map { it.fieldName }

fun main() {
    val currentTimeFormatted = DateTimeFormatter.ISO_DATE_TIME.format(LocalDateTime.now())

    CSVWriter("simulation_$currentTimeFormatted.csv", CSV_COLUMNS).use {
        runSimulation(it)
    }
}

fun runSimulation(csv: CSVWriter) {
    val initialPose = RotationalPointMassPose(0.3, 0.01, AngularPose())
    val motor = DCPermanentMagnetMotor(0.3, 2000.0)

    var motorMassPose = MotorAndRotationalPointMassPose(motor, initialPose)

    for (currentTimeNanos in SIMULATION_TICK_NANOS..SIMULATION_DURATION_NANOS step SIMULATION_TICK_NANOS) {
        val elapsedTimeS = SIMULATION_TICK_NANOS / TimeUnit.SECONDS.toNanos(1).toDouble()

        val dutyCycle = simulatedDutyCycleAtTime(currentTimeNanos)

        val updatePoseResult = motorMassPose.updatePose(dutyCycle, elapsedTimeS)
        motorMassPose = updatePoseResult.pose

        csv.writeRecord(mapOf(
            CSVColumn.SIMULATION_TIMESTAMP_SECONDS.fieldName to
                    currentTimeNanos / TimeUnit.SECONDS.toNanos(1).toDouble(),

            CSVColumn.POSITION_RAD.fieldName to motorMassPose.pose.positionRad,
            CSVColumn.VELOCITY_RAD_PER_S.fieldName to motorMassPose.pose.velocityRadPerS,
            CSVColumn.NET_TORQUE_NM.fieldName to updatePoseResult.computedTorqueNM,
            CSVColumn.ACCEL_RAD_PER_S2.fieldName to motorMassPose.pose.accelerationRadPerS2,
            CSVColumn.JERK_RAD_PER_S3.fieldName to motorMassPose.pose.jerkRadPerS3,
            CSVColumn.MOTOR_DUTY_CYCLE.fieldName to dutyCycle
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