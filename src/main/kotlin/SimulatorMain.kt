import algorithms.normalizeTorqueToDutyCycle
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
    MOTOR_DUTY_CYCLE("Motor Duty Cycle (%)"),
    DESIRED_TORQUE("Desired Torque (%)"),
    SETPOINT_VEL_PERC("Setpoint Velocity (%)"),
    PID_EVENT_TRIGGERED("PID Event Triggered (Bool)")
}

val CSV_COLUMNS = CSVColumn.values().map { it.fieldName }

fun main() {
    val currentTimeFormatted = DateTimeFormatter.ISO_DATE_TIME.format(LocalDateTime.now())

    CSVWriter("simulation_$currentTimeFormatted.csv", CSV_COLUMNS).use {
        runSimulation(it)
    }
}

const val maxVelRadPerS = 2000.0

const val velocityErrorkP = 10 // Request 100% torque per 10% velocity error
const val velocitykF = 0.05/0.5 // 5% stall torque to balance drag at 50% speed

const val torqueLimitPercent = 1.0 // Never apply more percent torque than this

fun runSimulation(csv: CSVWriter) {
    val initialPose = RotationalPointMassPose(0.3, 0.01, AngularPose())
    val motor = DCPermanentMagnetMotor(0.3, maxVelRadPerS)

    var motorMassPose = MotorAndRotationalPointMassPose(motor, initialPose)

    var setpointVelocityPercent = initialPose.pose.velocityRadPerS / maxVelRadPerS
    var dutyCycle = 0.0

    val discreteUpdateTimer = PeriodicEvent(0.020) {
        val percentError = setpointVelocityPercent - (motorMassPose.pose.velocityRadPerS / maxVelRadPerS)
        val velErrorReqTorquePercent = (percentError * velocityErrorkP)

        val dragReqTorquePercent = velocitykF * setpointVelocityPercent

        (velErrorReqTorquePercent + dragReqTorquePercent).coerceIn(-torqueLimitPercent, torqueLimitPercent)
    }
    discreteUpdateTimer.handleSimulationTick(0)

    for (currentTimeNanos in SIMULATION_TICK_NANOS..SIMULATION_DURATION_NANOS step SIMULATION_TICK_NANOS) {
        val elapsedTimeS = SIMULATION_TICK_NANOS / TimeUnit.SECONDS.toNanos(1).toDouble()
        val currentTimeS = currentTimeNanos / TimeUnit.SECONDS.toNanos(1).toDouble()

        setpointVelocityPercent = simulatedSetpointPercentMaxVelAtTime(currentTimeS)

        val output = discreteUpdateTimer.handleSimulationTick(currentTimeNanos)
        if (output != null) {
            dutyCycle = normalizeTorqueToDutyCycle(
                maxVelocityRadPerS = maxVelRadPerS,
                currentVelocityRadPerS = motorMassPose.pose.velocityRadPerS,
                desiredTorquePercent = output)
        }

        val updatePoseResult = motorMassPose.updatePose(dutyCycle, elapsedTimeS)
        motorMassPose = updatePoseResult.pose

        csv.writeRecord(mapOf(
            CSVColumn.SIMULATION_TIMESTAMP_SECONDS.fieldName to
                    currentTimeS,

            CSVColumn.POSITION_RAD.fieldName to motorMassPose.pose.positionRad,
            CSVColumn.VELOCITY_RAD_PER_S.fieldName to motorMassPose.pose.velocityRadPerS,
            CSVColumn.NET_TORQUE_NM.fieldName to updatePoseResult.computedTorqueNM,
            CSVColumn.ACCEL_RAD_PER_S2.fieldName to motorMassPose.pose.accelerationRadPerS2,
            CSVColumn.JERK_RAD_PER_S3.fieldName to motorMassPose.pose.jerkRadPerS3,
            CSVColumn.MOTOR_DUTY_CYCLE.fieldName to dutyCycle,
            CSVColumn.DESIRED_TORQUE.fieldName to output,
            CSVColumn.SETPOINT_VEL_PERC.fieldName to setpointVelocityPercent,
            CSVColumn.PID_EVENT_TRIGGERED.fieldName to if (output != null) 1.0 else 0.0
        ))
    }
}

fun simulatedSetpointPercentMaxVelAtTime(currentTimeS: Double): Double {
    return if (currentTimeS >= 1)
        0.5
    else
        0.0
}