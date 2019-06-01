package algorithms

import kotlin.math.abs
import kotlin.math.sign

fun normalizeTorqueToDutyCycle(maxVelocityRadPerS: Double,
                               currentVelocityRadPerS: Double,
                               desiredTorquePercent: Double): Double {

    require(maxVelocityRadPerS > 0) { "maxVelocity must not be zero" }

    val isReversing = sign(currentVelocityRadPerS) != 0.0 &&
            sign(desiredTorquePercent) != 0.0 &&
            sign(currentVelocityRadPerS) != sign(desiredTorquePercent)

    val directionalFactor = if (isReversing) -1 else 1

    val percentMaxVelocity = abs(currentVelocityRadPerS/maxVelocityRadPerS)
    val maxAvailableMotorTorquePercent = 1 - (directionalFactor * percentMaxVelocity)

    val boostFactor = 1 / maxAvailableMotorTorquePercent

    return (desiredTorquePercent * boostFactor).coerceIn(-1.0, 1.0)
}