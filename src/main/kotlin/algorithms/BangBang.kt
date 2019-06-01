package algorithms

import kotlin.math.abs
import kotlin.math.sign

fun bangbangDesiredTorque(setpoint: Double,
                          actual: Double,
                          tolerance: Double,
                          torqueToApply: Double): Double {

    val delta = setpoint - actual
    return if (abs(delta) > tolerance) {
        sign(delta) * torqueToApply
    } else 0.0
}