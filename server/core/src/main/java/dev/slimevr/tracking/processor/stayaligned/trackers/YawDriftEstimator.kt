package dev.slimevr.tracking.processor.stayaligned.trackers

import dev.slimevr.math.Angle
import kotlin.math.abs
import kotlin.math.min

/**
 * Learns a tracker's yaw drift rate while it is at rest and turns it into a
 * feed-forward correction for when the tracker moves.
 *
 * A tracker supported by a surface cannot rotate on its own, so any yaw change
 * measured while it is at rest is drift. Stay Aligned can only *react* to drift
 * through pose errors, which is slow while the player moves. This estimator
 * measures the drift directly during rest and cancels it at the measured rate
 * once the tracker moves again.
 *
 * Samples are averaged over [WINDOW_SECONDS], because per-tick yaw changes are
 * dominated by quantisation noise (a 0.01 deg step at 1 kHz looks like 10 deg/s).
 * Windows with implausible drift are rejected as slips or motion, and an
 * estimate that has not been refreshed for [STALE_SECONDS] is ignored.
 */
class YawDriftEstimator {
	/** Raw yaw drift rate in radians per second (positive = yaw increasing). */
	var rate = 0.0f
		private set

	/** Number of accepted windows. */
	var samples = 0
		private set

	private var lastError: Angle? = null
	private var windowDelta = 0.0f
	private var windowTime = 0.0f
	private var timeSinceSample = 0.0f

	/** Whether there is a usable drift estimate. */
	fun hasEstimate() = samples > 0 && rate != 0.0f

	/** Forgets the estimate, e.g. after a reset. */
	fun reset() {
		rate = 0.0f
		samples = 0
		lastError = null
		windowDelta = 0.0f
		windowTime = 0.0f
		timeSinceSample = 0.0f
	}

	/**
	 * Starts a fresh measurement window. Called when the tracker locks, because the
	 * previous window refers to the previous locked rotation.
	 */
	fun beginRest() {
		lastError = null
		windowDelta = 0.0f
		windowTime = 0.0f
	}

	/**
	 * Advances staleness tracking. Called every tick, also while moving.
	 */
	fun tick(dtSeconds: Float) {
		if (dtSeconds > 0.0f) {
			timeSinceSample += dtSeconds
		}
	}

	/**
	 * Feeds the yaw of the raw rotation relative to the rotation captured when the
	 * tracker locked. That error is the raw yaw drift itself in the convention of
	 * [TrackerYaw.yawDifference], so a positive slope means the raw yaw is drifting
	 * positive.
	 *
	 * Partial windows are dropped when the tracker starts moving: they mix drift
	 * with real motion and would bias the estimate.
	 */
	fun observe(error: Angle, dtSeconds: Float) {
		if (dtSeconds <= 0.0f) {
			return
		}
		val previous = lastError
		lastError = error
		if (previous == null || dtSeconds > MAX_DT_SECONDS) {
			windowDelta = 0.0f
			windowTime = 0.0f
			return
		}

		windowDelta += (error - previous).toRad()
		windowTime += dtSeconds
		if (windowTime < WINDOW_SECONDS) {
			return
		}

		val observedRate = windowDelta / windowTime
		if (abs(observedRate) <= MAX_RATE) {
			// Exponential moving average so a single noisy window cannot dominate.
			val alpha = min(1.0f, windowTime / TIME_CONSTANT_SECONDS)
			rate = if (samples == 0) observedRate else rate + (observedRate - rate) * alpha
			samples++
			timeSinceSample = 0.0f
		}
		windowDelta = 0.0f
		windowTime = 0.0f
	}

	/**
	 * Change to add to the tracker's yaw offset after [dtSeconds] of movement.
	 *
	 * The offset is pre-multiplied onto the raw rotation, so cancelling a positive
	 * raw drift means moving the offset in the negative direction.
	 */
	fun correction(dtSeconds: Float): Angle {
		if (!hasEstimate() || dtSeconds <= 0.0f || timeSinceSample > STALE_SECONDS) {
			return Angle.ZERO
		}
		return Angle.ofRad(-rate * min(dtSeconds, MAX_DT_SECONDS))
	}

	companion object {
		/** Sampling window, long enough to average out yaw quantisation. */
		const val WINDOW_SECONDS = 2.0f

		/** Time constant of the rate average. */
		const val TIME_CONSTANT_SECONDS = 8.0f

		/**
		 * Larger per-window rates are slips or motion, not drift (~21 deg/min).
		 * Stay Aligned already corrects bad IMUs at up to 24 deg/min, so this
		 * covers the trackers the heuristic is meant to help.
		 */
		const val MAX_RATE = 0.006f

		/** Ignore estimates that no rest period refreshed for half an hour. */
		const val STALE_SECONDS = 1800.0f

		/** Guards against a suspended server producing one huge step. */
		const val MAX_DT_SECONDS = 1.0f
	}
}
