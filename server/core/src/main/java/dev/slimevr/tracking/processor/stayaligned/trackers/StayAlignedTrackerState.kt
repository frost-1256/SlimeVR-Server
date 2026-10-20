package dev.slimevr.tracking.processor.stayaligned.trackers

import dev.slimevr.math.Angle
import dev.slimevr.tracking.processor.stayaligned.StayAlignedDefaults
import dev.slimevr.tracking.processor.stayaligned.adjust.TrackerYaw
import dev.slimevr.tracking.trackers.Tracker
import io.github.axisangles.ktmath.Quaternion
import kotlin.time.TimeSource

class StayAlignedTrackerState(
	val tracker: Tracker,
) {
	// Whether to hide the yaw correction
	var hideCorrection = false

	// Detects whether the tracker is at rest
	val restDetector = StayAlignedDefaults.makeRestDetector()

	// Rotation of the tracker when it was locked
	var lockedRotation: Quaternion? = null

	// Raw rotation of the tracker when it was locked. Any yaw change compared to
	// it while the tracker is at rest is drift.
	var lockedRawRotation: Quaternion? = null

	// Learned raw yaw drift, applied as a feed-forward correction while moving
	val yawDrift = YawDriftEstimator()

	// Yaw correction to apply to tracker rotation
	var yawCorrection = Angle.ZERO

	// Alignment error that yaw correction attempts to minimize
	var yawErrors = YawErrors()

	private var lastUpdate = TimeSource.Monotonic.markNow()
	private var lastAdjustment = TimeSource.Monotonic.markNow()

	fun update() {
		val now = TimeSource.Monotonic.markNow()
		val dtSeconds = ((now - lastUpdate).inWholeNanoseconds / 1e9f)
			.coerceIn(0.0f, MAX_UPDATE_DT_SECONDS)
		lastUpdate = now

		val rawRotation = tracker.getRawRotation()
		restDetector.update(rawRotation)
		if (restDetector.state == RestDetector.State.AT_REST) {
			if (lockedRotation == null) {
				lockedRotation = tracker.getAdjustedRotationForceStayAligned()
				lockedRawRotation = rawRotation
				yawDrift.beginRest()
			} else {
				lockedRawRotation?.let {
					yawDrift.observe(
						TrackerYaw.yawDifference(rawRotation, it),
						dtSeconds,
					)
				}
			}
		} else {
			lockedRotation = null
			lockedRawRotation = null
		}
		yawDrift.tick(dtSeconds)
	}

	/**
	 * Seconds since Stay Aligned last adjusted this tracker. Using the measured
	 * interval instead of the server frame time keeps the correction rate right
	 * when the loop lags or the tracker list changes.
	 */
	fun elapsedSinceAdjustment(): Float {
		val now = TimeSource.Monotonic.markNow()
		val dtSeconds = ((now - lastAdjustment).inWholeNanoseconds / 1e9f)
			.coerceIn(0.0f, MAX_ADJUST_DT_SECONDS)
		lastAdjustment = now
		return dtSeconds
	}

	fun reset() {
		val now = TimeSource.Monotonic.markNow()
		restDetector.reset()
		lockedRotation = null
		lockedRawRotation = null
		yawDrift.reset()
		yawCorrection = Angle.ZERO
		yawErrors = YawErrors()
		lastUpdate = now
		lastAdjustment = now
	}

	companion object {
		/** A single update should never account for more than this. */
		private const val MAX_UPDATE_DT_SECONDS = 1.0f

		/** Guards against a starved or suspended adjustment producing a jump. */
		const val MAX_ADJUST_DT_SECONDS = 0.1f
	}
}
