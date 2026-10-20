package dev.slimevr.tracking.trackers

import com.jme3.math.FastMath
import dev.slimevr.VRServer
import dev.slimevr.config.ArmsResetModes
import dev.slimevr.config.DriftCompensationConfig
import dev.slimevr.config.ResetsConfig
import dev.slimevr.filtering.CircularArrayList
import dev.slimevr.recording.LiveDataRecorder
import dev.slimevr.tracking.trackers.udp.TrackerDataType
import io.github.axisangles.ktmath.EulerAngles
import io.github.axisangles.ktmath.EulerOrder
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import kotlin.math.*

private const val DRIFT_COOLDOWN_MS = 50000L

/** Class taking care of full reset, yaw reset, mounting reset, and drift compensation logic. */
class TrackerResetsHandler(val tracker: Tracker) {

	private val HalfHorizontal = EulerAngles(
		EulerOrder.YZX,
		0f,
		Math.PI.toFloat(),
		0f,
	).toQuaternion()
	private var driftAmount = 0f
	private var averagedDriftQuat = Quaternion.IDENTITY
	private var rotationSinceReset = Quaternion.IDENTITY
	private var driftQuats = CircularArrayList<Quaternion>(0)
	private var driftTimes = CircularArrayList<Long>(0)
	private var totalDriftTime: Long = 0
	private var driftSince: Long = 0
	private var timeAtLastReset: Long = 0
	private var compensateDrift = false
	private var driftPrediction = false
	private var driftCompensationEnabled = false
	private var armsResetMode = ArmsResetModes.BACK
	private var yawResetSmoothTime = 0.0f
	var saveMountingReset = false
	var resetHmdPitch = false
	var allowDriftCompensation = false
	var lastResetQuaternion: Quaternion? = null

	/** Averaged drift correction, recorded for live data analysis. */
	val driftQuaternion: Quaternion
		get() = averagedDriftQuat

	/** Rotation accumulated since the last reset, recorded for live data analysis. */
	val rotationSinceLastReset: Quaternion
		get() = rotationSinceReset

	/** Fraction of the drift correction currently applied. */
	val appliedDriftAmount: Float
		get() = driftAmount

	/** Whether drift compensation is currently applied to this tracker. */
	val driftCompensationActive: Boolean
		get() = driftCompensationEnabled

	// Manual mounting orientation
	var mountingOrientation = HalfHorizontal
		set(value) {
			field = value
			// Clear the mounting reset now that it's been set manually
			clearMounting()
		}

	// Reference adjustment quats

	/**
	 * Gyro fix is set by full reset. This sets the current y rotation to 0, correcting
	 * for initial yaw rotation and the rotation incurred by mounting orientation. This
	 * is a local offset in rotation and does not affect the axes of rotation.
	 *
	 * This rotation is only used to compute [attachmentFix], otherwise [yawFix] would
	 * correct for the same rotation.
	 */
	private var gyroFix = Quaternion.IDENTITY

	/**
	 * Attachment fix is set by full reset. This sets the current x and z rotations to
	 * 0, correcting for initial pitch and roll rotation. This is a global offset in
	 * rotation and affects the axes of rotation.
	 *
	 * This effectively sets the rotation at the moment of a full reset to be
	 * zero-reference in the x and z axes.
	 */
	private var attachmentFix = Quaternion.IDENTITY

	/**
	 * Mounting rotation fix is set by mounting reset. This corrects for the mounting
	 * orientation, then the inverse is used to correct for the rotation incurred. This
	 * value is computed after [yawFix], but takes effect before [yawFix]. This affects
	 * the axes of rotation, but does not incur an offset in rotation.
	 *
	 * This rotation is done in addition to [mountingOrientation] as to not interfere
	 * with the functionality of manual mounting orientation. This effectively sets the
	 * rotation at the moment of a mounting reset to be zero-reference in the y-axis. If
	 * no mounting reset is done, then this rotation will not be used and only
	 * [mountingOrientation] will apply.
	 */
	var mountRotFix = Quaternion.IDENTITY
		private set

	/**
	 * Yaw fix is set by yaw reset. This sets the current y rotation to match the
	 * provided reference, correlating the tracker to the provided frame of reference.
	 * This is a local offset in rotation and does not affect the axes of rotation.
	 *
	 * This effectively aligns the current yaw rotation to the head tracker's yaw
	 * rotation.
	 */
	private var yawFix = Quaternion.IDENTITY

	/**
	 * Constraint fix is set by skeleton constraints. This corrects for any yaw rotation
	 * that violates the skeleton constraints. This is a local offset in rotation and
	 * does not affect the axes of rotation.
	 */
	private var constraintFix = Quaternion.IDENTITY

	// Zero-reference/identity adjustment quats for IMU debugging
	private var gyroFixNoMounting = Quaternion.IDENTITY
	private var attachmentFixNoMounting = Quaternion.IDENTITY
	private var yawFixZeroReference = Quaternion.IDENTITY

	/**
	 * T-Pose down fix is set by full reset. This corrects for the pitch of the rotation
	 * assuming a t-pose reference, adjusting to match our expected i-pose reference.
	 * This is a global offset in rotation and affects the axes of rotation.
	 */
	private var tposeDownFix = Quaternion.IDENTITY

	/**
	 * Reads/loads drift compensation settings from given config
	 */
	fun readDriftCompensationConfig(config: DriftCompensationConfig) {
		compensateDrift = config.enabled
		driftPrediction = config.prediction
		driftAmount = config.amount.coerceIn(0.0f, 1.0f)
		val maxResets = config.maxResets.coerceIn(1, MAX_DRIFT_SAMPLES)

		// Keep the ring buffers in sync with the configured sample count even when
		// compensation is disabled, so enabling it later uses the right capacity.
		if (maxResets != driftQuats.capacity()) {
			driftQuats = CircularArrayList<Quaternion>(maxResets)
			driftTimes = CircularArrayList<Long>(maxResets)
			clearDriftCompensation()
		}

		refreshDriftCompensationEnabled()
	}

	/**
	 * Clears drift compensation data.
	 *
	 * The averaged correction and the reset reference have to go too, otherwise the
	 * tracker keeps wearing the compensation from before the clear.
	 */
	fun clearDriftCompensation() {
		LiveDataRecorder.event(tracker, "clear_drift")
		driftSince = 0L
		timeAtLastReset = 0L
		totalDriftTime = 0L
		averagedDriftQuat = Quaternion.IDENTITY
		rotationSinceReset = Quaternion.IDENTITY
		driftQuats.clear()
		driftTimes.clear()
		refreshDriftCompensationEnabled()
	}

	/**
	 * Checks for compensateDrift, allowDriftCompensation, and if
	 * a computed head tracker exists.
	 */
	fun refreshDriftCompensationEnabled() {
		driftCompensationEnabled = compensateDrift &&
			allowDriftCompensation &&
			TrackerUtils.getNonInternalNonImuTrackerForBodyPosition(
				VRServer.instance.allTrackers,
				TrackerPosition.HEAD,
			) != null
	}

	/**
	 * Reads/loads reset settings from the given config
	 */
	fun readResetConfig(config: ResetsConfig) {
		armsResetMode = config.mode
		yawResetSmoothTime = config.yawResetSmoothTime
		saveMountingReset = config.saveMountingReset
		resetHmdPitch = config.resetHmdPitch
	}

	fun trySetMountingReset(quat: Quaternion) {
		if (saveMountingReset) {
			mountRotFix = quat
		}
	}

	/**
	 * Takes a rotation and adjusts it to resets, mounting,
	 * and drift compensation, with the HMD as the reference.
	 */
	fun getReferenceAdjustedDriftRotationFrom(rotation: Quaternion): Quaternion = adjustToDrift(adjustToReference(rotation))

	/**
	 * Takes a rotation and adjusts it to resets and mounting,
	 * with the identity Quaternion as the reference.
	 */
	fun getIdentityAdjustedDriftRotationFrom(rotation: Quaternion): Quaternion = adjustToDrift(adjustToIdentity(rotation))

	/**
	 * Get the reference adjusted accel.
	 */
	// TODO: Make this actually adjusted to the corrected IMU heading. The current
	//  implementation for heading correction doesn't appear to be correct and may simply
	//  make acceleration worse, so I'm just leaving this until we work that out. The
	//  output of this will be world space, but with an unknown offset to heading (yaw).
	//  - Butterscotch
	fun getReferenceAdjustedAccel(rawRot: Quaternion, accel: Vector3): Vector3 = rawRot.sandwich(accel)

	/**
	 * Converts raw or filtered rotation into reference- and
	 * mounting-reset-adjusted by applying quaternions produced after
	 * full reset, yaw rest and mounting reset
	 */
	private fun adjustToReference(rotation: Quaternion): Quaternion {
		var rot = rotation
		// Align heading axis with bone space
		if (!tracker.isHmd || tracker.trackerPosition != TrackerPosition.HEAD) {
			rot *= mountingOrientation
		}
		// Heading correction assuming manual orientation is correct
		rot = gyroFix * rot
		// Align attitude axes with bone space
		rot *= attachmentFix
		// Secondary heading axis alignment with bone space for automatic mounting
		// Note: Applying an inverse amount of heading correction corresponding to the
		//  axis alignment quaternion will leave the correction to another variable
		rot = mountRotFix.inv() * (rot * mountRotFix)
		// More attitude axes alignment specifically for the t-pose configuration, this
		//  probably shouldn't be a separate variable from attachmentFix?
		rot *= tposeDownFix
		// More heading correction
		rot = yawFix * rot
		rot = constraintFix * rot
		return rot
	}

	/**
	 * Converts raw or filtered rotation into zero-reference-adjusted by
	 * applying quaternions produced after full reset and yaw reset only
	 */
	// This is essentially just adjustToReference but aligning to quaternion identity
	//  rather than to the bone.
	private fun adjustToIdentity(rotation: Quaternion): Quaternion {
		var rot = rotation
		rot = gyroFixNoMounting * rot
		rot *= attachmentFixNoMounting
		rot = yawFixZeroReference * rot
		rot = constraintFix * rot
		return rot
	}

	/**
	 * Adjust the given rotation for drift compensation if enabled,
	 * and returns it
	 */
	private fun adjustToDrift(rotation: Quaternion): Quaternion {
		if (!driftCompensationEnabled || totalDriftTime <= 0) {
			return rotation
		}

		val elapsed = System.currentTimeMillis() - driftSince
		val ratio = driftTimeRatio(elapsed, totalDriftTime, driftPrediction)
		if (ratio <= 0.0f) {
			return rotation
		}

		// A sample larger than this is almost certainly a yaw wrap or a mounting
		// slip rather than drift; applying it would snap the tracker.
		val drift = if (averagedDriftQuat.angleToR(Quaternion.IDENTITY) > MAX_DRIFT_ANGLE_RAD) {
			Quaternion.IDENTITY
		} else {
			averagedDriftQuat
		}
		return drift.pow(driftAmount * ratio) * rotation
	}

	/**
	 * Reset the tracker so that its current rotation is counted as (0, HMD Yaw,
	 * 0). This allows the tracker to be strapped to body at any pitch and roll.
	 */
	fun resetFull(reference: Quaternion) {
		LiveDataRecorder.event(tracker, "reset_full")
		constraintFix = Quaternion.IDENTITY

		if (tracker.trackerDataType == TrackerDataType.FLEX_RESISTANCE) {
			tracker.trackerFlexHandler.resetMin()
			postProcessResetFull(reference)
			return
		} else if (tracker.trackerDataType == TrackerDataType.FLEX_ANGLE) {
			postProcessResetFull(reference)
			return
		}

		// Adjust for T-Pose (down)
		tposeDownFix = if (((tracker.trackerPosition.isLeftArm() || tracker.trackerPosition.isLeftFinger()) && armsResetMode == ArmsResetModes.TPOSE_DOWN)) {
			EulerAngles(EulerOrder.YZX, 0f, 0f, -FastMath.HALF_PI).toQuaternion()
		} else if (((tracker.trackerPosition.isRightArm() || tracker.trackerPosition.isRightFinger()) && armsResetMode == ArmsResetModes.TPOSE_DOWN)) {
			EulerAngles(EulerOrder.YZX, 0f, 0f, FastMath.HALF_PI).toQuaternion()
		} else {
			Quaternion.IDENTITY
		}

		// Old rot for drift compensation
		val oldRot = adjustToReference(tracker.getRawRotation())
		lastResetQuaternion = oldRot

		// Adjust raw rotation to mountingOrientation
		val mountingAdjustedRotation = tracker.getRawRotation() * mountingOrientation

		// Gyrofix
		if (tracker.allowMounting || (tracker.trackerPosition == TrackerPosition.HEAD && !tracker.isHmd)) {
			gyroFix = if (tracker.isComputed) {
				fixGyroscope(tracker.getRawRotation())
			} else {
				fixGyroscope(mountingAdjustedRotation * tposeDownFix)
			}
		}

		// Mounting for computed trackers
		if (tracker.isComputed && tracker.trackerPosition != TrackerPosition.HEAD) {
			// Set mounting to the reference's yaw so that a computed
			// tracker goes forward according to the head tracker.
			mountRotFix = getYawQuaternion(reference)
		}

		// Attachment fix
		attachmentFix = if (tracker.trackerPosition == TrackerPosition.HEAD && tracker.isHmd) {
			if (resetHmdPitch) {
				// Reset the HMD's pitch if it's assigned to head and resetHmdPitch is true
				// Get rotation without yaw (make sure to use the raw rotation directly!)
				val rotBuf = getYawQuaternion(tracker.getRawRotation()).inv() * tracker.getRawRotation()
				// Isolate pitch
				Quaternion(rotBuf.w, -rotBuf.x, 0f, 0f).unit()
			} else {
				// Don't reset the HMD at all
				Quaternion.IDENTITY
			}
		} else {
			fixAttachment(mountingAdjustedRotation)
		}

		// Rotate attachmentFix by 180 degrees as a workaround for t-pose (down)
		if (tposeDownFix != Quaternion.IDENTITY && tracker.allowMounting) {
			attachmentFix *= HalfHorizontal
		}

		makeIdentityAdjustmentQuatsFull()

		// Don't adjust yaw if head and computed
		if (tracker.trackerPosition != TrackerPosition.HEAD || !tracker.isComputed) {
			yawFix = fixYaw(mountingAdjustedRotation, reference)
			tracker.yawResetSmoothing.reset()
		}

		calculateDrift(oldRot)

		// Reset Stay Aligned (before resetting filtering, which depends on the
		// tracker's rotation)
		tracker.stayAligned.reset()

		postProcessResetFull(reference)
	}

	private fun postProcessResetFull(reference: Quaternion) {
		if (this.tracker.needReset) {
			this.tracker.needReset = false
		}

		tracker.resetFilteringQuats(reference)
	}

	/**
	 * Reset the tracker so that its current yaw rotation is aligned with the HMD's
	 * Yaw. This allows the tracker to have yaw independent of the HMD. Tracker
	 * should still report yaw as if it was mounted facing HMD, mounting
	 * position should be corrected in the source.
	 */
	fun resetYaw(reference: Quaternion) {
		LiveDataRecorder.event(tracker, "reset_yaw")
		// TODO HMD doesn't get yaw reset, which makes it so tracker.resetFilteringQuats() doesn't get called

		constraintFix = Quaternion.IDENTITY

		if (tracker.trackerDataType == TrackerDataType.FLEX_RESISTANCE ||
			tracker.trackerDataType == TrackerDataType.FLEX_ANGLE
		) {
			// Don't do anything as these don't have yaw anyways
			return
		}

		// Old rot for drift compensation
		val oldRot = adjustToReference(tracker.getRawRotation())
		lastResetQuaternion = oldRot

		val yawFixOld = yawFix
		yawFix = fixYaw(tracker.getRawRotation() * mountingOrientation, reference)
		tracker.yawResetSmoothing.reset()

		makeIdentityAdjustmentQuatsYaw()

		calculateDrift(oldRot)

		// Start at yaw before reset if smoothing enabled
		if (yawResetSmoothTime > 0.0f) {
			tracker.yawResetSmoothing.interpolate(
				yawFixOld / yawFix,
				Quaternion.IDENTITY,
				yawResetSmoothTime,
			)
		}

		// Reset Stay Aligned (before resetting filtering, which depends on the
		// tracker's rotation)
		tracker.stayAligned.reset()

		tracker.resetFilteringQuats(reference)
	}

	/**
	 * Perform the math to align the tracker to go forward
	 * and stores it in mountRotFix, and adjusts yawFix
	 */
	fun resetMounting(reference: Quaternion) {
		LiveDataRecorder.event(tracker, "reset_mounting")
		if (tracker.trackerDataType == TrackerDataType.FLEX_RESISTANCE) {
			tracker.trackerFlexHandler.resetMax()
			tracker.resetFilteringQuats(reference)
			return
		} else if (tracker.trackerDataType == TrackerDataType.FLEX_ANGLE) {
			return
		}

		constraintFix = Quaternion.IDENTITY

		// Get the current calibrated rotation
		var rotBuf = adjustToDrift(tracker.getRawRotation() * mountingOrientation)
		rotBuf = gyroFix * rotBuf
		rotBuf *= attachmentFix
		rotBuf = yawFix * rotBuf

		// Adjust buffer to reference
		rotBuf = reference.project(Vector3.POS_Y).inv().unit() * rotBuf

		// Rotate a vector pointing up by the quat
		val rotVector = rotBuf.sandwich(Vector3.POS_Y)

		// Calculate the yaw angle using tan
		var yawAngle = atan2(rotVector.x, rotVector.z)

		// Adjust for T-Pose and fingers
		if ((tracker.trackerPosition.isLeftArm() && armsResetMode == ArmsResetModes.TPOSE_DOWN) ||
			(tracker.trackerPosition.isRightArm() && armsResetMode == ArmsResetModes.TPOSE_UP) ||
			tracker.trackerPosition.isLeftFinger()
		) {
			// Tracker goes right
			yawAngle -= FastMath.HALF_PI
		}
		if ((tracker.trackerPosition.isLeftArm() && armsResetMode == ArmsResetModes.TPOSE_UP) ||
			(tracker.trackerPosition.isRightArm() && armsResetMode == ArmsResetModes.TPOSE_DOWN) ||
			tracker.trackerPosition.isRightFinger()
		) {
			// Tracker goes left
			yawAngle += FastMath.HALF_PI
		}

		// Adjust for forward/back arms and thighs
		val isLowerArmBack = armsResetMode == ArmsResetModes.BACK && (tracker.trackerPosition.isLeftLowerArm() || tracker.trackerPosition.isRightLowerArm())
		val isArmForward = armsResetMode == ArmsResetModes.FORWARD && (tracker.trackerPosition.isLeftArm() || tracker.trackerPosition.isRightArm())
		if (!tracker.trackerPosition.isThigh() && !isArmForward && !isLowerArmBack) {
			// Tracker goes back
			yawAngle -= FastMath.PI
		}

		// Make an adjustment quaternion from the angle
		mountRotFix = EulerAngles(EulerOrder.YZX, 0f, yawAngle, 0f).toQuaternion()

		// save mounting reset
		if (saveMountingReset) tracker.saveMountingResetOrientation(mountRotFix)

		tracker.resetFilteringQuats(reference)
	}

	/**
	 * Apply a corrective rotation to the gyroFix
	 */
	fun updateConstraintFix(correctedRotation: Quaternion) {
		constraintFix *= correctedRotation
	}

	fun clearMounting() {
		LiveDataRecorder.event(tracker, "clear_mounting")
		mountRotFix = Quaternion.IDENTITY
	}

	private fun fixGyroscope(sensorRotation: Quaternion): Quaternion = getYawQuaternion(sensorRotation).inv()

	private fun fixAttachment(sensorRotation: Quaternion): Quaternion = (gyroFix * sensorRotation).inv()

	private fun fixYaw(sensorRotation: Quaternion, reference: Quaternion): Quaternion {
		var rot = gyroFix * sensorRotation
		rot *= attachmentFix
		rot = mountRotFix.inv() * (rot * mountRotFix)
		rot = getYawQuaternion(rot)
		return rot.inv() * reference.project(Vector3.POS_Y).unit()
	}

	// TODO : isolating yaw for yaw reset bad.
	// The way we isolate the tracker's yaw for yaw reset is
	// incorrect. Projection around the Y-axis is worse.
	// In both cases, the isolated yaw value changes
	// with the tracker's roll when pointing forward.
	// calling twinNearest() makes sure this rotation has the wanted polarity (+-).
	private fun getYawQuaternion(rot: Quaternion): Quaternion = EulerAngles(EulerOrder.YZX, 0f, rot.toEulerAngles(EulerOrder.YZX).y, 0f).toQuaternion().twinNearest(rot)

	private fun makeIdentityAdjustmentQuatsFull() {
		val sensorRotation = tracker.getRawRotation()
		gyroFixNoMounting = fixGyroscope(sensorRotation)
		attachmentFixNoMounting = (gyroFixNoMounting * sensorRotation).inv()
		yawFixZeroReference = Quaternion.IDENTITY
	}

	private fun makeIdentityAdjustmentQuatsYaw() {
		var sensorRotation = tracker.getRawRotation()
		sensorRotation = gyroFixNoMounting * sensorRotation
		sensorRotation *= attachmentFixNoMounting
		yawFixZeroReference = fixGyroscope(sensorRotation)
	}

	/**
	 * Calculates drift since last reset and store the data related to it in
	 * driftQuat and timeAtLastReset
	 */
	private fun calculateDrift(beforeQuat: Quaternion) {
		if (driftCompensationEnabled) {
			val rotQuat = adjustToReference(tracker.getRawRotation())

			if (driftSince > 0 && System.currentTimeMillis() - timeAtLastReset > DRIFT_COOLDOWN_MS) {
				// Check and remove from lists to keep them under the reset limit
				if (driftQuats.size == driftQuats.capacity()) {
					driftQuats.removeLast()
					driftTimes.removeLast()
				}

				// Add new drift quaternion
				driftQuats.add(getYawQuaternion(rotQuat) / getYawQuaternion(beforeQuat))

				// Add drift time to total
				driftTimes.add(System.currentTimeMillis() - driftSince)
				totalDriftTime = 0
				for (time in driftTimes) {
					totalDriftTime += time
				}

				// Calculate drift Quaternions' weights
				val driftWeights = ArrayList<Float>(driftTimes.size)
				for (time in driftTimes) {
					driftWeights.add(time.toFloat() / totalDriftTime.toFloat())
				}

				// Make it so recent Quaternions weigh more
				for (i in driftWeights.size - 1 downTo 1) {
					// Add some of i-1's value to i
					driftWeights[i] = driftWeights[i] + driftWeights[i - 1] / driftWeights.size
					// Remove the value that was added to i from i-1
					driftWeights[i - 1] = driftWeights[i - 1] - driftWeights[i - 1] / driftWeights.size
				}

				// Set final averaged drift Quaternion
				averagedDriftQuat = fromAveragedQuaternions(driftQuats, driftWeights)

				// Save tracker rotation and current time
				rotationSinceReset = driftQuats.latest
				timeAtLastReset = System.currentTimeMillis()
			} else if (System.currentTimeMillis() - timeAtLastReset < DRIFT_COOLDOWN_MS && driftQuats.size > 0) {
				// Replace latest drift quaternion
				rotationSinceReset *= (getYawQuaternion(rotQuat) / getYawQuaternion(beforeQuat))
				driftQuats[driftQuats.size - 1] = rotationSinceReset

				// Add drift time to total
				driftTimes[driftTimes.size - 1] = driftTimes.latest + System.currentTimeMillis() - driftSince
				totalDriftTime = 0
				for (time in driftTimes) {
					totalDriftTime += time
				}

				// Calculate drift Quaternions' weights
				val driftWeights = ArrayList<Float>(driftTimes.size)
				for (time in driftTimes) {
					driftWeights.add(time.toFloat() / totalDriftTime.toFloat())
				}

				// Make it so recent Quaternions weigh more
				for (i in driftWeights.size - 1 downTo 1) {
					// Add some of i-1's value to i
					driftWeights[i] = driftWeights[i] + driftWeights[i - 1] / driftWeights.size
					// Remove the value that was added to i from i-1
					driftWeights[i - 1] = driftWeights[i - 1] - driftWeights[i - 1] / driftWeights.size
				}

				// Set final averaged drift Quaternion
				averagedDriftQuat = fromAveragedQuaternions(driftQuats, driftWeights)
			} else {
				timeAtLastReset = System.currentTimeMillis()
			}

			driftSince = System.currentTimeMillis()
		}
	}

	/**
	 * Calculates and returns the averaged Quaternion
	 * from the given Quaternions and weights.
	 */
	private fun fromAveragedQuaternions(
		qn: CircularArrayList<Quaternion>,
		tn: ArrayList<Float>,
	): Quaternion {
		var totalMatrix = qn[0].toMatrix() * tn[0]
		for (i in 1 until qn.size) {
			totalMatrix += (qn[i].toMatrix() * tn[i])
		}
		return totalMatrix.toQuaternion()
	}

	companion object {
		/** Upper bound on the number of stored drift samples. */
		const val MAX_DRIFT_SAMPLES = 32

		/**
		 * Prediction may run at most one extra averaging window past the last
		 * sample. Unbounded prediction kept rotating the tracker forever when no
		 * fresh sample arrived, which turned estimator noise into visible drift.
		 */
		const val MAX_DRIFT_PREDICTION_RATIO = 2.0f

		/** A larger "drift" is a yaw wrap or mounting slip, not drift. */
		const val MAX_DRIFT_ANGLE_RAD = 1.5f

		/**
		 * How much of the averaged drift to apply, as a multiple of the elapsed
		 * averaging window. Without prediction the correction saturates after one
		 * window; with prediction it may run one window ahead, then saturates.
		 */
		fun driftTimeRatio(elapsedMs: Long, totalMs: Long, prediction: Boolean): Float {
			if (elapsedMs <= 0 || totalMs <= 0) {
				return 0.0f
			}
			val ratio = elapsedMs.toFloat() / totalMs.toFloat()
			val maxRatio = if (prediction) MAX_DRIFT_PREDICTION_RATIO else 1.0f
			return min(maxRatio, ratio)
		}
	}
}
