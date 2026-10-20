package dev.slimevr.math

import io.github.axisangles.ktmath.Quaternion

/**
 * Helpers for rotations received from external sources.
 *
 * Quaternion math in the tracker pipeline assumes unit quaternions. Senders are
 * normally well behaved, but a single non-finite or unnormalised sample would
 * poison filtering and skeleton output, so inputs are sanitised at the edge.
 */
object Quaternions {
	/** Below this squared length a quaternion carries no usable rotation. */
	private const val MIN_NORM_SQUARED = 1e-12f

	/**
	 * Normalises [rotation], returning the identity for zero, near-zero or
	 * non-finite input.
	 */
	fun sanitize(rotation: Quaternion): Quaternion {
		val normSquared = rotation.w *
			rotation.w +
			rotation.x *
			rotation.x +
			rotation.y *
			rotation.y +
			rotation.z *
			rotation.z
		return if (!normSquared.isFinite() || normSquared < MIN_NORM_SQUARED) {
			Quaternion.IDENTITY
		} else {
			rotation.unit()
		}
	}
}
