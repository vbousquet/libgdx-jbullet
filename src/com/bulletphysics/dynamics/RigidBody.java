/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose, 
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package com.bulletphysics.dynamics;

import com.badlogic.gdx.math.Matrix3;
import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import com.bulletphysics.BulletGlobals;
import com.bulletphysics.collision.broadphase.BroadphaseProxy;
import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.CollisionObjectType;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraint;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.MiscUtil;
import com.bulletphysics.linearmath.MotionState;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.TransformUtil;
import com.bulletphysics.util.ObjectArrayList;
import com.bulletphysics.util.Stack;
import com.bulletphysics.util.StaticAlloc;

/** RigidBody is the main class for rigid body objects. It is derived from {@link CollisionObject}, so it keeps reference to
 * {@link CollisionShape}.
 * <p>
 * 
 * It is recommended for performance and memory use to share {@link CollisionShape} objects whenever possible.
 * <p>
 * 
 * There are 3 types of rigid bodies:<br>
 * <ol>
 * <li>Dynamic rigid bodies, with positive mass. Motion is controlled by rigid body dynamics.</li>
 * <li>Fixed objects with zero mass. They are not moving (basically collision objects).</li>
 * <li>Kinematic objects, which are objects without mass, but the user can move them. There is on-way interaction, and Bullet
 * calculates a velocity based on the timestep and previous and current world transform.</li>
 * </ol>
 * 
 * Bullet automatically deactivates dynamic rigid bodies, when the velocity is below a threshold for a given time.
 * <p>
 * 
 * Deactivated (sleeping) rigid bodies don't take any processing time, except a minor broadphase collision detection impact (to
 * allow active objects to activate/wake up sleeping objects).
 * 
 * @author jezek2 */
public class RigidBody extends CollisionObject {

	private static final float MAX_ANGVEL = BulletGlobals.SIMD_HALF_PI;

	private final Matrix3 invInertiaTensorWorld = new Matrix3();
	private final Vector3 linearVelocity = new Vector3();
	private final Vector3 angularVelocity = new Vector3();
	private float inverseMass;
	private final Vector3 angularFactor = new Vector3();
	private final Vector3 linearFactor = new Vector3();

	private final Vector3 gravity = new Vector3();
	private final Vector3 invInertiaLocal = new Vector3();
	private final Vector3 totalForce = new Vector3();
	private final Vector3 totalTorque = new Vector3();

	private float linearDamping;
	private float angularDamping;

	private boolean additionalDamping;
	private float additionalDampingFactor;
	private float additionalLinearDampingThresholdSqr;
	private float additionalAngularDampingThresholdSqr;
	private float additionalAngularDampingFactor;

	private float linearSleepingThreshold;
	private float angularSleepingThreshold;

	// optionalMotionState allows to automatic synchronize the world transform for active objects
	private MotionState optionalMotionState;

	// keep track of typed constraints referencing this rigid body
	private final ObjectArrayList<TypedConstraint> constraintRefs = new ObjectArrayList<TypedConstraint>();

	// for experimental overriding of friction/contact solver func
	public int contactSolverType;
	public int frictionSolverType;

	private static int uniqueId = 0;
	public int debugBodyId;

	public RigidBody (RigidBodyConstructionInfo constructionInfo) {
		setupRigidBody(constructionInfo);
	}

	public RigidBody (float mass, MotionState motionState, CollisionShape collisionShape) {
		this(mass, motionState, collisionShape, new Vector3(0f, 0f, 0f));
	}

	public RigidBody (float mass, MotionState motionState, CollisionShape collisionShape, Vector3 localInertia) {
		RigidBodyConstructionInfo cinfo = new RigidBodyConstructionInfo(mass, motionState, collisionShape, localInertia);
		setupRigidBody(cinfo);
	}

	private void setupRigidBody (RigidBodyConstructionInfo constructionInfo) {
		internalType = CollisionObjectType.RIGID_BODY;

		linearVelocity.set(0f, 0f, 0f);
		angularVelocity.set(0f, 0f, 0f);
		linearFactor.set(1f, 1f, 1f);
		angularFactor.set(1f, 1f, 1f);
		gravity.set(0f, 0f, 0f);
		totalForce.set(0f, 0f, 0f);
		totalTorque.set(0f, 0f, 0f);
		linearDamping = 0f;
		angularDamping = 0.5f;
		linearSleepingThreshold = constructionInfo.linearSleepingThreshold;
		angularSleepingThreshold = constructionInfo.angularSleepingThreshold;
		optionalMotionState = constructionInfo.motionState;
		contactSolverType = 0;
		frictionSolverType = 0;
		additionalDamping = constructionInfo.additionalDamping;
		additionalDampingFactor = constructionInfo.additionalDampingFactor;
		additionalLinearDampingThresholdSqr = constructionInfo.additionalLinearDampingThresholdSqr;
		additionalAngularDampingThresholdSqr = constructionInfo.additionalAngularDampingThresholdSqr;
		additionalAngularDampingFactor = constructionInfo.additionalAngularDampingFactor;

		if (optionalMotionState != null) {
			optionalMotionState.getWorldTransform(worldTransform);
		} else {
			worldTransform.set(constructionInfo.startWorldTransform);
		}

		interpolationWorldTransform.set(worldTransform);
		interpolationLinearVelocity.set(0f, 0f, 0f);
		interpolationAngularVelocity.set(0f, 0f, 0f);

		// moved to CollisionObject
		friction = constructionInfo.friction;
		restitution = constructionInfo.restitution;

		setCollisionShape(constructionInfo.collisionShape);
		debugBodyId = uniqueId++;

		setMassProps(constructionInfo.mass, constructionInfo.localInertia);
		setDamping(constructionInfo.linearDamping, constructionInfo.angularDamping);
		updateInertiaTensor();
	}

	public void destroy () {
		// No constraints should point to this rigidbody
		// Remove constraints from the dynamics world before you delete the related rigidbodies.
		assert (constraintRefs.size() == 0);
	}

	public void proceedToTransform (Transform newTrans) {
		setCenterOfMassTransform(newTrans);
	}

	/** To keep collision detection and dynamics separate we don't store a rigidbody pointer, but a rigidbody is derived from
	 * CollisionObject, so we can safely perform an upcast. */
	public static RigidBody upcast (CollisionObject colObj) {
		if (colObj.getInternalType() == CollisionObjectType.RIGID_BODY) {
			return (RigidBody)colObj;
		}
		return null;
	}

	/** Continuous collision detection needs prediction. */
	public void predictIntegratedTransform (float timeStep, Transform predictedTransform) {
		TransformUtil.integrateTransform(worldTransform, linearVelocity, angularVelocity, timeStep, predictedTransform);
	}

	public void saveKinematicState (float timeStep) {
		// todo: clamp to some (user definable) safe minimum timestep, to limit maximum angular/linear velocities
		if (timeStep != 0f) {
			// if we use motionstate to synchronize world transforms, get the new kinematic/animated world transform
			if (getMotionState() != null) {
				getMotionState().getWorldTransform(worldTransform);
			}
			// Vector3 linVel = new Vector3(), angVel = new Vector3();

			TransformUtil.calculateVelocity(interpolationWorldTransform, worldTransform, timeStep, linearVelocity, angularVelocity);
			interpolationLinearVelocity.set(linearVelocity);
			interpolationAngularVelocity.set(angularVelocity);
			interpolationWorldTransform.set(worldTransform);
			// printf("angular = %f %f %f\n",m_angularVelocity.getX(),m_angularVelocity.getY(),m_angularVelocity.getZ());
		}
	}

	public void applyGravity () {
		if (isStaticOrKinematicObject()) return;

		applyCentralForce(gravity);
	}

	public void setGravity (Vector3 acceleration) {
		if (inverseMass != 0f) {
			gravity.set(acceleration).scl(1f / inverseMass);
		}
	}

	public Vector3 getGravity (Vector3 out) {
		out.set(gravity);
		return out;
	}

	public void setDamping (float lin_damping, float ang_damping) {
		linearDamping = MiscUtil.GEN_clamped(lin_damping, 0f, 1f);
		angularDamping = MiscUtil.GEN_clamped(ang_damping, 0f, 1f);
	}

	public float getLinearDamping () {
		return linearDamping;
	}

	public float getAngularDamping () {
		return angularDamping;
	}

	public float getLinearSleepingThreshold () {
		return linearSleepingThreshold;
	}

	public float getAngularSleepingThreshold () {
		return angularSleepingThreshold;
	}

	/** Damps the velocity, using the given linearDamping and angularDamping. */
	public void applyDamping (float timeStep) {
		// On new damping: see discussion/issue report here: http://code.google.com/p/bullet/issues/detail?id=74
		// todo: do some performance comparisons (but other parts of the engine are probably bottleneck anyway

		// #define USE_OLD_DAMPING_METHOD 1
		// #ifdef USE_OLD_DAMPING_METHOD
		// linearVelocity.scl(MiscUtil.GEN_clamped((1f - timeStep * linearDamping), 0f, 1f));
		// angularVelocity.scl(MiscUtil.GEN_clamped((1f - timeStep * angularDamping), 0f, 1f));
		// #else
		linearVelocity.scl((float)Math.pow(1f - linearDamping, timeStep));
		angularVelocity.scl((float)Math.pow(1f - angularDamping, timeStep));
		// #endif

		if (additionalDamping) {
			// Additional damping can help avoiding lowpass jitter motion, help stability for ragdolls etc.
			// Such damping is undesirable, so once the overall simulation quality of the rigid body dynamics system has improved,
// this should become obsolete
			if ((angularVelocity.len2() < additionalAngularDampingThresholdSqr)
				&& (linearVelocity.len2() < additionalLinearDampingThresholdSqr)) {
				angularVelocity.scl(additionalDampingFactor);
				linearVelocity.scl(additionalDampingFactor);
			}

			float speed = linearVelocity.len();
			if (speed < linearDamping) {
				float dampVel = 0.005f;
				if (speed > dampVel) {
					Stack stack = Stack.enter();
					Vector3 dir = stack.alloc(linearVelocity);
					dir.nor();
					dir.scl(dampVel);
					linearVelocity.sub(dir);
					stack.leave();
				} else {
					linearVelocity.set(0f, 0f, 0f);
				}
			}

			float angSpeed = angularVelocity.len();
			if (angSpeed < angularDamping) {
				float angDampVel = 0.005f;
				if (angSpeed > angDampVel) {
					Stack stack = Stack.enter();
					Vector3 dir = stack.alloc(angularVelocity);
					dir.nor();
					dir.scl(angDampVel);
					angularVelocity.sub(dir);
					stack.leave();
				} else {
					angularVelocity.set(0f, 0f, 0f);
				}
			}
		}
	}

	public void setMassProps (float mass, Vector3 inertia) {
		if (mass == 0f) {
			collisionFlags |= CollisionFlags.STATIC_OBJECT;
			inverseMass = 0f;
		} else {
			collisionFlags &= (~CollisionFlags.STATIC_OBJECT);
			inverseMass = 1f / mass;
		}

		invInertiaLocal.set(inertia.x != 0f ? 1f / inertia.x : 0f, inertia.y != 0f ? 1f / inertia.y : 0f,
			inertia.z != 0f ? 1f / inertia.z : 0f);
	}

	public float getInvMass () {
		return inverseMass;
	}

	public Matrix3 getInvInertiaTensorWorld (Matrix3 out) {
		out.set(invInertiaTensorWorld);
		return out;
	}

	public void integrateVelocities (float step) {
		if (isStaticOrKinematicObject()) {
			return;
		}
		Stack stack = Stack.enter();
		linearVelocity.x += inverseMass * step * totalForce.x;
		linearVelocity.y += inverseMass * step * totalForce.y;
		linearVelocity.z += inverseMass * step * totalForce.z;
		Vector3 tmp = stack.alloc(totalTorque);
		tmp.mul(invInertiaTensorWorld);
		tmp.scl(step);
		angularVelocity.add(tmp);

		// clamp angular velocity. collision calculations will fail on higher angular velocities
		float angvel = angularVelocity.len();
		if (angvel * step > MAX_ANGVEL) {
			angularVelocity.scl((MAX_ANGVEL / step) / angvel);
		}
		stack.leave();
	}

	public void setCenterOfMassTransform (Transform xform) {
		if (isStaticOrKinematicObject()) {
			interpolationWorldTransform.set(worldTransform);
		} else {
			interpolationWorldTransform.set(xform);
		}
		getLinearVelocity(interpolationLinearVelocity);
		getAngularVelocity(interpolationAngularVelocity);
		worldTransform.set(xform);
		updateInertiaTensor();
	}

	public void applyCentralForce (Vector3 force) {
		totalForce.x += force.x * linearFactor.x;
		totalForce.y += force.y * linearFactor.y;
		totalForce.z += force.z * linearFactor.z;
	}

	public Vector3 getInvInertiaDiagLocal (Vector3 out) {
		out.set(invInertiaLocal);
		return out;
	}

	public void setInvInertiaDiagLocal (Vector3 diagInvInertia) {
		invInertiaLocal.set(diagInvInertia);
	}

	public void setSleepingThresholds (float linear, float angular) {
		linearSleepingThreshold = linear;
		angularSleepingThreshold = angular;
	}

	public void applyTorque (Vector3 torque) {
		totalTorque.add(torque);
	}

	public void applyForce (Vector3 force, Vector3 rel_pos) {
		Stack stack = Stack.enter();
		applyCentralForce(force);

		Vector3 tmp = stack.allocVector3();
		Vector3 scaledForce = stack.allocVector3();
		scaledForce.set(force.x * linearFactor.x, force.y * linearFactor.y, force.z * linearFactor.z);
		tmp.set(rel_pos).crs( scaledForce);
		tmp.x *= angularFactor.x;
		tmp.y *= angularFactor.y;
		tmp.z *= angularFactor.z;
		applyTorque(tmp);
		stack.leave();
	}

	public void applyCentralImpulse (Vector3 impulse) {
		Stack stack = Stack.enter();
		Vector3 scaledImpulse = stack.allocVector3();
		scaledImpulse.set(impulse.x * linearFactor.x, impulse.y * linearFactor.y, impulse.z * linearFactor.z);
		scaledImpulse.scl(inverseMass);
		linearVelocity.add(scaledImpulse);
		stack.leave();
	}

	@StaticAlloc
	public void applyTorqueImpulse (Vector3 torque) {
		Stack stack = Stack.enter();
		Vector3 tmp = stack.alloc(torque);
		tmp.mul(invInertiaTensorWorld);
		angularVelocity.add(tmp);
		stack.leave();
	}

	@StaticAlloc
	public void applyImpulse (Vector3 impulse, Vector3 rel_pos) {
		if (inverseMass != 0f) {
			applyCentralImpulse(impulse);
			Stack stack = Stack.enter();
			Vector3 tmp = stack.allocVector3();
			Vector3 scaledImpulse = stack.allocVector3();
			scaledImpulse.set(impulse.x * linearFactor.x, impulse.y * linearFactor.y, impulse.z * linearFactor.z);
			tmp.set(rel_pos).crs( scaledImpulse);
			tmp.x *= angularFactor.x;
			tmp.y *= angularFactor.y;
			tmp.z *= angularFactor.z;
			applyTorqueImpulse(tmp);
			stack.leave();
		}
	}

	/** Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position. */
	public void internalApplyImpulse (Vector3 linearComponent, Vector3 angularComponent, float impulseMagnitude) {
		if (inverseMass != 0f) {
			// linearVelocity.scaleAdd(impulseMagnitude, linearComponent, linearVelocity);
			linearVelocity.x += impulseMagnitude * angularComponent.x;
			linearVelocity.y += impulseMagnitude * angularComponent.y;
			linearVelocity.z += impulseMagnitude * angularComponent.z;
			angularVelocity.x += impulseMagnitude * angularFactor.x * angularComponent.x;
			angularVelocity.y += impulseMagnitude * angularFactor.y * angularComponent.y;
			angularVelocity.z += impulseMagnitude * angularFactor.z * angularComponent.z;
			// angularVelocity.scaleAdd(impulseMagnitude * angularFactor, angularComponent, angularVelocity);
		}
	}

	public void clearForces () {
		totalForce.set(0f, 0f, 0f);
		totalTorque.set(0f, 0f, 0f);
	}

	public void updateInertiaTensor () {
		Stack stack = Stack.enter();
		Matrix3 mat1 = stack.allocMatrix3();
		MatrixUtil.scale(mat1, worldTransform.basis, invInertiaLocal);

		Matrix3 mat2 = stack.alloc(worldTransform.basis);
		mat2.transpose();

		invInertiaTensorWorld.set(mat1).mul(mat2);
		stack.leave();
	}

	public Vector3 getCenterOfMassPosition (Vector3 out) {
		out.set(worldTransform.origin);
		return out;
	}

	public Quaternion getOrientation (Quaternion out) {
		MatrixUtil.getRotation(worldTransform.basis, out);
		return out;
	}

	public Transform getCenterOfMassTransform (Transform out) {
		out.set(worldTransform);
		return out;
	}

	public Vector3 getLinearVelocity (Vector3 out) {
		out.set(linearVelocity);
		return out;
	}

	public Vector3 getAngularVelocity (Vector3 out) {
		out.set(angularVelocity);
		return out;
	}

	public void setLinearVelocity (Vector3 lin_vel) {
		assert (collisionFlags != CollisionFlags.STATIC_OBJECT);
		linearVelocity.set(lin_vel);
	}

	public void setAngularVelocity (Vector3 ang_vel) {
		assert (collisionFlags != CollisionFlags.STATIC_OBJECT);
		angularVelocity.set(ang_vel);
	}

	public Vector3 getVelocityInLocalPoint (Vector3 rel_pos, Vector3 out) {
		// we also calculate lin/ang velocity for kinematic objects
		Vector3 vec = out;
		vec.set(angularVelocity).crs( rel_pos);
		vec.add(linearVelocity);
		return out;

		// for kinematic objects, we could also use use:
		// return (m_worldTransform(rel_pos) - m_interpolationWorldTransform(rel_pos)) / m_kinematicTimeStep;
	}

	public void translate (Vector3 v) {
		worldTransform.origin.add(v);
	}

	public void getAabb (Vector3 aabbMin, Vector3 aabbMax) {
		getCollisionShape().getAabb(worldTransform, aabbMin, aabbMax);
	}

	public float computeImpulseDenominator (Vector3 pos, Vector3 normal) {
		Stack stack = Stack.enter();
		Vector3 r0 = stack.allocVector3();
		r0.set(pos).sub(getCenterOfMassPosition(stack.allocVector3()));

		Vector3 c0 = stack.allocVector3();
		c0.set(r0).crs( normal);

		Vector3 tmp = stack.allocVector3();
		MatrixUtil.transposeTransform(tmp, c0, getInvInertiaTensorWorld(stack.allocMatrix3()));

		Vector3 vec = stack.allocVector3();
		vec.set(tmp).crs( r0);

		float result = inverseMass + normal.dot(vec);
		stack.leave();
		return result;
	}

	public float computeAngularImpulseDenominator (Vector3 axis) {
		Stack stack = Stack.enter();
		Vector3 vec = stack.allocVector3();
		MatrixUtil.transposeTransform(vec, axis, getInvInertiaTensorWorld(stack.allocMatrix3()));
		float result = axis.dot(vec);
		stack.leave();
		return result;
	}

	public void updateDeactivation (float timeStep) {
		if ((getActivationState() == ISLAND_SLEEPING) || (getActivationState() == DISABLE_DEACTIVATION)) {
			return;
		}
		Stack stack = Stack.enter();
		if ((getLinearVelocity(stack.allocVector3()).len2() < linearSleepingThreshold * linearSleepingThreshold)
			&& (getAngularVelocity(stack.allocVector3()).len2() < angularSleepingThreshold * angularSleepingThreshold)) {
			deactivationTime += timeStep;
		} else {
			deactivationTime = 0f;
			setActivationState(0);
		}
		stack.leave();
	}

	public boolean wantsSleeping () {
		if (getActivationState() == DISABLE_DEACTIVATION) {
			return false;
		}

		// disable deactivation
		if (BulletGlobals.isDeactivationDisabled() || (BulletGlobals.getDeactivationTime() == 0f)) {
			return false;
		}

		if ((getActivationState() == ISLAND_SLEEPING) || (getActivationState() == WANTS_DEACTIVATION)) {
			return true;
		}

		if (deactivationTime > BulletGlobals.getDeactivationTime()) {
			return true;
		}
		return false;
	}

	public BroadphaseProxy getBroadphaseProxy () {
		return broadphaseHandle;
	}

	public void setNewBroadphaseProxy (BroadphaseProxy broadphaseProxy) {
		this.broadphaseHandle = broadphaseProxy;
	}

	public MotionState getMotionState () {
		return optionalMotionState;
	}

	public void setMotionState (MotionState motionState) {
		this.optionalMotionState = motionState;
		if (optionalMotionState != null) {
			motionState.getWorldTransform(worldTransform);
		}
	}

	public void setAngularFactor (float angFac) {
		angularFactor.set(angFac, angFac, angFac);
	}

	public void setAngularFactor (Vector3 angFac) {
		angularFactor.set(angFac);
	}

	public void setLinearFactor (Vector3 linearFactor) {
		this.linearFactor.set(linearFactor);
		// FIXME update inverse mass
	}

	public Vector3 getAngularFactor () {
		return angularFactor;
	}

	public Vector3 getLinearFactor () {
		return linearFactor;
	}

	/** Is this rigidbody added to a CollisionWorld/DynamicsWorld/Broadphase? */
	public boolean isInWorld () {
		return (getBroadphaseProxy() != null);
	}

	@Override
	public boolean checkCollideWithOverride (CollisionObject co) {
		// TODO: change to cast
		RigidBody otherRb = RigidBody.upcast(co);
		if (otherRb == null) {
			return true;
		}

		for (int i = 0; i < constraintRefs.size(); ++i) {
			TypedConstraint c = constraintRefs.getQuick(i);
			if (c.getRigidBodyA() == otherRb || c.getRigidBodyB() == otherRb) {
				return false;
			}
		}

		return true;
	}

	public void addConstraintRef (TypedConstraint c) {
		int index = constraintRefs.indexOf(c);
		if (index == -1) {
			constraintRefs.add(c);
		}

		checkCollideWith = true;
	}

	public void removeConstraintRef (TypedConstraint c) {
		constraintRefs.remove(c);
		checkCollideWith = (constraintRefs.size() > 0);
	}

	public TypedConstraint getConstraintRef (int index) {
		return constraintRefs.getQuick(index);
	}

	public int getNumConstraintRefs () {
		return constraintRefs.size();
	}

}
