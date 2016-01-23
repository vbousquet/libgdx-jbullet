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

package com.bulletphysics.dynamics.vehicle;

import com.badlogic.gdx.math.Matrix3;
import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.FloatArray;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.constraintsolver.ContactConstraint;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraint;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraintType;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.MiscUtil;
import com.bulletphysics.linearmath.QuaternionUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ArrayPool;
import com.bulletphysics.util.ObjectArrayList;
import com.bulletphysics.util.Stack;
import com.bulletphysics.util.Suppliers;

/** Raycast vehicle, very special constraint that turn a rigidbody into a vehicle.
 * 
 * @author jezek2 */
public class RaycastVehicle extends TypedConstraint {

	private final ArrayPool<float[]> floatArrays = ArrayPool.get(float.class);

	private static RigidBody s_fixedObject = new RigidBody(0, null, null);
	private static final float sideFrictionStiffness2 = 1.0f;

	protected ObjectArrayList<Vector3> forwardWS = new ObjectArrayList<Vector3>();
	protected ObjectArrayList<Vector3> axle = new ObjectArrayList<Vector3>();
	protected FloatArray forwardImpulse = new FloatArray();
	protected FloatArray sideImpulse = new FloatArray();

	private float tau;
	private float damping;
	private VehicleRaycaster vehicleRaycaster;
	private float pitchControl = 0f;
	private float steeringValue;
	private float currentVehicleSpeedKmHour;

	private RigidBody chassisBody;

	private int indexRightAxis = 0;
	private int indexUpAxis = 2;
	private int indexForwardAxis = 1;

	public ObjectArrayList<WheelInfo> wheelInfo = new ObjectArrayList<WheelInfo>();

	// constructor to create a car from an existing rigidbody
	public RaycastVehicle (VehicleTuning tuning, RigidBody chassis, VehicleRaycaster raycaster) {
		super(TypedConstraintType.VEHICLE_CONSTRAINT_TYPE);
		this.vehicleRaycaster = raycaster;
		this.chassisBody = chassis;
		defaultInit(tuning);
	}

	private void defaultInit (VehicleTuning tuning) {
		currentVehicleSpeedKmHour = 0f;
		steeringValue = 0f;
	}

	/** Basically most of the code is general for 2 or 4 wheel vehicles, but some of it needs to be reviewed. */
	public WheelInfo addWheel (Vector3 connectionPointCS, Vector3 wheelDirectionCS0, Vector3 wheelAxleCS,
		float suspensionRestLength, float wheelRadius, VehicleTuning tuning, boolean isFrontWheel) {
		WheelInfoConstructionInfo ci = new WheelInfoConstructionInfo();

		ci.chassisConnectionCS.set(connectionPointCS);
		ci.wheelDirectionCS.set(wheelDirectionCS0);
		ci.wheelAxleCS.set(wheelAxleCS);
		ci.suspensionRestLength = suspensionRestLength;
		ci.wheelRadius = wheelRadius;
		ci.suspensionStiffness = tuning.suspensionStiffness;
		ci.wheelsDampingCompression = tuning.suspensionCompression;
		ci.wheelsDampingRelaxation = tuning.suspensionDamping;
		ci.frictionSlip = tuning.frictionSlip;
		ci.bIsFrontWheel = isFrontWheel;
		ci.maxSuspensionTravelCm = tuning.maxSuspensionTravelCm;

		wheelInfo.add(new WheelInfo(ci));

		WheelInfo wheel = wheelInfo.getQuick(getNumWheels() - 1);

		updateWheelTransformsWS(wheel, false);
		updateWheelTransform(getNumWheels() - 1, false);
		return wheel;
	}

	public Transform getWheelTransformWS (int wheelIndex, Transform out) {
		assert (wheelIndex < getNumWheels());
		WheelInfo wheel = wheelInfo.getQuick(wheelIndex);
		out.set(wheel.worldTransform);
		return out;
	}

	public void updateWheelTransform (int wheelIndex) {
		updateWheelTransform(wheelIndex, true);
	}

	public void updateWheelTransform (int wheelIndex, boolean interpolatedTransform) {
		WheelInfo wheel = wheelInfo.getQuick(wheelIndex);
		updateWheelTransformsWS(wheel, interpolatedTransform);
		Stack stack = Stack.enter();
		Vector3 up = stack.allocVector3();
		up.set(wheel.raycastInfo.wheelDirectionWS).scl(-1);
		Vector3 right = wheel.raycastInfo.wheelAxleWS;
		Vector3 fwd = stack.allocVector3();
		fwd.set(up).crs(right);
		fwd.nor();
		// up = right.cross(fwd);
		// up.nor();

		// rotate around steering over de wheelAxleWS
		float steering = wheel.steering;

		Quaternion steeringOrn = stack.allocQuaternion();
		QuaternionUtil.setRotation(steeringOrn, up, steering); // wheel.m_steering);
		Matrix3 steeringMat = stack.allocMatrix3();
		MatrixUtil.setRotation(steeringMat, steeringOrn);

		Quaternion rotatingOrn = stack.allocQuaternion();
		QuaternionUtil.setRotation(rotatingOrn, right, -wheel.rotation);
		Matrix3 rotatingMat = stack.allocMatrix3();
		MatrixUtil.setRotation(rotatingMat, rotatingOrn);

		Matrix3 basis2 = stack.allocMatrix3();
		MatrixUtil.setRow(basis2, 0, right.x, fwd.x, up.x);
		MatrixUtil.setRow(basis2, 1, right.y, fwd.y, up.y);
		MatrixUtil.setRow(basis2, 2, right.z, fwd.z, up.z);

		Matrix3 wheelBasis = wheel.worldTransform.basis;
		wheelBasis.set(steeringMat).mul(rotatingMat);
		wheelBasis.mul(basis2);

		wheel.worldTransform.origin.x = wheel.raycastInfo.hardPointWS.x
			+ wheel.raycastInfo.suspensionLength * wheel.raycastInfo.wheelDirectionWS.x;
		wheel.worldTransform.origin.y = wheel.raycastInfo.hardPointWS.y
			+ wheel.raycastInfo.suspensionLength * wheel.raycastInfo.wheelDirectionWS.y;
		wheel.worldTransform.origin.z = wheel.raycastInfo.hardPointWS.z
			+ wheel.raycastInfo.suspensionLength * wheel.raycastInfo.wheelDirectionWS.z;
		stack.leave();
	}

	public void resetSuspension () {
		int i;
		for (i = 0; i < wheelInfo.size(); i++) {
			WheelInfo wheel = wheelInfo.getQuick(i);
			wheel.raycastInfo.suspensionLength = wheel.getSuspensionRestLength();
			wheel.suspensionRelativeVelocity = 0f;

			wheel.raycastInfo.contactNormalWS.set(wheel.raycastInfo.wheelDirectionWS).scl(-1);
			// wheel_info.setContactFriction(btScalar(0.0));
			wheel.clippedInvContactDotSuspension = 1f;
		}
	}

	public void updateWheelTransformsWS (WheelInfo wheel) {
		updateWheelTransformsWS(wheel, true);
	}

	public void updateWheelTransformsWS (WheelInfo wheel, boolean interpolatedTransform) {
		wheel.raycastInfo.isInContact = false;
		Stack stack = Stack.enter();

		Transform chassisTrans = getChassisWorldTransform(stack.allocTransform());
		if (interpolatedTransform && (getRigidBody().getMotionState() != null)) {
			getRigidBody().getMotionState().getWorldTransform(chassisTrans);
		}

		wheel.raycastInfo.hardPointWS.set(wheel.chassisConnectionPointCS);
		chassisTrans.transform(wheel.raycastInfo.hardPointWS);

		wheel.raycastInfo.wheelDirectionWS.set(wheel.wheelDirectionCS);
		wheel.raycastInfo.wheelDirectionWS.mul(chassisTrans.basis);

		wheel.raycastInfo.wheelAxleWS.set(wheel.wheelAxleCS);
		wheel.raycastInfo.wheelAxleWS.mul(chassisTrans.basis);
		stack.leave();
	}

	public float rayCast (WheelInfo wheel) {
		Stack stack = Stack.enter();
		updateWheelTransformsWS(wheel, false);

		float depth = -1f;

		float raylen = wheel.getSuspensionRestLength() + wheel.wheelsRadius;

		Vector3 rayvector = stack.allocVector3();
		rayvector.set(wheel.raycastInfo.wheelDirectionWS).scl(raylen);
		Vector3 source = wheel.raycastInfo.hardPointWS;
		wheel.raycastInfo.contactPointWS.set(source).add(rayvector);
		Vector3 target = wheel.raycastInfo.contactPointWS;

		float param = 0f;

		VehicleRaycasterResult rayResults = new VehicleRaycasterResult();

		assert (vehicleRaycaster != null);

		Object object = vehicleRaycaster.castRay(source, target, rayResults);

		wheel.raycastInfo.groundObject = null;

		if (object != null) {
			param = rayResults.distFraction;
			depth = raylen * rayResults.distFraction;
			wheel.raycastInfo.contactNormalWS.set(rayResults.hitNormalInWorld);
			wheel.raycastInfo.isInContact = true;

			wheel.raycastInfo.groundObject = s_fixedObject; // todo for driving on dynamic/movable objects!;
			// wheel.m_raycastInfo.m_groundObject = object;

			float hitDistance = param * raylen;
			wheel.raycastInfo.suspensionLength = hitDistance - wheel.wheelsRadius;
			// clamp on max suspension travel

			float minSuspensionLength = wheel.getSuspensionRestLength() - wheel.maxSuspensionTravelCm * 0.01f;
			float maxSuspensionLength = wheel.getSuspensionRestLength() + wheel.maxSuspensionTravelCm * 0.01f;
			if (wheel.raycastInfo.suspensionLength < minSuspensionLength) {
				wheel.raycastInfo.suspensionLength = minSuspensionLength;
			}
			if (wheel.raycastInfo.suspensionLength > maxSuspensionLength) {
				wheel.raycastInfo.suspensionLength = maxSuspensionLength;
			}

			wheel.raycastInfo.contactPointWS.set(rayResults.hitPointInWorld);

			float denominator = wheel.raycastInfo.contactNormalWS.dot(wheel.raycastInfo.wheelDirectionWS);

			Vector3 chassis_velocity_at_contactPoint = stack.allocVector3();
			Vector3 relpos = stack.allocVector3();
			relpos.set(wheel.raycastInfo.contactPointWS).sub(getRigidBody().getCenterOfMassPosition(stack.allocVector3()));

			getRigidBody().getVelocityInLocalPoint(relpos, chassis_velocity_at_contactPoint);

			float projVel = wheel.raycastInfo.contactNormalWS.dot(chassis_velocity_at_contactPoint);

			if (denominator >= -0.1f) {
				wheel.suspensionRelativeVelocity = 0f;
				wheel.clippedInvContactDotSuspension = 1f / 0.1f;
			} else {
				float inv = -1f / denominator;
				wheel.suspensionRelativeVelocity = projVel * inv;
				wheel.clippedInvContactDotSuspension = inv;
			}

		} else {
			// put wheel info as in rest position
			wheel.raycastInfo.suspensionLength = wheel.getSuspensionRestLength();
			wheel.suspensionRelativeVelocity = 0f;
			wheel.raycastInfo.contactNormalWS.set(wheel.raycastInfo.wheelDirectionWS).scl(-1);
			wheel.clippedInvContactDotSuspension = 1f;
		}

		stack.leave();
		return depth;
	}

	public Transform getChassisWorldTransform (Transform out) {
		/*
		 * if (getRigidBody()->getMotionState()) { btTransform chassisWorldTrans;
		 * getRigidBody()->getMotionState()->getWorldTransform(chassisWorldTrans); return chassisWorldTrans; }
		 */

		return getRigidBody().getCenterOfMassTransform(out);
	}

	public void updateVehicle (float step) {
		for (int i = 0; i < getNumWheels(); i++) {
			updateWheelTransform(i, false);
		}

		Stack stack = Stack.enter();
		Vector3 tmp = stack.allocVector3();

		currentVehicleSpeedKmHour = 3.6f * getRigidBody().getLinearVelocity(tmp).len();

		Transform chassisTrans = getChassisWorldTransform(stack.allocTransform());

		Vector3 forwardW = stack.allocVector3();
		forwardW.set(MatrixUtil.getElement(chassisTrans.basis, 0, indexForwardAxis),
			MatrixUtil.getElement(chassisTrans.basis, 1, indexForwardAxis),
			MatrixUtil.getElement(chassisTrans.basis, 2, indexForwardAxis));

		if (forwardW.dot(getRigidBody().getLinearVelocity(tmp)) < 0f) {
			currentVehicleSpeedKmHour *= -1f;
		}

		//
		// simulate suspension
		//

		int i = 0;
		for (i = 0; i < wheelInfo.size(); i++) {
			float depth;
			depth = rayCast(wheelInfo.getQuick(i));
		}

		updateSuspension(step);

		for (i = 0; i < wheelInfo.size(); i++) {
			// apply suspension force
			WheelInfo wheel = wheelInfo.getQuick(i);

			float suspensionForce = wheel.wheelsSuspensionForce;

			float gMaxSuspensionForce = 6000f;
			if (suspensionForce > gMaxSuspensionForce) {
				suspensionForce = gMaxSuspensionForce;
			}
			Vector3 impulse = stack.allocVector3();
			impulse.set(wheel.raycastInfo.contactNormalWS).scl(suspensionForce * step);
			Vector3 relpos = stack.allocVector3();
			relpos.set(wheel.raycastInfo.contactPointWS).sub(getRigidBody().getCenterOfMassPosition(tmp));

			getRigidBody().applyImpulse(impulse, relpos);
		}

		updateFriction(step);

		for (i = 0; i < wheelInfo.size(); i++) {
			WheelInfo wheel = wheelInfo.getQuick(i);
			Vector3 relpos = stack.allocVector3();
			relpos.set(wheel.raycastInfo.hardPointWS).sub(getRigidBody().getCenterOfMassPosition(tmp));
			Vector3 vel = getRigidBody().getVelocityInLocalPoint(relpos, stack.allocVector3());

			if (wheel.raycastInfo.isInContact) {
				Transform chassisWorldTransform = getChassisWorldTransform(stack.allocTransform());

				Vector3 fwd = stack.allocVector3();
				fwd.set(MatrixUtil.getElement(chassisWorldTransform.basis, 0, indexForwardAxis),
					MatrixUtil.getElement(chassisWorldTransform.basis, 1, indexForwardAxis),
					MatrixUtil.getElement(chassisWorldTransform.basis, 2, indexForwardAxis));

				float proj = fwd.dot(wheel.raycastInfo.contactNormalWS);
				tmp.set(wheel.raycastInfo.contactNormalWS).scl(proj);
				fwd.sub(tmp);

				float proj2 = fwd.dot(vel);

				wheel.deltaRotation = (proj2 * step) / (wheel.wheelsRadius);
				wheel.rotation += wheel.deltaRotation;

			} else {
				wheel.rotation += wheel.deltaRotation;
			}

			wheel.deltaRotation *= 0.99f; // damping of rotation when not in contact
		}
		stack.leave();
	}

	public void setSteeringValue (float steering, int wheel) {
		assert (wheel >= 0 && wheel < getNumWheels());

		WheelInfo wheel_info = getWheelInfo(wheel);
		wheel_info.steering = steering;
	}

	public float getSteeringValue (int wheel) {
		return getWheelInfo(wheel).steering;
	}

	public void applyEngineForce (float force, int wheel) {
		assert (wheel >= 0 && wheel < getNumWheels());
		WheelInfo wheel_info = getWheelInfo(wheel);
		wheel_info.engineForce = force;
	}

	public WheelInfo getWheelInfo (int index) {
		assert ((index >= 0) && (index < getNumWheels()));

		return wheelInfo.getQuick(index);
	}

	public void setBrake (float brake, int wheelIndex) {
		assert ((wheelIndex >= 0) && (wheelIndex < getNumWheels()));
		getWheelInfo(wheelIndex).brake = brake;
	}

	public void updateSuspension (float deltaTime) {
		float chassisMass = 1f / chassisBody.getInvMass();

		for (int w_it = 0; w_it < getNumWheels(); w_it++) {
			WheelInfo wheel_info = wheelInfo.getQuick(w_it);

			if (wheel_info.raycastInfo.isInContact) {
				float force;
				// Spring
				{
					float susp_length = wheel_info.getSuspensionRestLength();
					float current_length = wheel_info.raycastInfo.suspensionLength;

					float length_diff = (susp_length - current_length);

					force = wheel_info.suspensionStiffness * length_diff * wheel_info.clippedInvContactDotSuspension;
				}

				// Damper
				{
					float projected_rel_vel = wheel_info.suspensionRelativeVelocity;
					{
						float susp_damping;
						if (projected_rel_vel < 0f) {
							susp_damping = wheel_info.wheelsDampingCompression;
						} else {
							susp_damping = wheel_info.wheelsDampingRelaxation;
						}
						force -= susp_damping * projected_rel_vel;
					}
				}

				// RESULT
				wheel_info.wheelsSuspensionForce = force * chassisMass;
				if (wheel_info.wheelsSuspensionForce < 0f) {
					wheel_info.wheelsSuspensionForce = 0f;
				}
			} else {
				wheel_info.wheelsSuspensionForce = 0f;
			}
		}
	}

	private float calcRollingFriction (WheelContactPoint contactPoint) {
		Stack stack = Stack.enter();
		Vector3 tmp = stack.allocVector3();

		float j1 = 0f;

		Vector3 contactPosWorld = contactPoint.frictionPositionWorld;

		Vector3 rel_pos1 = stack.allocVector3();
		rel_pos1.set(contactPosWorld).sub(contactPoint.body0.getCenterOfMassPosition(tmp));
		Vector3 rel_pos2 = stack.allocVector3();
		rel_pos2.set(contactPosWorld).sub(contactPoint.body1.getCenterOfMassPosition(tmp));

		float maxImpulse = contactPoint.maxImpulse;

		Vector3 vel1 = contactPoint.body0.getVelocityInLocalPoint(rel_pos1, stack.allocVector3());
		Vector3 vel2 = contactPoint.body1.getVelocityInLocalPoint(rel_pos2, stack.allocVector3());
		Vector3 vel = stack.allocVector3();
		vel.set(vel1).sub(vel2);

		float vrel = contactPoint.frictionDirectionWorld.dot(vel);

		// calculate j that moves us to zero relative velocity
		j1 = -vrel * contactPoint.jacDiagABInv;
		j1 = Math.min(j1, maxImpulse);
		j1 = Math.max(j1, -maxImpulse);
		stack.leave();
		return j1;
	}

	public void updateFriction (float timeStep) {
		// calculate the impulse, so that the wheels don't move sidewards
		int numWheel = getNumWheels();
		if (numWheel == 0) {
			return;
		}

		MiscUtil.resize(forwardWS, numWheel, Suppliers.NEW_VECTOR3_SUPPLIER);
		MiscUtil.resize(axle, numWheel, Suppliers.NEW_VECTOR3_SUPPLIER);
		MiscUtil.resize(forwardImpulse, numWheel, 0f);
		MiscUtil.resize(sideImpulse, numWheel, 0f);

		Stack stack = Stack.enter();
		Vector3 tmp = stack.allocVector3();

		int numWheelsOnGround = 0;

		// collapse all those loops into one!
		for (int i = 0; i < getNumWheels(); i++) {
			WheelInfo wheel_info = wheelInfo.getQuick(i);
			RigidBody groundObject = (RigidBody)wheel_info.raycastInfo.groundObject;
			if (groundObject != null) {
				numWheelsOnGround++;
			}
			sideImpulse.set(i, 0f);
			forwardImpulse.set(i, 0f);
		}

		{
			Transform wheelTrans = stack.allocTransform();
			for (int i = 0; i < getNumWheels(); i++) {

				WheelInfo wheel_info = wheelInfo.getQuick(i);

				RigidBody groundObject = (RigidBody)wheel_info.raycastInfo.groundObject;

				if (groundObject != null) {
					getWheelTransformWS(i, wheelTrans);

					Matrix3 wheelBasis0 = stack.alloc(wheelTrans.basis);
					axle.getQuick(i).set(MatrixUtil.getElement(wheelBasis0, 0, indexRightAxis),
						MatrixUtil.getElement(wheelBasis0, 1, indexRightAxis), MatrixUtil.getElement(wheelBasis0, 2, indexRightAxis));

					Vector3 surfNormalWS = wheel_info.raycastInfo.contactNormalWS;
					float proj = axle.getQuick(i).dot(surfNormalWS);
					tmp.set(surfNormalWS).scl(proj);
					axle.getQuick(i).sub(tmp);
					axle.getQuick(i).nor();

					forwardWS.getQuick(i).set(surfNormalWS).crs(axle.getQuick(i));
					forwardWS.getQuick(i).nor();

					float[] floatPtr = floatArrays.getFixed(1);
					ContactConstraint.resolveSingleBilateral(chassisBody, wheel_info.raycastInfo.contactPointWS, groundObject,
						wheel_info.raycastInfo.contactPointWS, 0f, axle.getQuick(i), floatPtr, timeStep);
					sideImpulse.set(i, floatPtr[0]);
					floatArrays.release(floatPtr);

					sideImpulse.set(i, sideImpulse.get(i) * sideFrictionStiffness2);
				}
			}
		}

		float sideFactor = 1f;
		float fwdFactor = 0.5f;

		boolean sliding = false;
		{
			for (int wheel = 0; wheel < getNumWheels(); wheel++) {
				WheelInfo wheel_info = wheelInfo.getQuick(wheel);
				RigidBody groundObject = (RigidBody)wheel_info.raycastInfo.groundObject;

				float rollingFriction = 0f;

				if (groundObject != null) {
					if (wheel_info.engineForce != 0f) {
						rollingFriction = wheel_info.engineForce * timeStep;
					} else {
						float defaultRollingFrictionImpulse = 0f;
						float maxImpulse = wheel_info.brake != 0f ? wheel_info.brake : defaultRollingFrictionImpulse;
						WheelContactPoint contactPt = new WheelContactPoint(chassisBody, groundObject,
							wheel_info.raycastInfo.contactPointWS, forwardWS.getQuick(wheel), maxImpulse);
						rollingFriction = calcRollingFriction(contactPt);
					}
				}

				// switch between active rolling (throttle), braking and non-active rolling friction (no throttle/break)

				forwardImpulse.set(wheel, 0f);
				wheelInfo.getQuick(wheel).skidInfo = 1f;

				if (groundObject != null) {
					wheelInfo.getQuick(wheel).skidInfo = 1f;

					float maximp = wheel_info.wheelsSuspensionForce * timeStep * wheel_info.frictionSlip;
					float maximpSide = maximp;

					float maximpSquared = maximp * maximpSide;

					forwardImpulse.set(wheel, rollingFriction); // wheelInfo.m_engineForce* timeStep;

					float x = (forwardImpulse.get(wheel)) * fwdFactor;
					float y = (sideImpulse.get(wheel)) * sideFactor;

					float impulseSquared = (x * x + y * y);

					if (impulseSquared > maximpSquared) {
						sliding = true;

						float factor = maximp / (float)Math.sqrt(impulseSquared);

						wheelInfo.getQuick(wheel).skidInfo *= factor;
					}
				}

			}
		}

		if (sliding) {
			for (int wheel = 0; wheel < getNumWheels(); wheel++) {
				if (sideImpulse.get(wheel) != 0f) {
					if (wheelInfo.getQuick(wheel).skidInfo < 1f) {
						forwardImpulse.set(wheel, forwardImpulse.get(wheel) * wheelInfo.getQuick(wheel).skidInfo);
						sideImpulse.set(wheel, sideImpulse.get(wheel) * wheelInfo.getQuick(wheel).skidInfo);
					}
				}
			}
		}

		// apply the impulses
		{
			for (int wheel = 0; wheel < getNumWheels(); wheel++) {
				WheelInfo wheel_info = wheelInfo.getQuick(wheel);

				Vector3 rel_pos = stack.allocVector3();
				rel_pos.set(wheel_info.raycastInfo.contactPointWS).sub(chassisBody.getCenterOfMassPosition(tmp));

				if (forwardImpulse.get(wheel) != 0f) {
					tmp.set(forwardWS.getQuick(wheel)).scl(forwardImpulse.get(wheel));
					chassisBody.applyImpulse(tmp, rel_pos);
				}
				if (sideImpulse.get(wheel) != 0f) {
					RigidBody groundObject = (RigidBody)wheelInfo.getQuick(wheel).raycastInfo.groundObject;

					Vector3 rel_pos2 = stack.allocVector3();
					rel_pos2.set(wheel_info.raycastInfo.contactPointWS).sub(groundObject.getCenterOfMassPosition(tmp));

					Vector3 sideImp = stack.allocVector3();
					sideImp.set(axle.getQuick(wheel)).scl(sideImpulse.get(wheel));

					rel_pos.z *= wheel_info.rollInfluence;
					chassisBody.applyImpulse(sideImp, rel_pos);

					// apply friction impulse on the ground
					tmp.set(sideImp).scl(-1);
					groundObject.applyImpulse(tmp, rel_pos2);
				}
			}
		}
		stack.leave();
	}

	@Override
	public void buildJacobian () {
		// not yet
	}

	@Override
	public void solveConstraint (float timeStep) {
		// not yet
	}

	public int getNumWheels () {
		return wheelInfo.size();
	}

	public void setPitchControl (float pitch) {
		this.pitchControl = pitch;
	}

	public RigidBody getRigidBody () {
		return chassisBody;
	}

	public int getRightAxis () {
		return indexRightAxis;
	}

	public int getUpAxis () {
		return indexUpAxis;
	}

	public int getForwardAxis () {
		return indexForwardAxis;
	}

	/** Worldspace forward vector. */
	public Vector3 getForwardVector (Vector3 out) {
		Stack stack = Stack.enter();
		Transform chassisTrans = getChassisWorldTransform(stack.allocTransform());

		out.set(MatrixUtil.getElement(chassisTrans.basis, 0, indexForwardAxis),
			MatrixUtil.getElement(chassisTrans.basis, 1, indexForwardAxis),
			MatrixUtil.getElement(chassisTrans.basis, 2, indexForwardAxis));

		stack.leave();
		return out;
	}

	/** Velocity of vehicle (positive if velocity vector has same direction as foward vector). */
	public float getCurrentSpeedKmHour () {
		return currentVehicleSpeedKmHour;
	}

	public void setCoordinateSystem (int rightIndex, int upIndex, int forwardIndex) {
		this.indexRightAxis = rightIndex;
		this.indexUpAxis = upIndex;
		this.indexForwardAxis = forwardIndex;
	}

	// //////////////////////////////////////////////////////////////////////////

	private static class WheelContactPoint {
		public RigidBody body0;
		public RigidBody body1;
		public final Vector3 frictionPositionWorld = new Vector3();
		public final Vector3 frictionDirectionWorld = new Vector3();
		public float jacDiagABInv;
		public float maxImpulse;

		public WheelContactPoint (RigidBody body0, RigidBody body1, Vector3 frictionPosWorld, Vector3 frictionDirectionWorld,
			float maxImpulse) {
			this.body0 = body0;
			this.body1 = body1;
			this.frictionPositionWorld.set(frictionPosWorld);
			this.frictionDirectionWorld.set(frictionDirectionWorld);
			this.maxImpulse = maxImpulse;

			float denom0 = body0.computeImpulseDenominator(frictionPosWorld, frictionDirectionWorld);
			float denom1 = body1.computeImpulseDenominator(frictionPosWorld, frictionDirectionWorld);
			float relaxation = 1f;
			jacDiagABInv = relaxation / (denom0 + denom1);
		}
	}

}
