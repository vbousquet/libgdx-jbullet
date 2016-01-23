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

/* Hinge Constraint by Dirk Gregorius. Limits added by Marcus Hennix at Starbreeze Studios */

package com.bulletphysics.dynamics.constraintsolver;

import com.badlogic.gdx.math.Matrix3;
import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import com.bulletphysics.BulletGlobals;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.QuaternionUtil;
import com.bulletphysics.linearmath.ScalarUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.TransformUtil;
import com.bulletphysics.util.Stack;

/** Hinge constraint between two rigid bodies each with a pivot point that descibes the axis location in local space. Axis defines
 * the orientation of the hinge axis.
 * 
 * @author jezek2 */
public class HingeConstraint extends TypedConstraint {

	private JacobianEntry[] jac/* [3] */ = new JacobianEntry[] {new JacobianEntry(), new JacobianEntry(), new JacobianEntry()}; // 3
																																											// orthogonal
																																											// linear
																																											// constraints
	private JacobianEntry[] jacAng/* [3] */ = new JacobianEntry[] {new JacobianEntry(), new JacobianEntry(), new JacobianEntry()}; // 2
																																												// orthogonal
																																												// angular
																																												// constraints+
																																												// 1
																																												// for
																																												// limit/motor

	private final Transform rbAFrame = new Transform(); // constraint axii. Assumes z is hinge axis.
	private final Transform rbBFrame = new Transform();

	private float motorTargetVelocity;
	private float maxMotorImpulse;

	private float limitSoftness;
	private float biasFactor;
	private float relaxationFactor;

	private float lowerLimit;
	private float upperLimit;

	private float kHinge;

	private float limitSign;
	private float correction;

	private float accLimitImpulse;

	private boolean angularOnly;
	private boolean enableAngularMotor;
	private boolean solveLimit;

	public HingeConstraint () {
		super(TypedConstraintType.HINGE_CONSTRAINT_TYPE);
		enableAngularMotor = false;
	}

	public HingeConstraint (RigidBody rbA, RigidBody rbB, Vector3 pivotInA, Vector3 pivotInB, Vector3 axisInA, Vector3 axisInB) {
		super(TypedConstraintType.HINGE_CONSTRAINT_TYPE, rbA, rbB);
		Stack stack = Stack.enter();
		angularOnly = false;
		enableAngularMotor = false;

		rbAFrame.origin.set(pivotInA);

		// since no frame is given, assume this to be zero angle and just pick rb transform axis
		Vector3 rbAxisA1 = stack.allocVector3();
		Vector3 rbAxisA2 = stack.allocVector3();

		Transform centerOfMassA = rbA.getCenterOfMassTransform(stack.allocTransform());
		MatrixUtil.getColumn(centerOfMassA.basis, 0, rbAxisA1);
		float projection = axisInA.dot(rbAxisA1);

		if (projection >= 1.0f - BulletGlobals.SIMD_EPSILON) {
			MatrixUtil.getColumn(centerOfMassA.basis, 2, rbAxisA1);
			rbAxisA1.scl(-1);
			MatrixUtil.getColumn(centerOfMassA.basis, 1, rbAxisA2);
		} else if (projection <= -1.0f + BulletGlobals.SIMD_EPSILON) {
			MatrixUtil.getColumn(centerOfMassA.basis, 2, rbAxisA1);
			MatrixUtil.getColumn(centerOfMassA.basis, 1, rbAxisA2);
		} else {
			rbAxisA2.set(axisInA).crs(rbAxisA1);
			rbAxisA1.set(rbAxisA2).crs(axisInA);
		}

		MatrixUtil.setRow(rbAFrame.basis, 0, rbAxisA1.x, rbAxisA2.x, axisInA.x);
		MatrixUtil.setRow(rbAFrame.basis, 1, rbAxisA1.y, rbAxisA2.y, axisInA.y);
		MatrixUtil.setRow(rbAFrame.basis, 2, rbAxisA1.z, rbAxisA2.z, axisInA.z);

		Quaternion rotationArc = QuaternionUtil.shortestArcQuat(axisInA, axisInB, stack.allocQuaternion());
		Vector3 rbAxisB1 = QuaternionUtil.quatRotate(rotationArc, rbAxisA1, stack.allocVector3());
		Vector3 rbAxisB2 = stack.allocVector3();
		rbAxisB2.set(axisInB).crs(rbAxisB1);

		rbBFrame.origin.set(pivotInB);
		MatrixUtil.setRow(rbBFrame.basis, 0, rbAxisB1.x, rbAxisB2.x, -axisInB.x);
		MatrixUtil.setRow(rbBFrame.basis, 1, rbAxisB1.y, rbAxisB2.y, -axisInB.y);
		MatrixUtil.setRow(rbBFrame.basis, 2, rbAxisB1.z, rbAxisB2.z, -axisInB.z);

		// start with free
		lowerLimit = 1e30f;
		upperLimit = -1e30f;
		biasFactor = 0.3f;
		relaxationFactor = 1.0f;
		limitSoftness = 0.9f;
		solveLimit = false;
		stack.leave();
	}

	public HingeConstraint (RigidBody rbA, Vector3 pivotInA, Vector3 axisInA) {
		super(TypedConstraintType.HINGE_CONSTRAINT_TYPE, rbA);
		angularOnly = false;
		enableAngularMotor = false;
		Stack stack = Stack.enter();
		// since no frame is given, assume this to be zero angle and just pick rb transform axis
		// fixed axis in worldspace
		Vector3 rbAxisA1 = stack.allocVector3();
		Transform centerOfMassA = rbA.getCenterOfMassTransform(stack.allocTransform());
		MatrixUtil.getColumn(centerOfMassA.basis, 0, rbAxisA1);

		float projection = rbAxisA1.dot(axisInA);
		if (projection > BulletGlobals.FLT_EPSILON) {
			rbAxisA1.scl(projection);
			rbAxisA1.sub(axisInA);
		} else {
			MatrixUtil.getColumn(centerOfMassA.basis, 1, rbAxisA1);
		}

		Vector3 rbAxisA2 = stack.allocVector3();
		rbAxisA2.set(axisInA).crs(rbAxisA1);

		rbAFrame.origin.set(pivotInA);
		MatrixUtil.setRow(rbAFrame.basis, 0, rbAxisA1.x, rbAxisA2.x, axisInA.x);
		MatrixUtil.setRow(rbAFrame.basis, 1, rbAxisA1.y, rbAxisA2.y, axisInA.y);
		MatrixUtil.setRow(rbAFrame.basis, 2, rbAxisA1.z, rbAxisA2.z, axisInA.z);

		Vector3 axisInB = stack.allocVector3();
		axisInB.set(axisInA).scl(-1);
		axisInB.mul(centerOfMassA.basis);

		Quaternion rotationArc = QuaternionUtil.shortestArcQuat(axisInA, axisInB, stack.allocQuaternion());
		Vector3 rbAxisB1 = QuaternionUtil.quatRotate(rotationArc, rbAxisA1, stack.allocVector3());
		Vector3 rbAxisB2 = stack.allocVector3();
		rbAxisB2.set(axisInB).crs(rbAxisB1);

		rbBFrame.origin.set(pivotInA);
		centerOfMassA.transform(rbBFrame.origin);
		MatrixUtil.setRow(rbBFrame.basis, 0, rbAxisB1.x, rbAxisB2.x, axisInB.x);
		MatrixUtil.setRow(rbBFrame.basis, 1, rbAxisB1.y, rbAxisB2.y, axisInB.y);
		MatrixUtil.setRow(rbBFrame.basis, 2, rbAxisB1.z, rbAxisB2.z, axisInB.z);

		// start with free
		lowerLimit = 1e30f;
		upperLimit = -1e30f;
		biasFactor = 0.3f;
		relaxationFactor = 1.0f;
		limitSoftness = 0.9f;
		solveLimit = false;
		stack.leave();
	}

	public HingeConstraint (RigidBody rbA, RigidBody rbB, Transform rbAFrame, Transform rbBFrame) {
		super(TypedConstraintType.HINGE_CONSTRAINT_TYPE, rbA, rbB);
		this.rbAFrame.set(rbAFrame);
		this.rbBFrame.set(rbBFrame);
		angularOnly = false;
		enableAngularMotor = false;

		// flip axis
		this.rbBFrame.basis.val[Matrix3.M02] *= -1f;
		this.rbBFrame.basis.val[Matrix3.M12] *= -1f;
		this.rbBFrame.basis.val[Matrix3.M22] *= -1f;

		// start with free
		lowerLimit = 1e30f;
		upperLimit = -1e30f;
		biasFactor = 0.3f;
		relaxationFactor = 1.0f;
		limitSoftness = 0.9f;
		solveLimit = false;
	}

	public HingeConstraint (RigidBody rbA, Transform rbAFrame) {
		super(TypedConstraintType.HINGE_CONSTRAINT_TYPE, rbA);
		this.rbAFrame.set(rbAFrame);
		this.rbBFrame.set(rbAFrame);
		angularOnly = false;
		enableAngularMotor = false;
		Stack stack = Stack.enter();

		// not providing rigidbody B means implicitly using worldspace for body B

		// flip axis
		this.rbBFrame.basis.val[Matrix3.M02] *= -1f;
		this.rbBFrame.basis.val[Matrix3.M12] *= -1f;
		this.rbBFrame.basis.val[Matrix3.M22] *= -1f;

		this.rbBFrame.origin.set(this.rbAFrame.origin);
		rbA.getCenterOfMassTransform(stack.allocTransform()).transform(this.rbBFrame.origin);

		// start with free
		lowerLimit = 1e30f;
		upperLimit = -1e30f;
		biasFactor = 0.3f;
		relaxationFactor = 1.0f;
		limitSoftness = 0.9f;
		solveLimit = false;
		stack.leave();
	}

	@Override
	public void buildJacobian () {
		Stack stack = Stack.enter();
		Vector3 tmp = stack.allocVector3();
		Vector3 tmp1 = stack.allocVector3();
		Vector3 tmp2 = stack.allocVector3();
		Vector3 tmpVec = stack.allocVector3();
		Matrix3 mat1 = stack.allocMatrix3();
		Matrix3 mat2 = stack.allocMatrix3();

		Transform centerOfMassA = rbA.getCenterOfMassTransform(stack.allocTransform());
		Transform centerOfMassB = rbB.getCenterOfMassTransform(stack.allocTransform());

		appliedImpulse = 0f;

		if (!angularOnly) {
			Vector3 pivotAInW = stack.alloc(rbAFrame.origin);
			centerOfMassA.transform(pivotAInW);

			Vector3 pivotBInW = stack.alloc(rbBFrame.origin);
			centerOfMassB.transform(pivotBInW);

			Vector3 relPos = stack.allocVector3();
			relPos.set(pivotBInW).sub(pivotAInW);

			Vector3[] normal/* [3] */ = new Vector3[] {stack.allocVector3(), stack.allocVector3(), stack.allocVector3()};
			if (relPos.len2() > BulletGlobals.FLT_EPSILON) {
				normal[0].set(relPos);
				normal[0].nor();
			} else {
				normal[0].set(1f, 0f, 0f);
			}

			TransformUtil.planeSpace1(normal[0], normal[1], normal[2]);

			for (int i = 0; i < 3; i++) {
				mat1.set(centerOfMassA.basis).transpose();
				mat2.set(centerOfMassB.basis).transpose();

				tmp1.set(pivotAInW).sub(rbA.getCenterOfMassPosition(tmpVec));
				tmp2.set(pivotBInW).sub(rbB.getCenterOfMassPosition(tmpVec));

				jac[i].init(mat1, mat2, tmp1, tmp2, normal[i], rbA.getInvInertiaDiagLocal(stack.allocVector3()), rbA.getInvMass(),
					rbB.getInvInertiaDiagLocal(stack.allocVector3()), rbB.getInvMass());
			}
		}

		// calculate two perpendicular jointAxis, orthogonal to hingeAxis
		// these two jointAxis require equal angular velocities for both bodies

		// this is unused for now, it's a todo
		Vector3 jointAxis0local = stack.allocVector3();
		Vector3 jointAxis1local = stack.allocVector3();

		MatrixUtil.getColumn(rbAFrame.basis, 2, tmp);
		TransformUtil.planeSpace1(tmp, jointAxis0local, jointAxis1local);

		// TODO: check this
		// getRigidBodyA().getCenterOfMassTransform().getBasis() * m_rbAFrame.getBasis().getColumn(2);

		Vector3 jointAxis0 = stack.alloc(jointAxis0local);
		jointAxis0.mul(centerOfMassA.basis);

		Vector3 jointAxis1 = stack.alloc(jointAxis1local);
		jointAxis1.mul(centerOfMassA.basis);

		Vector3 hingeAxisWorld = stack.allocVector3();
		MatrixUtil.getColumn(rbAFrame.basis, 2, hingeAxisWorld);
		hingeAxisWorld.mul(centerOfMassA.basis);

		mat1.set(centerOfMassA.basis).transpose();
		mat2.set(centerOfMassB.basis).transpose();
		jacAng[0].init(jointAxis0, mat1, mat2, rbA.getInvInertiaDiagLocal(stack.allocVector3()),
			rbB.getInvInertiaDiagLocal(stack.allocVector3()));

		// JAVA NOTE: reused mat1 and mat2, as recomputation is not needed
		jacAng[1].init(jointAxis1, mat1, mat2, rbA.getInvInertiaDiagLocal(stack.allocVector3()),
			rbB.getInvInertiaDiagLocal(stack.allocVector3()));

		// JAVA NOTE: reused mat1 and mat2, as recomputation is not needed
		jacAng[2].init(hingeAxisWorld, mat1, mat2, rbA.getInvInertiaDiagLocal(stack.allocVector3()),
			rbB.getInvInertiaDiagLocal(stack.allocVector3()));

		// Compute limit information
		float hingeAngle = getHingeAngle();

		// set bias, sign, clear accumulator
		correction = 0f;
		limitSign = 0f;
		solveLimit = false;
		accLimitImpulse = 0f;

		if (lowerLimit < upperLimit) {
			if (hingeAngle <= lowerLimit * limitSoftness) {
				correction = (lowerLimit - hingeAngle);
				limitSign = 1.0f;
				solveLimit = true;
			} else if (hingeAngle >= upperLimit * limitSoftness) {
				correction = upperLimit - hingeAngle;
				limitSign = -1.0f;
				solveLimit = true;
			}
		}

		// Compute K = J*W*J' for hinge axis
		Vector3 axisA = stack.allocVector3();
		MatrixUtil.getColumn(rbAFrame.basis, 2, axisA);
		axisA.mul(centerOfMassA.basis);

		kHinge = 1.0f
			/ (getRigidBodyA().computeAngularImpulseDenominator(axisA) + getRigidBodyB().computeAngularImpulseDenominator(axisA));
		stack.leave();
	}

	@Override
	public void solveConstraint (float timeStep) {
		Stack stack = Stack.enter();
		Vector3 tmp = stack.allocVector3();
		Vector3 tmp2 = stack.allocVector3();
		Vector3 tmpVec = stack.allocVector3();

		Transform centerOfMassA = rbA.getCenterOfMassTransform(stack.allocTransform());
		Transform centerOfMassB = rbB.getCenterOfMassTransform(stack.allocTransform());

		Vector3 pivotAInW = stack.alloc(rbAFrame.origin);
		centerOfMassA.transform(pivotAInW);

		Vector3 pivotBInW = stack.alloc(rbBFrame.origin);
		centerOfMassB.transform(pivotBInW);

		float tau = 0.3f;

		// linear part
		if (!angularOnly) {
			Vector3 rel_pos1 = stack.allocVector3();
			rel_pos1.set(pivotAInW).sub(rbA.getCenterOfMassPosition(tmpVec));

			Vector3 rel_pos2 = stack.allocVector3();
			rel_pos2.set(pivotBInW).sub(rbB.getCenterOfMassPosition(tmpVec));

			Vector3 vel1 = rbA.getVelocityInLocalPoint(rel_pos1, stack.allocVector3());
			Vector3 vel2 = rbB.getVelocityInLocalPoint(rel_pos2, stack.allocVector3());
			Vector3 vel = stack.allocVector3();
			vel.set(vel1).sub(vel2);

			for (int i = 0; i < 3; i++) {
				Vector3 normal = jac[i].linearJointAxis;
				float jacDiagABInv = 1f / jac[i].getDiagonal();

				float rel_vel;
				rel_vel = normal.dot(vel);
				// positional error (zeroth order error)
				tmp.set(pivotAInW).sub(pivotBInW);
				float depth = -(tmp).dot(normal); // this is the error projected on the normal
				float impulse = depth * tau / timeStep * jacDiagABInv - rel_vel * jacDiagABInv;
				appliedImpulse += impulse;
				Vector3 impulse_vector = stack.allocVector3();
				impulse_vector.set(normal).scl(impulse);

				tmp.set(pivotAInW).sub(rbA.getCenterOfMassPosition(tmpVec));
				rbA.applyImpulse(impulse_vector, tmp);

				tmp.set(impulse_vector).scl(-1);
				tmp2.set(pivotBInW).sub(rbB.getCenterOfMassPosition(tmpVec));
				rbB.applyImpulse(tmp, tmp2);
			}
		}

		{
			// solve angular part

			// get axes in world space
			Vector3 axisA = stack.allocVector3();
			MatrixUtil.getColumn(rbAFrame.basis, 2, axisA);
			axisA.mul(centerOfMassA.basis);

			Vector3 axisB = stack.allocVector3();
			MatrixUtil.getColumn(rbBFrame.basis, 2, axisB);
			axisB.mul(centerOfMassB.basis);

			Vector3 angVelA = getRigidBodyA().getAngularVelocity(stack.allocVector3());
			Vector3 angVelB = getRigidBodyB().getAngularVelocity(stack.allocVector3());

			Vector3 angVelAroundHingeAxisA = stack.allocVector3();
			angVelAroundHingeAxisA.set(axisA).scl(axisA.dot(angVelA));

			Vector3 angVelAroundHingeAxisB = stack.allocVector3();
			angVelAroundHingeAxisB.set(axisB).scl(axisB.dot(angVelB));

			Vector3 angAorthog = stack.allocVector3();
			angAorthog.set(angVelA).sub(angVelAroundHingeAxisA);

			Vector3 angBorthog = stack.allocVector3();
			angBorthog.set(angVelB).sub(angVelAroundHingeAxisB);

			Vector3 velrelOrthog = stack.allocVector3();
			velrelOrthog.set(angAorthog).sub(angBorthog);

			{
				// solve orthogonal angular velocity correction
				float relaxation = 1f;
				float len = velrelOrthog.len();
				if (len > 0.00001f) {
					Vector3 normal = stack.allocVector3();
					normal.set(velrelOrthog).nor();

					float denom = getRigidBodyA().computeAngularImpulseDenominator(normal)
						+ getRigidBodyB().computeAngularImpulseDenominator(normal);
					// scale for mass and relaxation
					// todo: expose this 0.9 factor to developer
					velrelOrthog.scl((1f / denom) * relaxationFactor);
				}

				// solve angular positional correction
				// TODO: check
				// Vector3 angularError = -axisA.cross(axisB) *(btScalar(1.)/timeStep);
				Vector3 angularError = stack.allocVector3();
				angularError.set(axisA).crs(axisB);
				angularError.scl(-1);
				angularError.scl(1f / timeStep);
				float len2 = angularError.len();
				if (len2 > 0.00001f) {
					Vector3 normal2 = stack.allocVector3();
					normal2.set(angularError).nor();

					float denom2 = getRigidBodyA().computeAngularImpulseDenominator(normal2)
						+ getRigidBodyB().computeAngularImpulseDenominator(normal2);
					angularError.scl((1f / denom2) * relaxation);
				}

				tmp.set(velrelOrthog).scl(-1);
				tmp.add(angularError);
				rbA.applyTorqueImpulse(tmp);

				tmp.set(velrelOrthog).sub(angularError);
				rbB.applyTorqueImpulse(tmp);

				// solve limit
				if (solveLimit) {
					tmp.set(angVelB).sub(angVelA);
					float amplitude = ((tmp).dot(axisA) * relaxationFactor + correction * (1f / timeStep) * biasFactor) * limitSign;

					float impulseMag = amplitude * kHinge;

					// Clamp the accumulated impulse
					float temp = accLimitImpulse;
					accLimitImpulse = Math.max(accLimitImpulse + impulseMag, 0f);
					impulseMag = accLimitImpulse - temp;

					Vector3 impulse = stack.allocVector3();
					impulse.set(axisA).scl(impulseMag * limitSign);

					rbA.applyTorqueImpulse(impulse);

					tmp.set(impulse).scl(-1);
					rbB.applyTorqueImpulse(tmp);
				}
			}

			// apply motor
			if (enableAngularMotor) {
				// todo: add limits too
				Vector3 angularLimit = stack.allocVector3();
				angularLimit.set(0f, 0f, 0f);

				Vector3 velrel = stack.allocVector3();
				velrel.set(angVelAroundHingeAxisA).sub(angVelAroundHingeAxisB);
				float projRelVel = velrel.dot(axisA);

				float desiredMotorVel = motorTargetVelocity;
				float motor_relvel = desiredMotorVel - projRelVel;

				float unclippedMotorImpulse = kHinge * motor_relvel;
				// todo: should clip against accumulated impulse
				float clippedMotorImpulse = unclippedMotorImpulse > maxMotorImpulse ? maxMotorImpulse : unclippedMotorImpulse;
				clippedMotorImpulse = clippedMotorImpulse < -maxMotorImpulse ? -maxMotorImpulse : clippedMotorImpulse;
				Vector3 motorImp = stack.allocVector3();
				motorImp.set(axisA).scl(clippedMotorImpulse);

				tmp.set(motorImp).add(angularLimit);
				rbA.applyTorqueImpulse(tmp);

				tmp.set(motorImp).scl(-1);
				tmp.sub(angularLimit);
				rbB.applyTorqueImpulse(tmp);
			}
		}
		stack.leave();
	}

	public void updateRHS (float timeStep) {
	}

	public float getHingeAngle () {
		Stack stack = Stack.enter();
		Transform centerOfMassA = rbA.getCenterOfMassTransform(stack.allocTransform());
		Transform centerOfMassB = rbB.getCenterOfMassTransform(stack.allocTransform());

		Vector3 refAxis0 = stack.allocVector3();
		MatrixUtil.getColumn(rbAFrame.basis, 0, refAxis0);
		refAxis0.mul(centerOfMassA.basis);

		Vector3 refAxis1 = stack.allocVector3();
		MatrixUtil.getColumn(rbAFrame.basis, 1, refAxis1);
		refAxis1.mul(centerOfMassA.basis);

		Vector3 swingAxis = stack.allocVector3();
		MatrixUtil.getColumn(rbBFrame.basis, 1, swingAxis);
		swingAxis.mul(centerOfMassB.basis);

		float result = ScalarUtil.atan2Fast(swingAxis.dot(refAxis0), swingAxis.dot(refAxis1));
		stack.leave();
		return result;
	}

	public void setAngularOnly (boolean angularOnly) {
		this.angularOnly = angularOnly;
	}

	public void enableAngularMotor (boolean enableMotor, float targetVelocity, float maxMotorImpulse) {
		this.enableAngularMotor = enableMotor;
		this.motorTargetVelocity = targetVelocity;
		this.maxMotorImpulse = maxMotorImpulse;
	}

	public void setLimit (float low, float high) {
		setLimit(low, high, 0.9f, 0.3f, 1.0f);
	}

	public void setLimit (float low, float high, float _softness, float _biasFactor, float _relaxationFactor) {
		lowerLimit = low;
		upperLimit = high;

		limitSoftness = _softness;
		biasFactor = _biasFactor;
		relaxationFactor = _relaxationFactor;
	}

	public float getLowerLimit () {
		return lowerLimit;
	}

	public float getUpperLimit () {
		return upperLimit;
	}

	public Transform getAFrame (Transform out) {
		out.set(rbAFrame);
		return out;
	}

	public Transform getBFrame (Transform out) {
		out.set(rbBFrame);
		return out;
	}

	public boolean getSolveLimit () {
		return solveLimit;
	}

	public float getLimitSign () {
		return limitSign;
	}

	public boolean getAngularOnly () {
		return angularOnly;
	}

	public boolean getEnableAngularMotor () {
		return enableAngularMotor;
	}

	public float getMotorTargetVelosity () {
		return motorTargetVelocity;
	}

	public float getMaxMotorImpulse () {
		return maxMotorImpulse;
	}

}
