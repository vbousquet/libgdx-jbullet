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

/*
Added by Roman Ponomarev (rponom@gmail.com)
April 04, 2008

TODO:
 - add clamping od accumulated impulse to improve stability
 - add conversion for ODE constraint solver
*/

package com.bulletphysics.dynamics.constraintsolver;

import com.badlogic.gdx.math.Matrix3;
import com.badlogic.gdx.math.Vector3;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.Stack;

// JAVA NOTE: SliderConstraint from 2.71

/** @author jezek2 */
public class SliderConstraint extends TypedConstraint {

	public static final float SLIDER_CONSTRAINT_DEF_SOFTNESS = 1.0f;
	public static final float SLIDER_CONSTRAINT_DEF_DAMPING = 1.0f;
	public static final float SLIDER_CONSTRAINT_DEF_RESTITUTION = 0.7f;

	protected final Transform frameInA = new Transform();
	protected final Transform frameInB = new Transform();
	// use frameA fo define limits, if true
	protected boolean useLinearReferenceFrameA;
	// linear limits
	protected float lowerLinLimit;
	protected float upperLinLimit;
	// angular limits
	protected float lowerAngLimit;
	protected float upperAngLimit;
	// softness, restitution and damping for different cases
	// DirLin - moving inside linear limits
	// LimLin - hitting linear limit
	// DirAng - moving inside angular limits
	// LimAng - hitting angular limit
	// OrthoLin, OrthoAng - against constraint axis
	protected float softnessDirLin;
	protected float restitutionDirLin;
	protected float dampingDirLin;
	protected float softnessDirAng;
	protected float restitutionDirAng;
	protected float dampingDirAng;
	protected float softnessLimLin;
	protected float restitutionLimLin;
	protected float dampingLimLin;
	protected float softnessLimAng;
	protected float restitutionLimAng;
	protected float dampingLimAng;
	protected float softnessOrthoLin;
	protected float restitutionOrthoLin;
	protected float dampingOrthoLin;
	protected float softnessOrthoAng;
	protected float restitutionOrthoAng;
	protected float dampingOrthoAng;

	// for interlal use
	protected boolean solveLinLim;
	protected boolean solveAngLim;

	protected JacobianEntry[] jacLin = new JacobianEntry[/* 3 */] {new JacobianEntry(), new JacobianEntry(), new JacobianEntry()};
	protected float[] jacLinDiagABInv = new float[3];

	protected JacobianEntry[] jacAng = new JacobianEntry[/* 3 */] {new JacobianEntry(), new JacobianEntry(), new JacobianEntry()};

	protected float timeStep;
	protected final Transform calculatedTransformA = new Transform();
	protected final Transform calculatedTransformB = new Transform();

	protected final Vector3 sliderAxis = new Vector3();
	protected final Vector3 realPivotAInW = new Vector3();
	protected final Vector3 realPivotBInW = new Vector3();
	protected final Vector3 projPivotInW = new Vector3();
	protected final Vector3 delta = new Vector3();
	protected final Vector3 depth = new Vector3();
	protected final Vector3 relPosA = new Vector3();
	protected final Vector3 relPosB = new Vector3();

	protected float linPos;

	protected float angDepth;
	protected float kAngle;

	protected boolean poweredLinMotor;
	protected float targetLinMotorVelocity;
	protected float maxLinMotorForce;
	protected float accumulatedLinMotorImpulse;

	protected boolean poweredAngMotor;
	protected float targetAngMotorVelocity;
	protected float maxAngMotorForce;
	protected float accumulatedAngMotorImpulse;

	public SliderConstraint () {
		super(TypedConstraintType.SLIDER_CONSTRAINT_TYPE);
		useLinearReferenceFrameA = true;
		initParams();
	}

	public SliderConstraint (RigidBody rbA, RigidBody rbB, Transform frameInA, Transform frameInB,
		boolean useLinearReferenceFrameA) {
		super(TypedConstraintType.SLIDER_CONSTRAINT_TYPE, rbA, rbB);
		this.frameInA.set(frameInA);
		this.frameInB.set(frameInB);
		this.useLinearReferenceFrameA = useLinearReferenceFrameA;
		initParams();
	}

	protected void initParams () {
		lowerLinLimit = 1f;
		upperLinLimit = -1f;
		lowerAngLimit = 0f;
		upperAngLimit = 0f;
		softnessDirLin = SLIDER_CONSTRAINT_DEF_SOFTNESS;
		restitutionDirLin = SLIDER_CONSTRAINT_DEF_RESTITUTION;
		dampingDirLin = 0f;
		softnessDirAng = SLIDER_CONSTRAINT_DEF_SOFTNESS;
		restitutionDirAng = SLIDER_CONSTRAINT_DEF_RESTITUTION;
		dampingDirAng = 0f;
		softnessOrthoLin = SLIDER_CONSTRAINT_DEF_SOFTNESS;
		restitutionOrthoLin = SLIDER_CONSTRAINT_DEF_RESTITUTION;
		dampingOrthoLin = SLIDER_CONSTRAINT_DEF_DAMPING;
		softnessOrthoAng = SLIDER_CONSTRAINT_DEF_SOFTNESS;
		restitutionOrthoAng = SLIDER_CONSTRAINT_DEF_RESTITUTION;
		dampingOrthoAng = SLIDER_CONSTRAINT_DEF_DAMPING;
		softnessLimLin = SLIDER_CONSTRAINT_DEF_SOFTNESS;
		restitutionLimLin = SLIDER_CONSTRAINT_DEF_RESTITUTION;
		dampingLimLin = SLIDER_CONSTRAINT_DEF_DAMPING;
		softnessLimAng = SLIDER_CONSTRAINT_DEF_SOFTNESS;
		restitutionLimAng = SLIDER_CONSTRAINT_DEF_RESTITUTION;
		dampingLimAng = SLIDER_CONSTRAINT_DEF_DAMPING;

		poweredLinMotor = false;
		targetLinMotorVelocity = 0f;
		maxLinMotorForce = 0f;
		accumulatedLinMotorImpulse = 0f;

		poweredAngMotor = false;
		targetAngMotorVelocity = 0f;
		maxAngMotorForce = 0f;
		accumulatedAngMotorImpulse = 0f;
	}

	@Override
	public void buildJacobian () {
		if (useLinearReferenceFrameA) {
			buildJacobianInt(rbA, rbB, frameInA, frameInB);
		} else {
			buildJacobianInt(rbB, rbA, frameInB, frameInA);
		}
	}

	@Override
	public void solveConstraint (float timeStep) {
		this.timeStep = timeStep;
		if (useLinearReferenceFrameA) {
			solveConstraintInt(rbA, rbB);
		} else {
			solveConstraintInt(rbB, rbA);
		}
	}

	public Transform getCalculatedTransformA (Transform out) {
		out.set(calculatedTransformA);
		return out;
	}

	public Transform getCalculatedTransformB (Transform out) {
		out.set(calculatedTransformB);
		return out;
	}

	public Transform getFrameOffsetA (Transform out) {
		out.set(frameInA);
		return out;
	}

	public Transform getFrameOffsetB (Transform out) {
		out.set(frameInB);
		return out;
	}

	public float getLowerLinLimit () {
		return lowerLinLimit;
	}

	public void setLowerLinLimit (float lowerLimit) {
		this.lowerLinLimit = lowerLimit;
	}

	public float getUpperLinLimit () {
		return upperLinLimit;
	}

	public void setUpperLinLimit (float upperLimit) {
		this.upperLinLimit = upperLimit;
	}

	public float getLowerAngLimit () {
		return lowerAngLimit;
	}

	public void setLowerAngLimit (float lowerLimit) {
		this.lowerAngLimit = lowerLimit;
	}

	public float getUpperAngLimit () {
		return upperAngLimit;
	}

	public void setUpperAngLimit (float upperLimit) {
		this.upperAngLimit = upperLimit;
	}

	public boolean getUseLinearReferenceFrameA () {
		return useLinearReferenceFrameA;
	}

	public float getSoftnessDirLin () {
		return softnessDirLin;
	}

	public float getRestitutionDirLin () {
		return restitutionDirLin;
	}

	public float getDampingDirLin () {
		return dampingDirLin;
	}

	public float getSoftnessDirAng () {
		return softnessDirAng;
	}

	public float getRestitutionDirAng () {
		return restitutionDirAng;
	}

	public float getDampingDirAng () {
		return dampingDirAng;
	}

	public float getSoftnessLimLin () {
		return softnessLimLin;
	}

	public float getRestitutionLimLin () {
		return restitutionLimLin;
	}

	public float getDampingLimLin () {
		return dampingLimLin;
	}

	public float getSoftnessLimAng () {
		return softnessLimAng;
	}

	public float getRestitutionLimAng () {
		return restitutionLimAng;
	}

	public float getDampingLimAng () {
		return dampingLimAng;
	}

	public float getSoftnessOrthoLin () {
		return softnessOrthoLin;
	}

	public float getRestitutionOrthoLin () {
		return restitutionOrthoLin;
	}

	public float getDampingOrthoLin () {
		return dampingOrthoLin;
	}

	public float getSoftnessOrthoAng () {
		return softnessOrthoAng;
	}

	public float getRestitutionOrthoAng () {
		return restitutionOrthoAng;
	}

	public float getDampingOrthoAng () {
		return dampingOrthoAng;
	}

	public void setSoftnessDirLin (float softnessDirLin) {
		this.softnessDirLin = softnessDirLin;
	}

	public void setRestitutionDirLin (float restitutionDirLin) {
		this.restitutionDirLin = restitutionDirLin;
	}

	public void setDampingDirLin (float dampingDirLin) {
		this.dampingDirLin = dampingDirLin;
	}

	public void setSoftnessDirAng (float softnessDirAng) {
		this.softnessDirAng = softnessDirAng;
	}

	public void setRestitutionDirAng (float restitutionDirAng) {
		this.restitutionDirAng = restitutionDirAng;
	}

	public void setDampingDirAng (float dampingDirAng) {
		this.dampingDirAng = dampingDirAng;
	}

	public void setSoftnessLimLin (float softnessLimLin) {
		this.softnessLimLin = softnessLimLin;
	}

	public void setRestitutionLimLin (float restitutionLimLin) {
		this.restitutionLimLin = restitutionLimLin;
	}

	public void setDampingLimLin (float dampingLimLin) {
		this.dampingLimLin = dampingLimLin;
	}

	public void setSoftnessLimAng (float softnessLimAng) {
		this.softnessLimAng = softnessLimAng;
	}

	public void setRestitutionLimAng (float restitutionLimAng) {
		this.restitutionLimAng = restitutionLimAng;
	}

	public void setDampingLimAng (float dampingLimAng) {
		this.dampingLimAng = dampingLimAng;
	}

	public void setSoftnessOrthoLin (float softnessOrthoLin) {
		this.softnessOrthoLin = softnessOrthoLin;
	}

	public void setRestitutionOrthoLin (float restitutionOrthoLin) {
		this.restitutionOrthoLin = restitutionOrthoLin;
	}

	public void setDampingOrthoLin (float dampingOrthoLin) {
		this.dampingOrthoLin = dampingOrthoLin;
	}

	public void setSoftnessOrthoAng (float softnessOrthoAng) {
		this.softnessOrthoAng = softnessOrthoAng;
	}

	public void setRestitutionOrthoAng (float restitutionOrthoAng) {
		this.restitutionOrthoAng = restitutionOrthoAng;
	}

	public void setDampingOrthoAng (float dampingOrthoAng) {
		this.dampingOrthoAng = dampingOrthoAng;
	}

	public void setPoweredLinMotor (boolean onOff) {
		this.poweredLinMotor = onOff;
	}

	public boolean getPoweredLinMotor () {
		return poweredLinMotor;
	}

	public void setTargetLinMotorVelocity (float targetLinMotorVelocity) {
		this.targetLinMotorVelocity = targetLinMotorVelocity;
	}

	public float getTargetLinMotorVelocity () {
		return targetLinMotorVelocity;
	}

	public void setMaxLinMotorForce (float maxLinMotorForce) {
		this.maxLinMotorForce = maxLinMotorForce;
	}

	public float getMaxLinMotorForce () {
		return maxLinMotorForce;
	}

	public void setPoweredAngMotor (boolean onOff) {
		this.poweredAngMotor = onOff;
	}

	public boolean getPoweredAngMotor () {
		return poweredAngMotor;
	}

	public void setTargetAngMotorVelocity (float targetAngMotorVelocity) {
		this.targetAngMotorVelocity = targetAngMotorVelocity;
	}

	public float getTargetAngMotorVelocity () {
		return targetAngMotorVelocity;
	}

	public void setMaxAngMotorForce (float maxAngMotorForce) {
		this.maxAngMotorForce = maxAngMotorForce;
	}

	public float getMaxAngMotorForce () {
		return this.maxAngMotorForce;
	}

	public float getLinearPos () {
		return this.linPos;
	}

	// access for ODE solver

	public boolean getSolveLinLimit () {
		return solveLinLim;
	}

	public float getLinDepth () {
		return depth.x;
	}

	public boolean getSolveAngLimit () {
		return solveAngLim;
	}

	public float getAngDepth () {
		return angDepth;
	}

	// internal

	public void buildJacobianInt (RigidBody rbA, RigidBody rbB, Transform frameInA, Transform frameInB) {
		Stack stack = Stack.enter();
		Transform tmpTrans = stack.allocTransform();
		Transform tmpTrans1 = stack.allocTransform();
		Transform tmpTrans2 = stack.allocTransform();
		Vector3 tmp = stack.allocVector3();
		Vector3 tmp2 = stack.allocVector3();

		// calculate transforms
		calculatedTransformA.mul(rbA.getCenterOfMassTransform(tmpTrans), frameInA);
		calculatedTransformB.mul(rbB.getCenterOfMassTransform(tmpTrans), frameInB);
		realPivotAInW.set(calculatedTransformA.origin);
		realPivotBInW.set(calculatedTransformB.origin);
		MatrixUtil.getColumn(calculatedTransformA.basis, 0, tmp);
		sliderAxis.set(tmp); // along X
		delta.set(realPivotBInW).sub(realPivotAInW);
		projPivotInW.x = realPivotAInW.x + sliderAxis.dot(delta) * sliderAxis.x;
		projPivotInW.y = realPivotAInW.y + sliderAxis.dot(delta) * sliderAxis.y;
		projPivotInW.z = realPivotAInW.z + sliderAxis.dot(delta) * sliderAxis.z;
		relPosA.set(projPivotInW).sub(rbA.getCenterOfMassPosition(tmp));
		relPosB.set(realPivotBInW).sub(rbB.getCenterOfMassPosition(tmp));
		Vector3 normalWorld = stack.allocVector3();

		// linear part
		for (int i = 0; i < 3; i++) {
			MatrixUtil.getColumn(calculatedTransformA.basis, i, normalWorld);

			Matrix3 mat1 = rbA.getCenterOfMassTransform(tmpTrans1).basis;
			mat1.transpose();

			Matrix3 mat2 = rbB.getCenterOfMassTransform(tmpTrans2).basis;
			mat2.transpose();

			jacLin[i].init(mat1, mat2, relPosA, relPosB, normalWorld, rbA.getInvInertiaDiagLocal(tmp), rbA.getInvMass(),
				rbB.getInvInertiaDiagLocal(tmp2), rbB.getInvMass());
			jacLinDiagABInv[i] = 1f / jacLin[i].getDiagonal();
			VectorUtil.setCoord(depth, i, delta.dot(normalWorld));
		}
		testLinLimits();

		// angular part
		for (int i = 0; i < 3; i++) {
			MatrixUtil.getColumn(calculatedTransformA.basis, i, normalWorld);

			Matrix3 mat1 = rbA.getCenterOfMassTransform(tmpTrans1).basis;
			mat1.transpose();

			Matrix3 mat2 = rbB.getCenterOfMassTransform(tmpTrans2).basis;
			mat2.transpose();

			jacAng[i].init(normalWorld, mat1, mat2, rbA.getInvInertiaDiagLocal(tmp), rbB.getInvInertiaDiagLocal(tmp2));
		}
		testAngLimits();

		Vector3 axisA = stack.allocVector3();
		MatrixUtil.getColumn(calculatedTransformA.basis, 0, axisA);
		kAngle = 1f / (rbA.computeAngularImpulseDenominator(axisA) + rbB.computeAngularImpulseDenominator(axisA));
		// clear accumulator for motors
		accumulatedLinMotorImpulse = 0f;
		accumulatedAngMotorImpulse = 0f;
		stack.leave();
	}

	public void solveConstraintInt (RigidBody rbA, RigidBody rbB) {
		Stack stack = Stack.enter();
		Vector3 tmp = stack.allocVector3();

		// linear
		Vector3 velA = rbA.getVelocityInLocalPoint(relPosA, stack.allocVector3());
		Vector3 velB = rbB.getVelocityInLocalPoint(relPosB, stack.allocVector3());
		Vector3 vel = stack.allocVector3();
		vel.set(velA).sub(velB);

		Vector3 impulse_vector = stack.allocVector3();

		for (int i = 0; i < 3; i++) {
			Vector3 normal = jacLin[i].linearJointAxis;
			float rel_vel = normal.dot(vel);
			// calculate positional error
			float depth = VectorUtil.getCoord(this.depth, i);
			// get parameters
			float softness = (i != 0) ? softnessOrthoLin : (solveLinLim ? softnessLimLin : softnessDirLin);
			float restitution = (i != 0) ? restitutionOrthoLin : (solveLinLim ? restitutionLimLin : restitutionDirLin);
			float damping = (i != 0) ? dampingOrthoLin : (solveLinLim ? dampingLimLin : dampingDirLin);
			// calcutate and apply impulse
			float normalImpulse = softness * (restitution * depth / timeStep - damping * rel_vel) * jacLinDiagABInv[i];
			impulse_vector.set(normal).scl(normalImpulse);
			rbA.applyImpulse(impulse_vector, relPosA);
			tmp.set(impulse_vector).scl(-1);
			rbB.applyImpulse(tmp, relPosB);

			if (poweredLinMotor && (i == 0)) {
				// apply linear motor
				if (accumulatedLinMotorImpulse < maxLinMotorForce) {
					float desiredMotorVel = targetLinMotorVelocity;
					float motor_relvel = desiredMotorVel + rel_vel;
					normalImpulse = -motor_relvel * jacLinDiagABInv[i];
					// clamp accumulated impulse
					float new_acc = accumulatedLinMotorImpulse + Math.abs(normalImpulse);
					if (new_acc > maxLinMotorForce) {
						new_acc = maxLinMotorForce;
					}
					float del = new_acc - accumulatedLinMotorImpulse;
					if (normalImpulse < 0f) {
						normalImpulse = -del;
					} else {
						normalImpulse = del;
					}
					accumulatedLinMotorImpulse = new_acc;
					// apply clamped impulse
					impulse_vector.set(normal).scl(normalImpulse);
					rbA.applyImpulse(impulse_vector, relPosA);
					tmp.set(impulse_vector).scl(-1);
					rbB.applyImpulse(tmp, relPosB);
				}
			}
		}

		// angular
		// get axes in world space
		Vector3 axisA = stack.allocVector3();
		MatrixUtil.getColumn(calculatedTransformA.basis, 0, axisA);
		Vector3 axisB = stack.allocVector3();
		MatrixUtil.getColumn(calculatedTransformB.basis, 0, axisB);

		Vector3 angVelA = rbA.getAngularVelocity(stack.allocVector3());
		Vector3 angVelB = rbB.getAngularVelocity(stack.allocVector3());

		Vector3 angVelAroundAxisA = stack.allocVector3();
		angVelAroundAxisA.set(axisA).scl(axisA.dot(angVelA));
		Vector3 angVelAroundAxisB = stack.allocVector3();
		angVelAroundAxisB.set(axisB).scl(axisB.dot(angVelB));

		Vector3 angAorthog = stack.allocVector3();
		angAorthog.set(angVelA).sub(angVelAroundAxisA);
		Vector3 angBorthog = stack.allocVector3();
		angBorthog.set(angVelB).sub(angVelAroundAxisB);
		Vector3 velrelOrthog = stack.allocVector3();
		velrelOrthog.set(angAorthog).sub(angBorthog);

		// solve orthogonal angular velocity correction
		float len = velrelOrthog.len();
		if (len > 0.00001f) {
			Vector3 normal = stack.allocVector3();
			normal.set(velrelOrthog).nor();
			float denom = rbA.computeAngularImpulseDenominator(normal) + rbB.computeAngularImpulseDenominator(normal);
			velrelOrthog.scl((1f / denom) * dampingOrthoAng * softnessOrthoAng);
		}

		// solve angular positional correction
		Vector3 angularError = stack.allocVector3();
		angularError.set(axisA).crs(axisB);
		angularError.scl(1f / timeStep);
		float len2 = angularError.len();
		if (len2 > 0.00001f) {
			Vector3 normal2 = stack.allocVector3();
			normal2.set(angularError).nor();
			float denom2 = rbA.computeAngularImpulseDenominator(normal2) + rbB.computeAngularImpulseDenominator(normal2);
			angularError.scl((1f / denom2) * restitutionOrthoAng * softnessOrthoAng);
		}

		// apply impulse
		tmp.set(velrelOrthog).scl(-1);
		tmp.add(angularError);
		rbA.applyTorqueImpulse(tmp);
		tmp.set(velrelOrthog).sub(angularError);
		rbB.applyTorqueImpulse(tmp);
		float impulseMag;

		// solve angular limits
		if (solveAngLim) {
			tmp.set(angVelB).sub(angVelA);
			impulseMag = tmp.dot(axisA) * dampingLimAng + angDepth * restitutionLimAng / timeStep;
			impulseMag *= kAngle * softnessLimAng;
		} else {
			tmp.set(angVelB).sub(angVelA);
			impulseMag = tmp.dot(axisA) * dampingDirAng + angDepth * restitutionDirAng / timeStep;
			impulseMag *= kAngle * softnessDirAng;
		}
		Vector3 impulse = stack.allocVector3();
		impulse.set(axisA).scl(impulseMag);
		rbA.applyTorqueImpulse(impulse);
		tmp.set(impulse).scl(-1);
		rbB.applyTorqueImpulse(tmp);

		// apply angular motor
		if (poweredAngMotor) {
			if (accumulatedAngMotorImpulse < maxAngMotorForce) {
				Vector3 velrel = stack.allocVector3();
				velrel.set(angVelAroundAxisA).sub(angVelAroundAxisB);
				float projRelVel = velrel.dot(axisA);

				float desiredMotorVel = targetAngMotorVelocity;
				float motor_relvel = desiredMotorVel - projRelVel;

				float angImpulse = kAngle * motor_relvel;
				// clamp accumulated impulse
				float new_acc = accumulatedAngMotorImpulse + Math.abs(angImpulse);
				if (new_acc > maxAngMotorForce) {
					new_acc = maxAngMotorForce;
				}
				float del = new_acc - accumulatedAngMotorImpulse;
				if (angImpulse < 0f) {
					angImpulse = -del;
				} else {
					angImpulse = del;
				}
				accumulatedAngMotorImpulse = new_acc;

				// apply clamped impulse
				Vector3 motorImp = stack.allocVector3();
				motorImp.set(axisA).scl(angImpulse);
				rbA.applyTorqueImpulse(motorImp);
				tmp.set(motorImp).scl(-1);
				rbB.applyTorqueImpulse(tmp);
			}
		}
		stack.leave();
	}

	// shared code used by ODE solver

	public void calculateTransforms () {
		Stack stack = Stack.enter();
		Transform tmpTrans = stack.allocTransform();

		if (useLinearReferenceFrameA) {
			calculatedTransformA.mul(rbA.getCenterOfMassTransform(tmpTrans), frameInA);
			calculatedTransformB.mul(rbB.getCenterOfMassTransform(tmpTrans), frameInB);
		} else {
			calculatedTransformA.mul(rbB.getCenterOfMassTransform(tmpTrans), frameInB);
			calculatedTransformB.mul(rbA.getCenterOfMassTransform(tmpTrans), frameInA);
		}
		realPivotAInW.set(calculatedTransformA.origin);
		realPivotBInW.set(calculatedTransformB.origin);
		MatrixUtil.getColumn(calculatedTransformA.basis, 0, sliderAxis); // along X
		delta.set(realPivotBInW).sub(realPivotAInW);
		projPivotInW.x = realPivotAInW.x + sliderAxis.dot(delta) * sliderAxis.x;
		projPivotInW.y = realPivotAInW.y + sliderAxis.dot(delta) * sliderAxis.y;
		projPivotInW.z = realPivotAInW.z + sliderAxis.dot(delta) * sliderAxis.z;
		Vector3 normalWorld = stack.allocVector3();
		// linear part
		for (int i = 0; i < 3; i++) {
			MatrixUtil.getColumn(calculatedTransformA.basis, i, normalWorld);
			VectorUtil.setCoord(depth, i, delta.dot(normalWorld));
		}
		stack.leave();
	}

	public void testLinLimits () {
		solveLinLim = false;
		linPos = depth.x;
		if (lowerLinLimit <= upperLinLimit) {
			if (depth.x > upperLinLimit) {
				depth.x -= upperLinLimit;
				solveLinLim = true;
			} else if (depth.x < lowerLinLimit) {
				depth.x -= lowerLinLimit;
				solveLinLim = true;
			} else {
				depth.x = 0f;
			}
		} else {
			depth.x = 0f;
		}
	}

	public void testAngLimits () {
		angDepth = 0f;
		solveAngLim = false;
		if (lowerAngLimit <= upperAngLimit) {
			Stack stack = Stack.enter();
			Vector3 axisA0 = stack.allocVector3();
			MatrixUtil.getColumn(calculatedTransformA.basis, 1, axisA0);
			Vector3 axisA1 = stack.allocVector3();
			MatrixUtil.getColumn(calculatedTransformA.basis, 2, axisA1);
			Vector3 axisB0 = stack.allocVector3();
			MatrixUtil.getColumn(calculatedTransformB.basis, 1, axisB0);

			float rot = (float)Math.atan2(axisB0.dot(axisA1), axisB0.dot(axisA0));
			if (rot < lowerAngLimit) {
				angDepth = rot - lowerAngLimit;
				solveAngLim = true;
			} else if (rot > upperAngLimit) {
				angDepth = rot - upperAngLimit;
				solveAngLim = true;
			}
			stack.leave();
		}
	}

	// access for PE Solver

	public Vector3 getAncorInA (Vector3 out) {
		Stack stack = Stack.enter();
		Transform tmpTrans = stack.allocTransform();

		Vector3 ancorInA = out;
		ancorInA.set(sliderAxis).scl((lowerLinLimit + upperLinLimit) * 0.5f).add(realPivotAInW);
		rbA.getCenterOfMassTransform(tmpTrans);
		tmpTrans.inverse();
		tmpTrans.transform(ancorInA);
		stack.leave();
		return ancorInA;
	}

	public Vector3 getAncorInB (Vector3 out) {
		Vector3 ancorInB = out;
		ancorInB.set(frameInB.origin);
		return ancorInB;
	}

}
