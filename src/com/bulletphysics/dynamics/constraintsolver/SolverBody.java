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

package com.bulletphysics.dynamics.constraintsolver;

import com.badlogic.gdx.math.Vector3;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.TransformUtil;
import com.bulletphysics.util.Stack;

/** SolverBody is an internal data structure for the constraint solver. Only necessary data is packed to increase cache
 * coherence/performance.
 * 
 * @author jezek2 */
public class SolverBody {

	// protected final BulletStack stack = BulletStack.get();

	public final Vector3 angularVelocity = new Vector3();
	public final Vector3 angularFactor = new Vector3();
	public final Vector3 linearFactor = new Vector3();
	public float invMass;
	public float friction;
	public RigidBody originalBody;
	public final Vector3 linearVelocity = new Vector3();
	public final Vector3 centerOfMassPosition = new Vector3();

	public final Vector3 pushVelocity = new Vector3();
	public final Vector3 turnVelocity = new Vector3();

	public void getVelocityInLocalPoint (Vector3 rel_pos, Vector3 velocity) {
		Stack stack = Stack.enter();
		Vector3 tmp = stack.allocVector3();
		tmp.set(angularVelocity).crs(rel_pos);
		velocity.set(linearVelocity).add(tmp);
		stack.leave();
	}

	/** Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position. */
	public void internalApplyImpulse (Vector3 linearComponent, Vector3 angularComponent, float impulseMagnitude) {
		if (invMass != 0f) {
			linearVelocity.x += impulseMagnitude * linearFactor.x * linearComponent.x;
			linearVelocity.y += impulseMagnitude * linearFactor.y * linearComponent.y;
			linearVelocity.z += impulseMagnitude * linearFactor.z * linearComponent.z;
			// linearVelocity.scaleAdd(impulseMagnitude, linearComponent, linearVelocity);
			angularVelocity.x += impulseMagnitude * angularFactor.x * angularComponent.x;
			angularVelocity.y += impulseMagnitude * angularFactor.y * angularComponent.y;
			angularVelocity.z += impulseMagnitude * angularFactor.z * angularComponent.z;
			// angularVelocity.scaleAdd(impulseMagnitude * angularFactor, angularComponent, angularVelocity);
		}
	}

	public void internalApplyPushImpulse (Vector3 linearComponent, Vector3 angularComponent, float impulseMagnitude) {
		if (invMass != 0f) {
			pushVelocity.x += impulseMagnitude * linearFactor.x * linearComponent.x;
			pushVelocity.y += impulseMagnitude * linearFactor.y * linearComponent.y;
			pushVelocity.z += impulseMagnitude * linearFactor.z * linearComponent.z;
			// pushVelocity.scaleAdd(impulseMagnitude, linearComponent, pushVelocity);
			turnVelocity.x += impulseMagnitude * angularFactor.x * angularComponent.x;
			turnVelocity.y += impulseMagnitude * angularFactor.y * angularComponent.y;
			turnVelocity.z += impulseMagnitude * angularFactor.z * angularComponent.z;
			// turnVelocity.scaleAdd(impulseMagnitude * angularFactor, angularComponent, turnVelocity);
		}
	}

	public void writebackVelocity () {
		if (invMass != 0f) {
			originalBody.setLinearVelocity(linearVelocity);
			originalBody.setAngularVelocity(angularVelocity);
			// m_originalBody->setCompanionId(-1);
		}
	}

	public void writebackVelocity (float timeStep) {
		if (invMass != 0f) {
			originalBody.setLinearVelocity(linearVelocity);
			originalBody.setAngularVelocity(angularVelocity);
			Stack stack = Stack.enter();

			// correct the position/orientation based on push/turn recovery
			Transform newTransform = stack.allocTransform();
			Transform curTrans = originalBody.getWorldTransform(stack.allocTransform());
			TransformUtil.integrateTransform(curTrans, pushVelocity, turnVelocity, timeStep, newTransform);
			originalBody.setWorldTransform(newTransform);

			// m_originalBody->setCompanionId(-1);
			stack.leave();
		}
	}

	public void readVelocity () {
		if (invMass != 0f) {
			originalBody.getLinearVelocity(linearVelocity);
			originalBody.getAngularVelocity(angularVelocity);
		}
	}

}
