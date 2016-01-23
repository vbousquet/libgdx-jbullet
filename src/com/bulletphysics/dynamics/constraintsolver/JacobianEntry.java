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

import com.badlogic.gdx.math.Matrix3;
import com.badlogic.gdx.math.Vector3;
import com.bulletphysics.BulletGlobals;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.Stack;

//notes:
// Another memory optimization would be to store m_1MinvJt in the remaining 3 w components
// which makes the btJacobianEntry memory layout 16 bytes
// if you only are interested in angular part, just feed massInvA and massInvB zero

/** Jacobian entry is an abstraction that allows to describe constraints. It can be used in combination with a constraint solver.
 * Can be used to relate the effect of an impulse to the constraint error.
 * 
 * @author jezek2 */
public class JacobianEntry {

	// protected final BulletStack stack = BulletStack.get();

	public final Vector3 linearJointAxis = new Vector3();
	public final Vector3 aJ = new Vector3();
	public final Vector3 bJ = new Vector3();
	public final Vector3 m_0MinvJt = new Vector3();
	public final Vector3 m_1MinvJt = new Vector3();
	// Optimization: can be stored in the w/last component of one of the vectors
	public float Adiag;

	public JacobianEntry () {
	}

	/** Constraint between two different rigidbodies. */
	public void init (Matrix3 world2A, Matrix3 world2B, Vector3 rel_pos1, Vector3 rel_pos2, Vector3 jointAxis, Vector3 inertiaInvA,
		float massInvA, Vector3 inertiaInvB, float massInvB) {
		linearJointAxis.set(jointAxis);

		aJ.set(rel_pos1).crs(linearJointAxis);
		aJ.mul(world2A);

		bJ.set(linearJointAxis);
		bJ.scl(-1);
		bJ.set(rel_pos2).crs(bJ);
		bJ.mul(world2B);

		VectorUtil.mul(m_0MinvJt, inertiaInvA, aJ);
		VectorUtil.mul(m_1MinvJt, inertiaInvB, bJ);
		Adiag = massInvA + m_0MinvJt.dot(aJ) + massInvB + m_1MinvJt.dot(bJ);

		assert (Adiag > 0f);
	}

	/** Angular constraint between two different rigidbodies. */
	public void init (Vector3 jointAxis, Matrix3 world2A, Matrix3 world2B, Vector3 inertiaInvA, Vector3 inertiaInvB) {
		linearJointAxis.set(0f, 0f, 0f);

		aJ.set(jointAxis);
		aJ.mul(world2A);

		bJ.set(jointAxis);
		bJ.scl(-1);
		bJ.mul(world2B);

		VectorUtil.mul(m_0MinvJt, inertiaInvA, aJ);
		VectorUtil.mul(m_1MinvJt, inertiaInvB, bJ);
		Adiag = m_0MinvJt.dot(aJ) + m_1MinvJt.dot(bJ);

		assert (Adiag > 0f);
	}

	/** Angular constraint between two different rigidbodies. */
	public void init (Vector3 axisInA, Vector3 axisInB, Vector3 inertiaInvA, Vector3 inertiaInvB) {
		linearJointAxis.set(0f, 0f, 0f);
		aJ.set(axisInA);

		bJ.set(axisInB);
		bJ.scl(-1);

		VectorUtil.mul(m_0MinvJt, inertiaInvA, aJ);
		VectorUtil.mul(m_1MinvJt, inertiaInvB, bJ);
		Adiag = m_0MinvJt.dot(aJ) + m_1MinvJt.dot(bJ);

		assert (Adiag > 0f);
	}

	/** Constraint on one rigidbody. */
	public void init (Matrix3 world2A, Vector3 rel_pos1, Vector3 rel_pos2, Vector3 jointAxis, Vector3 inertiaInvA,
		float massInvA) {
		linearJointAxis.set(jointAxis);

		aJ.set(rel_pos1).crs(jointAxis);
		aJ.mul(world2A);

		bJ.set(jointAxis);
		bJ.scl(-1);
		bJ.set(rel_pos2).crs(bJ);
		bJ.mul(world2A);

		VectorUtil.mul(m_0MinvJt, inertiaInvA, aJ);
		m_1MinvJt.set(0f, 0f, 0f);
		Adiag = massInvA + m_0MinvJt.dot(aJ);

		assert (Adiag > 0f);
	}

	public float getDiagonal () {
		return Adiag;
	}

	/** For two constraints on the same rigidbody (for example vehicle friction). */
	public float getNonDiagonal (JacobianEntry jacB, float massInvA) {
		JacobianEntry jacA = this;
		float lin = massInvA * jacA.linearJointAxis.dot(jacB.linearJointAxis);
		float ang = jacA.m_0MinvJt.dot(jacB.aJ);
		return lin + ang;
	}

	/** For two constraints on sharing two same rigidbodies (for example two contact points between two rigidbodies). */
	public float getNonDiagonal (JacobianEntry jacB, float massInvA, float massInvB) {
		JacobianEntry jacA = this;
		Stack stack = Stack.enter();

		Vector3 lin = stack.allocVector3();
		VectorUtil.mul(lin, jacA.linearJointAxis, jacB.linearJointAxis);

		Vector3 ang0 = stack.allocVector3();
		VectorUtil.mul(ang0, jacA.m_0MinvJt, jacB.aJ);

		Vector3 ang1 = stack.allocVector3();
		VectorUtil.mul(ang1, jacA.m_1MinvJt, jacB.bJ);

		Vector3 lin0 = stack.allocVector3();
		lin0.set(lin).scl(massInvA);

		Vector3 lin1 = stack.allocVector3();
		lin1.set(lin).scl(massInvB);

		Vector3 sum = stack.allocVector3();
		VectorUtil.add(sum, ang0, ang1, lin0, lin1);

		float result = sum.x + sum.y + sum.z;
		stack.leave();
		return result;
	}

	public float getRelativeVelocity (Vector3 linvelA, Vector3 angvelA, Vector3 linvelB, Vector3 angvelB) {
		Stack stack = Stack.enter();
		Vector3 linrel = stack.allocVector3();
		linrel.set(linvelA).sub(linvelB);

		Vector3 angvela = stack.allocVector3();
		VectorUtil.mul(angvela, angvelA, aJ);

		Vector3 angvelb = stack.allocVector3();
		VectorUtil.mul(angvelb, angvelB, bJ);

		VectorUtil.mul(linrel, linrel, linearJointAxis);

		angvela.add(angvelb);
		angvela.add(linrel);

		float rel_vel2 = angvela.x + angvela.y + angvela.z;
		stack.leave();
		return rel_vel2 + BulletGlobals.FLT_EPSILON;
	}

}
