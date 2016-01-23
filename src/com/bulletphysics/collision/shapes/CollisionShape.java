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

package com.bulletphysics.collision.shapes;

import com.badlogic.gdx.math.Vector3;
import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.Stack;

/** CollisionShape class provides an interface for collision shapes that can be shared among {@link CollisionObject}s.
 * 
 * @author jezek2 */
public abstract class CollisionShape {

	// protected final BulletStack stack = BulletStack.get();

	protected Object userPointer;

	// /getAabb returns the axis aligned bounding box in the coordinate frame of the given transform t.
	public abstract void getAabb (Transform t, Vector3 aabbMin, Vector3 aabbMax);

	public float getBoundingSphere (Vector3 center) {
		Stack stack = Stack.enter();
		Vector3 tmp = stack.allocVector3();

		Transform tr = stack.allocTransform();
		tr.setIdentity();
		Vector3 aabbMin = stack.allocVector3(), aabbMax = stack.allocVector3();

		getAabb(tr, aabbMin, aabbMax);

		tmp.set(aabbMax).sub(aabbMin);
		float radius = tmp.len() * 0.5f;

		tmp.set(aabbMin).add(aabbMax);
		center.set(tmp).scl(0.5f);
		stack.leave();
		return radius;
	}

	// /getAngularMotionDisc returns the maximus radius needed for Conservative Advancement to handle time-of-impact with
// rotations.
	public float getAngularMotionDisc () {
		Stack stack = Stack.enter();
		Vector3 center = stack.allocVector3();
		float disc = getBoundingSphere(center);
		disc += center.len();
		stack.leave();
		return disc;
	}

	// /calculateTemporalAabb calculates the enclosing aabb for the moving object over interval [0..timeStep)
	// /result is conservative
	public void calculateTemporalAabb (Transform curTrans, Vector3 linvel, Vector3 angvel, float timeStep,
		Vector3 temporalAabbMin, Vector3 temporalAabbMax) {
		// start with static aabb
		getAabb(curTrans, temporalAabbMin, temporalAabbMax);

		Stack stack = Stack.enter();

		float temporalAabbMaxx = temporalAabbMax.x;
		float temporalAabbMaxy = temporalAabbMax.y;
		float temporalAabbMaxz = temporalAabbMax.z;
		float temporalAabbMinx = temporalAabbMin.x;
		float temporalAabbMiny = temporalAabbMin.y;
		float temporalAabbMinz = temporalAabbMin.z;

		// add linear motion
		Vector3 linMotion = stack.alloc(linvel);
		linMotion.scl(timeStep);

		// todo: simd would have a vector max/min operation, instead of per-element access
		if (linMotion.x > 0f) {
			temporalAabbMaxx += linMotion.x;
		} else {
			temporalAabbMinx += linMotion.x;
		}
		if (linMotion.y > 0f) {
			temporalAabbMaxy += linMotion.y;
		} else {
			temporalAabbMiny += linMotion.y;
		}
		if (linMotion.z > 0f) {
			temporalAabbMaxz += linMotion.z;
		} else {
			temporalAabbMinz += linMotion.z;
		}

		// add conservative angular motion
		float angularMotion = angvel.len() * getAngularMotionDisc() * timeStep;
		Vector3 angularMotion3d = stack.allocVector3();
		angularMotion3d.set(angularMotion, angularMotion, angularMotion);
		temporalAabbMin.set(temporalAabbMinx, temporalAabbMiny, temporalAabbMinz);
		temporalAabbMax.set(temporalAabbMaxx, temporalAabbMaxy, temporalAabbMaxz);

		temporalAabbMin.sub(angularMotion3d);
		temporalAabbMax.add(angularMotion3d);

		stack.leave();
	}

// #ifndef __SPU__
	public boolean isPolyhedral () {
		return getShapeType().isPolyhedral();
	}

	public boolean isConvex () {
		return getShapeType().isConvex();
	}

	public boolean isConcave () {
		return getShapeType().isConcave();
	}

	public boolean isCompound () {
		return getShapeType().isCompound();
	}

	// /isInfinite is used to catch simulation error (aabb check)
	public boolean isInfinite () {
		return getShapeType().isInfinite();
	}

	public abstract BroadphaseNativeType getShapeType ();

	public abstract void setLocalScaling (Vector3 scaling);

	// TODO: returns const
	public abstract Vector3 getLocalScaling (Vector3 out);

	public abstract void calculateLocalInertia (float mass, Vector3 inertia);

// debugging support
	public abstract String getName ();

// #endif //__SPU__
	public abstract void setMargin (float margin);

	public abstract float getMargin ();

	// optional user data pointer
	public void setUserPointer (Object userPtr) {
		userPointer = userPtr;
	}

	public Object getUserPointer () {
		return userPointer;
	}

}
