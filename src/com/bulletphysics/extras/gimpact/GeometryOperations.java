/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * This source file is part of GIMPACT Library.
 *
 * For the latest info, see http://gimpact.sourceforge.net/
 *
 * Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
 * email: projectileman@yahoo.com
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

package com.bulletphysics.extras.gimpact;

import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import com.bulletphysics.BulletGlobals;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.Stack;

/** @author jezek2 */
class GeometryOperations {

	public static final float PLANEDIREPSILON = 0.0000001f;
	public static final float PARALELENORMALS = 0.000001f;

	public static final float CLAMP (float number, float minval, float maxval) {
		return (number < minval ? minval : (number > maxval ? maxval : number));
	}

	/** Calc a plane from a triangle edge an a normal. */
	public static void edge_plane (Vector3 e1, Vector3 e2, Vector3 normal, Quaternion plane) {
		Stack stack = Stack.enter();
		Vector3 planenormal = stack.allocVector3();
		planenormal.set(e2).sub(e1);
		planenormal.set(planenormal).crs(normal);
		planenormal.nor();

		plane.set(planenormal, e2.dot(planenormal));
		stack.leave();
	}

	/** Finds the closest point(cp) to (v) on a segment (e1,e2). */
	public static void closest_point_on_segment (Vector3 cp, Vector3 v, Vector3 e1, Vector3 e2) {
		Stack stack = Stack.enter();
		Vector3 n = stack.allocVector3();
		n.set(e2).sub(e1);
		cp.set(v).sub(e1);
		float _scalar = cp.dot(n) / n.dot(n);
		if (_scalar < 0.0f) {
			cp = e1;
		} else if (_scalar > 1.0f) {
			cp = e2;
		} else {
			n.scl(_scalar);
			cp.set(e1).add(n);
		}
		stack.leave();
	}

	/** Line plane collision.
	 * 
	 * @return -0 if the ray never intersects, -1 if the ray collides in front, -2 if the ray collides in back */
	public static int line_plane_collision (Quaternion plane, Vector3 vDir, Vector3 vPoint, Vector3 pout, float[] tparam, float tmin,
		float tmax) {
		float _dotdir = VectorUtil.dot3(vDir, plane);

		if (Math.abs(_dotdir) < PLANEDIREPSILON) {
			tparam[0] = tmax;
			return 0;
		}

		float _dis = ClipPolygon.distance_point_plane(plane, vPoint);
		int returnvalue = _dis < 0.0f ? 2 : 1;
		tparam[0] = -_dis / _dotdir;

		if (tparam[0] < tmin) {
			returnvalue = 0;
			tparam[0] = tmin;
		} else if (tparam[0] > tmax) {
			returnvalue = 0;
			tparam[0] = tmax;
		}
		pout.x = vPoint.x + tparam[0] * vDir.x;
		pout.y = vPoint.y + tparam[0] * vDir.y;
		pout.z = vPoint.z + tparam[0] * vDir.z;
		return returnvalue;
	}

	/** Find closest points on segments. */
	public static void segment_collision (Vector3 vA1, Vector3 vA2, Vector3 vB1, Vector3 vB2, Vector3 vPointA, Vector3 vPointB) {
		Stack stack = Stack.enter();
		Vector3 AD = stack.allocVector3();
		AD.set(vA2).sub(vA1);

		Vector3 BD = stack.allocVector3();
		BD.set(vB2).sub(vB1);

		Vector3 N = stack.allocVector3();
		N.set(AD).crs(BD);
		float[] tp = new float[] {N.len2()};

		Quaternion _M = stack.allocQuaternion();// plane

		if (tp[0] < BulletGlobals.SIMD_EPSILON)// ARE PARALELE
		{
			// project B over A
			boolean invert_b_order = false;
			_M.x = vB1.dot(AD);
			_M.y = vB2.dot(AD);

			if (_M.x > _M.y) {
				invert_b_order = true;
				// BT_SWAP_NUMBERS(_M[0],_M[1]);
				_M.x = _M.x + _M.y;
				_M.y = _M.x - _M.y;
				_M.x = _M.x - _M.y;
			}
			_M.z = vA1.dot(AD);
			_M.w = vA2.dot(AD);
			// mid points
			N.x = (_M.x + _M.y) * 0.5f;
			N.y = (_M.z + _M.w) * 0.5f;

			if (N.x < N.y) {
				if (_M.y < _M.z) {
					vPointB = invert_b_order ? vB1 : vB2;
					vPointA = vA1;
				} else if (_M.y < _M.w) {
					vPointB = invert_b_order ? vB1 : vB2;
					closest_point_on_segment(vPointA, vPointB, vA1, vA2);
				} else {
					vPointA = vA2;
					closest_point_on_segment(vPointB, vPointA, vB1, vB2);
				}
			} else {
				if (_M.w < _M.x) {
					vPointB = invert_b_order ? vB2 : vB1;
					vPointA = vA2;
				} else if (_M.w < _M.y) {
					vPointA = vA2;
					closest_point_on_segment(vPointB, vPointA, vB1, vB2);
				} else {
					vPointB = invert_b_order ? vB1 : vB2;
					closest_point_on_segment(vPointA, vPointB, vA1, vA2);
				}
			}
			stack.leave();
			return;
		}

		N.set(N).crs(BD);
		_M.set(N.x, N.y, N.z, vB1.dot(N));

		// get point A as the plane collision point
		line_plane_collision(_M, AD, vA1, vPointA, tp, 0f, 1f);

		/* Closest point on segment */
		vPointB.set(vPointA).sub(vB1);
		tp[0] = vPointB.dot(BD);
		tp[0] /= BD.dot(BD);
		tp[0] = CLAMP(tp[0], 0.0f, 1.0f);

		BD.scl(tp[0]);
		vPointB.set(vB1).add(BD);
		stack.leave();
	}

}
