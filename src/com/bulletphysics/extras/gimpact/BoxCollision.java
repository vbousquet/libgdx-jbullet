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

import com.badlogic.gdx.math.Matrix3;
import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import com.bulletphysics.BulletGlobals;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.Stack;

/** @author jezek2 */
public class BoxCollision {

	public static final float BOX_PLANE_EPSILON = 0.000001f;

	public static boolean BT_GREATER (float x, float y) {
		return Math.abs(x) > y;
	}

	public static float BT_MAX3 (float a, float b, float c) {
		return Math.max(a, Math.max(b, c));
	}

	public static float BT_MIN3 (float a, float b, float c) {
		return Math.min(a, Math.min(b, c));
	}

	public static boolean TEST_CROSS_EDGE_BOX_MCR (Vector3 edge, Vector3 absolute_edge, Vector3 pointa, Vector3 pointb,
		Vector3 _extend, int i_dir_0, int i_dir_1, int i_comp_0, int i_comp_1) {
		float dir0 = -VectorUtil.getCoord(edge, i_dir_0);
		float dir1 = VectorUtil.getCoord(edge, i_dir_1);
		float pmin = VectorUtil.getCoord(pointa, i_comp_0) * dir0 + VectorUtil.getCoord(pointa, i_comp_1) * dir1;
		float pmax = VectorUtil.getCoord(pointb, i_comp_0) * dir0 + VectorUtil.getCoord(pointb, i_comp_1) * dir1;
		if (pmin > pmax) {
			// BT_SWAP_NUMBERS(pmin,pmax);
			pmin = pmin + pmax;
			pmax = pmin - pmax;
			pmin = pmin - pmax;
		}
		float abs_dir0 = VectorUtil.getCoord(absolute_edge, i_dir_0);
		float abs_dir1 = VectorUtil.getCoord(absolute_edge, i_dir_1);
		float rad = VectorUtil.getCoord(_extend, i_comp_0) * abs_dir0 + VectorUtil.getCoord(_extend, i_comp_1) * abs_dir1;
		if (pmin > rad || -rad > pmax) {
			return false;
		}
		return true;
	}

	public static boolean TEST_CROSS_EDGE_BOX_X_AXIS_MCR (Vector3 edge, Vector3 absolute_edge, Vector3 pointa, Vector3 pointb,
		Vector3 _extend) {
		return TEST_CROSS_EDGE_BOX_MCR(edge, absolute_edge, pointa, pointb, _extend, 2, 1, 1, 2);
	}

	public static boolean TEST_CROSS_EDGE_BOX_Y_AXIS_MCR (Vector3 edge, Vector3 absolute_edge, Vector3 pointa, Vector3 pointb,
		Vector3 _extend) {
		return TEST_CROSS_EDGE_BOX_MCR(edge, absolute_edge, pointa, pointb, _extend, 0, 2, 2, 0);
	}

	public static boolean TEST_CROSS_EDGE_BOX_Z_AXIS_MCR (Vector3 edge, Vector3 absolute_edge, Vector3 pointa, Vector3 pointb,
		Vector3 _extend) {
		return TEST_CROSS_EDGE_BOX_MCR(edge, absolute_edge, pointa, pointb, _extend, 1, 0, 0, 1);
	}

	/** Returns the dot product between a vec3f and the col of a matrix. */
	public static float bt_mat3_dot_col (Matrix3 mat, Vector3 vec3, int colindex) {
		return vec3.x * MatrixUtil.getElement(mat, 0, colindex) + vec3.y * MatrixUtil.getElement(mat, 1, colindex)
			+ vec3.z * MatrixUtil.getElement(mat, 2, colindex);
	}

	/** Compairison of transformation objects. */
	public static boolean compareTransformsEqual (Transform t1, Transform t2) {
		return t1.equals(t2);
	}

	// //////////////////////////////////////////////////////////////////////////

	public static class BoxBoxTransformCache {
		public final Vector3 T1to0 = new Vector3(); // Transforms translation of model1 to model 0
		public final Matrix3 R1to0 = new Matrix3(); // Transforms Rotation of model1 to model 0, equal to R0' * R1
		public final Matrix3 AR = new Matrix3(); // Absolute value of m_R1to0

		public void set (BoxBoxTransformCache cache) {
			throw new UnsupportedOperationException();
		}

		public void calc_absolute_matrix () {
			// static const btVector3 vepsi(1e-6f,1e-6f,1e-6f);
			// m_AR[0] = vepsi + m_R1to0[0].absolute();
			// m_AR[1] = vepsi + m_R1to0[1].absolute();
			// m_AR[2] = vepsi + m_R1to0[2].absolute();

			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					MatrixUtil.setElement(AR, i, j, 1e-6f + Math.abs(MatrixUtil.getElement(R1to0, i, j)));
				}
			}
		}

		/** Calc the transformation relative 1 to 0. Inverts matrics by transposing. */
		public void calc_from_homogenic (Transform trans0, Transform trans1) {
			Stack stack = Stack.enter();
			Transform temp_trans = stack.allocTransform();
			temp_trans.inverse(trans0);
			temp_trans.mul(trans1);

			T1to0.set(temp_trans.origin);
			R1to0.set(temp_trans.basis);

			calc_absolute_matrix();
			stack.leave();
		}

		/** Calcs the full invertion of the matrices. Useful for scaling matrices. */
		public void calc_from_full_invert (Transform trans0, Transform trans1) {
			R1to0.set(trans0.basis).inv();
			T1to0.set(trans0.origin).scl(-1);
			T1to0.mul(R1to0);
			Stack stack = Stack.enter();
			Vector3 tmp = stack.allocVector3();
			tmp.set(trans1.origin);
			tmp.mul(R1to0);
			T1to0.add(tmp);

			R1to0.mul(trans1.basis);

			calc_absolute_matrix();
			stack.leave();
		}

		public Vector3 transform (Vector3 point, Vector3 out) {
			Stack stack = Stack.enter();
			if (point == out) {
				point = stack.alloc(point);
			}

			Vector3 tmp = stack.allocVector3();
			MatrixUtil.getRow(R1to0, 0, tmp);
			out.x = tmp.dot(point) + T1to0.x;
			MatrixUtil.getRow(R1to0, 1, tmp);
			out.y = tmp.dot(point) + T1to0.y;
			MatrixUtil.getRow(R1to0, 2, tmp);
			out.z = tmp.dot(point) + T1to0.z;
			stack.leave();
			return out;
		}
	}

	// //////////////////////////////////////////////////////////////////////////

	public static class AABB {
		public final Vector3 min = new Vector3();
		public final Vector3 max = new Vector3();

		public AABB () {
		}

		public AABB (Vector3 V1, Vector3 V2, Vector3 V3) {
			calc_from_triangle(V1, V2, V3);
		}

		public AABB (Vector3 V1, Vector3 V2, Vector3 V3, float margin) {
			calc_from_triangle_margin(V1, V2, V3, margin);
		}

		public AABB (AABB other) {
			set(other);
		}

		public AABB (AABB other, float margin) {
			this(other);
			min.x -= margin;
			min.y -= margin;
			min.z -= margin;
			max.x += margin;
			max.y += margin;
			max.z += margin;
		}

		public void init (Vector3 V1, Vector3 V2, Vector3 V3, float margin) {
			calc_from_triangle_margin(V1, V2, V3, margin);
		}

		public void set (AABB other) {
			min.set(other.min);
			max.set(other.max);
		}

		public void invalidate () {
			min.set(BulletGlobals.SIMD_INFINITY, BulletGlobals.SIMD_INFINITY, BulletGlobals.SIMD_INFINITY);
			max.set(-BulletGlobals.SIMD_INFINITY, -BulletGlobals.SIMD_INFINITY, -BulletGlobals.SIMD_INFINITY);
		}

		public void increment_margin (float margin) {
			min.x -= margin;
			min.y -= margin;
			min.z -= margin;
			max.x += margin;
			max.y += margin;
			max.z += margin;
		}

		public void copy_with_margin (AABB other, float margin) {
			min.x = other.min.x - margin;
			min.y = other.min.y - margin;
			min.z = other.min.z - margin;

			max.x = other.max.x + margin;
			max.y = other.max.y + margin;
			max.z = other.max.z + margin;
		}

		public void calc_from_triangle (Vector3 V1, Vector3 V2, Vector3 V3) {
			min.x = BT_MIN3(V1.x, V2.x, V3.x);
			min.y = BT_MIN3(V1.y, V2.y, V3.y);
			min.z = BT_MIN3(V1.z, V2.z, V3.z);

			max.x = BT_MAX3(V1.x, V2.x, V3.x);
			max.y = BT_MAX3(V1.y, V2.y, V3.y);
			max.z = BT_MAX3(V1.z, V2.z, V3.z);
		}

		public void calc_from_triangle_margin (Vector3 V1, Vector3 V2, Vector3 V3, float margin) {
			calc_from_triangle(V1, V2, V3);
			min.x -= margin;
			min.y -= margin;
			min.z -= margin;
			max.x += margin;
			max.y += margin;
			max.z += margin;
		}

		/** Apply a transform to an AABB. */
		public void appy_transform (Transform trans) {
			Stack stack = Stack.enter();
			Vector3 tmp = stack.allocVector3();

			Vector3 center = stack.allocVector3();
			center.set(max).add(min);
			center.scl(0.5f);

			Vector3 extends_ = stack.allocVector3();
			extends_.set(max).sub(center);

			// Compute new center
			trans.transform(center);

			Vector3 textends = stack.allocVector3();

			MatrixUtil.getRow(trans.basis, 0, tmp);
			VectorUtil.absolute(tmp);
			textends.x = extends_.dot(tmp);

			MatrixUtil.getRow(trans.basis, 1, tmp);
			VectorUtil.absolute(tmp);
			textends.y = extends_.dot(tmp);

			MatrixUtil.getRow(trans.basis, 2, tmp);
			VectorUtil.absolute(tmp);
			textends.z = extends_.dot(tmp);

			min.set(center).sub(textends);
			max.set(center).add(textends);
			stack.leave();
		}

		/** Apply a transform to an AABB. */
		public void appy_transform_trans_cache (BoxBoxTransformCache trans) {
			Stack stack = Stack.enter();
			Vector3 tmp = stack.allocVector3();

			Vector3 center = stack.allocVector3();
			center.set(max).add(min);
			center.scl(0.5f);

			Vector3 extends_ = stack.allocVector3();
			extends_.set(max).sub(center);

			// Compute new center
			trans.transform(center, center);

			Vector3 textends = stack.allocVector3();

			MatrixUtil.getRow(trans.R1to0, 0, tmp);
			VectorUtil.absolute(tmp);
			textends.x = extends_.dot(tmp);

			MatrixUtil.getRow(trans.R1to0, 1, tmp);
			VectorUtil.absolute(tmp);
			textends.y = extends_.dot(tmp);

			MatrixUtil.getRow(trans.R1to0, 2, tmp);
			VectorUtil.absolute(tmp);
			textends.z = extends_.dot(tmp);

			min.set(center).sub(textends);
			max.set(center).add(textends);
			stack.leave();
		}

		/** Merges a Box. */
		public void merge (AABB box) {
			min.x = Math.min(min.x, box.min.x);
			min.y = Math.min(min.y, box.min.y);
			min.z = Math.min(min.z, box.min.z);

			max.x = Math.max(max.x, box.max.x);
			max.y = Math.max(max.y, box.max.y);
			max.z = Math.max(max.z, box.max.z);
		}

		/** Merges a point. */
		public void merge_point (Vector3 point) {
			min.x = Math.min(min.x, point.x);
			min.y = Math.min(min.y, point.y);
			min.z = Math.min(min.z, point.z);

			max.x = Math.max(max.x, point.x);
			max.y = Math.max(max.y, point.y);
			max.z = Math.max(max.z, point.z);
		}

		/** Gets the extend and center. */
		public void get_center_extend (Vector3 center, Vector3 extend) {
			center.set(max).add(min);
			center.scl(0.5f);

			extend.set(max).sub(center);
		}

		/** Finds the intersecting box between this box and the other. */
		public void find_intersection (AABB other, AABB intersection) {
			intersection.min.x = Math.max(other.min.x, min.x);
			intersection.min.y = Math.max(other.min.y, min.y);
			intersection.min.z = Math.max(other.min.z, min.z);

			intersection.max.x = Math.min(other.max.x, max.x);
			intersection.max.y = Math.min(other.max.y, max.y);
			intersection.max.z = Math.min(other.max.z, max.z);
		}

		public boolean has_collision (AABB other) {
			if (min.x > other.max.x || max.x < other.min.x || min.y > other.max.y || max.y < other.min.y || min.z > other.max.z
				|| max.z < other.min.z) {
				return false;
			}
			return true;
		}

		/** Finds the Ray intersection parameter.
		 * 
		 * @param aabb aligned box
		 * @param vorigin a vec3f with the origin of the ray
		 * @param vdir a vec3f with the direction of the ray */
		public boolean collide_ray (Vector3 vorigin, Vector3 vdir) {
			Stack stack = Stack.enter();
			Vector3 extents = stack.allocVector3(), center = stack.allocVector3();
			get_center_extend(center, extents);

			float Dx = vorigin.x - center.x;
			if (BT_GREATER(Dx, extents.x) && Dx * vdir.x >= 0.0f) {
				stack.leave();
				return false;
			}

			float Dy = vorigin.y - center.y;
			if (BT_GREATER(Dy, extents.y) && Dy * vdir.y >= 0.0f) {
				stack.leave();
				return false;
			}

			float Dz = vorigin.z - center.z;
			if (BT_GREATER(Dz, extents.z) && Dz * vdir.z >= 0.0f) {
				stack.leave();
				return false;
			}

			float f = vdir.y * Dz - vdir.z * Dy;
			if (Math.abs(f) > extents.y * Math.abs(vdir.z) + extents.z * Math.abs(vdir.y)) {
				stack.leave();
				return false;
			}

			f = vdir.z * Dx - vdir.x * Dz;
			if (Math.abs(f) > extents.x * Math.abs(vdir.z) + extents.z * Math.abs(vdir.x)) {
				stack.leave();
				return false;
			}

			f = vdir.x * Dy - vdir.y * Dx;
			if (Math.abs(f) > extents.x * Math.abs(vdir.y) + extents.y * Math.abs(vdir.x)) {
				stack.leave();
				return false;
			}
			stack.leave();
			return true;
		}

		public void projection_interval (Vector3 direction, float[] vmin, float[] vmax) {
			Stack stack = Stack.enter();
			Vector3 tmp = stack.allocVector3();

			Vector3 center = stack.allocVector3();
			Vector3 extend = stack.allocVector3();
			get_center_extend(center, extend);

			float _fOrigin = direction.dot(center);
			VectorUtil.absolute(tmp.set(direction));
			float _fMaximumExtent = extend.dot(tmp);
			vmin[0] = _fOrigin - _fMaximumExtent;
			vmax[0] = _fOrigin + _fMaximumExtent;
			stack.leave();
		}

		public PlaneIntersectionType plane_classify (Quaternion plane) {
			Stack stack = Stack.enter();
			Vector3 tmp = stack.allocVector3();

			float[] _fmin = new float[1], _fmax = new float[1];
			tmp.set(plane.x, plane.y, plane.z);
			projection_interval(tmp, _fmin, _fmax);

			if (plane.w > _fmax[0] + BOX_PLANE_EPSILON) {
				stack.leave();
				return PlaneIntersectionType.BACK_PLANE; // 0
			}

			if (plane.w + BOX_PLANE_EPSILON >= _fmin[0]) {
				stack.leave();
				return PlaneIntersectionType.COLLIDE_PLANE; // 1
			}

			stack.leave();
			return PlaneIntersectionType.FRONT_PLANE; // 2
		}

		public boolean overlapping_trans_conservative (AABB box, Transform trans1_to_0) {
			Stack stack = Stack.enter();
			AABB tbox = stack.alloc(box);
			tbox.appy_transform(trans1_to_0);
			boolean result = has_collision(tbox);
			stack.leave();
			return result;
		}

		public boolean overlapping_trans_conservative2 (AABB box, BoxBoxTransformCache trans1_to_0) {
			Stack stack = Stack.enter();
			AABB tbox = stack.alloc(box);
			tbox.appy_transform_trans_cache(trans1_to_0);
			boolean result = has_collision(tbox);
			stack.leave();
			return result;
		}

		/** transcache is the transformation cache from box to this AABB. */
		public boolean overlapping_trans_cache (AABB box, BoxBoxTransformCache transcache, boolean fulltest) {
			Stack stack = Stack.enter();
			Vector3 tmp = stack.allocVector3();

			// Taken from OPCODE
			Vector3 ea = stack.allocVector3(), eb = stack.allocVector3(); // extends
			Vector3 ca = stack.allocVector3(), cb = stack.allocVector3(); // extends
			get_center_extend(ca, ea);
			box.get_center_extend(cb, eb);

			Vector3 T = stack.allocVector3();
			float t, t2;

			// Class I : A's basis vectors
			for (int i = 0; i < 3; i++) {
				MatrixUtil.getRow(transcache.R1to0, i, tmp);
				VectorUtil.setCoord(T, i, tmp.dot(cb) + VectorUtil.getCoord(transcache.T1to0, i) - VectorUtil.getCoord(ca, i));

				MatrixUtil.getRow(transcache.AR, i, tmp);
				t = tmp.dot(eb) + VectorUtil.getCoord(ea, i);
				if (BT_GREATER(VectorUtil.getCoord(T, i), t)) {
					stack.leave();
					return false;
				}
			}
			// Class II : B's basis vectors
			for (int i = 0; i < 3; i++) {
				t = bt_mat3_dot_col(transcache.R1to0, T, i);
				t2 = bt_mat3_dot_col(transcache.AR, ea, i) + VectorUtil.getCoord(eb, i);
				if (BT_GREATER(t, t2)) {
					stack.leave();
					return false;
				}
			}
			// Class III : 9 cross products
			if (fulltest) {
				int m, n, o, p, q, r;
				for (int i = 0; i < 3; i++) {
					m = (i + 1) % 3;
					n = (i + 2) % 3;
					o = (i == 0) ? 1 : 0;
					p = (i == 2) ? 1 : 2;
					for (int j = 0; j < 3; j++) {
						q = j == 2 ? 1 : 2;
						r = j == 0 ? 1 : 0;
						t = VectorUtil.getCoord(T, n) * MatrixUtil.getElement(transcache.R1to0, m, j)
							- VectorUtil.getCoord(T, m) * MatrixUtil.getElement(transcache.R1to0, n, j);
						t2 = VectorUtil.getCoord(ea, o) * MatrixUtil.getElement(transcache.AR, p, j)
							+ VectorUtil.getCoord(ea, p) * MatrixUtil.getElement(transcache.AR, o, j)
							+ VectorUtil.getCoord(eb, r) * MatrixUtil.getElement(transcache.AR, i, q)
							+ VectorUtil.getCoord(eb, q) * MatrixUtil.getElement(transcache.AR, i, r);
						if (BT_GREATER(t, t2)) {
							stack.leave();
							return false;
						}
					}
				}
			}
			stack.leave();
			return true;
		}

		/** Simple test for planes. */
		public boolean collide_plane (Quaternion plane) {
			PlaneIntersectionType classify = plane_classify(plane);
			return (classify == PlaneIntersectionType.COLLIDE_PLANE);
		}

		/** Test for a triangle, with edges. */
		public boolean collide_triangle_exact (Vector3 p1, Vector3 p2, Vector3 p3, Quaternion triangle_plane) {
			if (!collide_plane(triangle_plane)) {
				return false;
			}
			Stack stack = Stack.enter();
			Vector3 center = stack.allocVector3(), extends_ = stack.allocVector3();
			get_center_extend(center, extends_);

			Vector3 v1 = stack.allocVector3();
			v1.set(p1).sub(center);
			Vector3 v2 = stack.allocVector3();
			v2.set(p2).sub(center);
			Vector3 v3 = stack.allocVector3();
			v3.set(p3).sub(center);

			// First axis
			Vector3 diff = stack.allocVector3();
			diff.set(v2).sub(v1);
			Vector3 abs_diff = stack.allocVector3();
			VectorUtil.absolute(abs_diff.set(diff));

			// Test With X axis
			TEST_CROSS_EDGE_BOX_X_AXIS_MCR(diff, abs_diff, v1, v3, extends_);
			// Test With Y axis
			TEST_CROSS_EDGE_BOX_Y_AXIS_MCR(diff, abs_diff, v1, v3, extends_);
			// Test With Z axis
			TEST_CROSS_EDGE_BOX_Z_AXIS_MCR(diff, abs_diff, v1, v3, extends_);

			diff.set(v3).sub(v2);
			VectorUtil.absolute(abs_diff.set(diff));

			// Test With X axis
			TEST_CROSS_EDGE_BOX_X_AXIS_MCR(diff, abs_diff, v2, v1, extends_);
			// Test With Y axis
			TEST_CROSS_EDGE_BOX_Y_AXIS_MCR(diff, abs_diff, v2, v1, extends_);
			// Test With Z axis
			TEST_CROSS_EDGE_BOX_Z_AXIS_MCR(diff, abs_diff, v2, v1, extends_);

			diff.set(v1).sub(v3);
			VectorUtil.absolute(abs_diff.set(diff));

			// Test With X axis
			TEST_CROSS_EDGE_BOX_X_AXIS_MCR(diff, abs_diff, v3, v2, extends_);
			// Test With Y axis
			TEST_CROSS_EDGE_BOX_Y_AXIS_MCR(diff, abs_diff, v3, v2, extends_);
			// Test With Z axis
			TEST_CROSS_EDGE_BOX_Z_AXIS_MCR(diff, abs_diff, v3, v2, extends_);

			stack.leave();
			return true;
		}
	}

}
