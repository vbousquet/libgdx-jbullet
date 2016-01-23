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

package com.bulletphysics.linearmath;

import com.badlogic.gdx.math.Matrix3;
import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.GdxRuntimeException;
import com.bulletphysics.BulletGlobals;
import com.bulletphysics.util.Stack;

/** Utility functions for matrices.
 * 
 * @author jezek2 */
public class MatrixUtil {

	public static void add (Matrix3 a, Matrix3 b) {
		a.val[Matrix3.M00] += b.val[Matrix3.M00];
		a.val[Matrix3.M01] += b.val[Matrix3.M01];
		a.val[Matrix3.M02] += b.val[Matrix3.M02];
		a.val[Matrix3.M10] += b.val[Matrix3.M10];
		a.val[Matrix3.M11] += b.val[Matrix3.M11];
		a.val[Matrix3.M12] += b.val[Matrix3.M12];
		a.val[Matrix3.M20] += b.val[Matrix3.M20];
		a.val[Matrix3.M21] += b.val[Matrix3.M21];
		a.val[Matrix3.M22] += b.val[Matrix3.M22];
	}

	public static float getElement (Matrix3 src, int row, int column) {
		switch (row) {
		case 0:
			switch (column) {
			case 0:
				return src.val[Matrix3.M00];
			case 1:
				return src.val[Matrix3.M01];
			case 2:
				return src.val[Matrix3.M02];
			default:
				break;
			}
			break;
		case 1:
			switch (column) {
			case 0:
				return src.val[Matrix3.M10];
			case 1:
				return src.val[Matrix3.M11];
			case 2:
				return src.val[Matrix3.M12];
			default:
				break;
			}
			break;

		case 2:
			switch (column) {
			case 0:
				return src.val[Matrix3.M20];
			case 1:
				return src.val[Matrix3.M21];
			case 2:
				return src.val[Matrix3.M22];
			default:
				break;
			}
			break;

		default:
			break;
		}
		throw new GdxRuntimeException("Invalid [row, col]");
	}

	public static void setElement (Matrix3 src, int row, int column, float v) {
		switch (row) {
		case 0:
			switch (column) {
			case 0:
				src.val[Matrix3.M00] = v;
				return;
			case 1:
				src.val[Matrix3.M01] = v;
				return;
			case 2:
				src.val[Matrix3.M02] = v;
				return;
			default:
				break;
			}
			break;
		case 1:
			switch (column) {
			case 0:
				src.val[Matrix3.M10] = v;
				return;
			case 1:
				src.val[Matrix3.M11] = v;
				return;
			case 2:
				src.val[Matrix3.M12] = v;
				return;
			default:
				break;
			}
			break;

		case 2:
			switch (column) {
			case 0:
				src.val[Matrix3.M20] = v;
				return;
			case 1:
				src.val[Matrix3.M21] = v;
				return;
			case 2:
				src.val[Matrix3.M22] = v;
				return;
			default:
				break;
			}
			break;

		default:
			break;
		}
		throw new GdxRuntimeException("Invalid [row, col]");
	}

	public static void getColumn (Matrix3 src, int col, Vector3 v) {
		if (col == 0) {
			v.x = src.val[Matrix3.M00];
			v.y = src.val[Matrix3.M10];
			v.z = src.val[Matrix3.M20];
		} else if (col == 1) {
			v.x = src.val[Matrix3.M01];
			v.y = src.val[Matrix3.M11];
			v.z = src.val[Matrix3.M21];
		} else if (col == 2) {
			v.x = src.val[Matrix3.M02];
			v.y = src.val[Matrix3.M12];
			v.z = src.val[Matrix3.M22];
		} else {
			throw new GdxRuntimeException("Invalid column");
		}
	}

	public static void setColumn (Matrix3 src, int col, Vector3 v) {
		if (col == 0) {
			src.val[Matrix3.M00] = v.x;
			src.val[Matrix3.M10] = v.y;
			src.val[Matrix3.M20] = v.z;
		} else if (col == 1) {
			src.val[Matrix3.M01] = v.x;
			src.val[Matrix3.M11] = v.y;
			src.val[Matrix3.M21] = v.z;
		} else if (col == 2) {
			src.val[Matrix3.M02] = v.x;
			src.val[Matrix3.M12] = v.y;
			src.val[Matrix3.M22] = v.z;
		} else {
			throw new GdxRuntimeException("Invalid column");
		}
	}

	public static void getRow (Matrix3 src, int row, Vector3 v) {
		if (row == 0) {
			v.x = src.val[Matrix3.M00];
			v.y = src.val[Matrix3.M01];
			v.z = src.val[Matrix3.M02];
		} else if (row == 1) {
			v.x = src.val[Matrix3.M10];
			v.y = src.val[Matrix3.M11];
			v.z = src.val[Matrix3.M12];
		} else if (row == 2) {
			v.x = src.val[Matrix3.M20];
			v.y = src.val[Matrix3.M21];
			v.z = src.val[Matrix3.M22];
		} else {
			throw new GdxRuntimeException("Invalid row");
		}
	}

	public static void setRow (Matrix3 src, int col, Vector3 v) {
		setRow(src, col, v.x, v.y, v.z);
	}

	public static void setRow (Matrix3 src, int row, float x, float y, float z) {
		if (row == 0) {
			src.val[Matrix3.M00] = x;
			src.val[Matrix3.M01] = y;
			src.val[Matrix3.M02] = z;
		} else if (row == 1) {
			src.val[Matrix3.M10] = x;
			src.val[Matrix3.M11] = y;
			src.val[Matrix3.M12] = z;
		} else if (row == 2) {
			src.val[Matrix3.M20] = x;
			src.val[Matrix3.M21] = y;
			src.val[Matrix3.M22] = z;
		} else {
			throw new GdxRuntimeException("Invalid row");
		}
	}

	public static void scale (Matrix3 dest, Matrix3 mat, Vector3 s) {
		dest.val[Matrix3.M00] = mat.val[Matrix3.M00] * s.x;
		dest.val[Matrix3.M01] = mat.val[Matrix3.M01] * s.y;
		dest.val[Matrix3.M02] = mat.val[Matrix3.M02] * s.z;
		dest.val[Matrix3.M10] = mat.val[Matrix3.M10] * s.x;
		dest.val[Matrix3.M11] = mat.val[Matrix3.M11] * s.y;
		dest.val[Matrix3.M12] = mat.val[Matrix3.M12] * s.z;
		dest.val[Matrix3.M20] = mat.val[Matrix3.M20] * s.x;
		dest.val[Matrix3.M21] = mat.val[Matrix3.M21] * s.y;
		dest.val[Matrix3.M22] = mat.val[Matrix3.M22] * s.z;
	}

	public static void absolute (Matrix3 mat) {
		mat.val[Matrix3.M00] = Math.abs(mat.val[Matrix3.M00]);
		mat.val[Matrix3.M01] = Math.abs(mat.val[Matrix3.M01]);
		mat.val[Matrix3.M02] = Math.abs(mat.val[Matrix3.M02]);
		mat.val[Matrix3.M10] = Math.abs(mat.val[Matrix3.M10]);
		mat.val[Matrix3.M11] = Math.abs(mat.val[Matrix3.M11]);
		mat.val[Matrix3.M12] = Math.abs(mat.val[Matrix3.M12]);
		mat.val[Matrix3.M20] = Math.abs(mat.val[Matrix3.M20]);
		mat.val[Matrix3.M21] = Math.abs(mat.val[Matrix3.M21]);
		mat.val[Matrix3.M22] = Math.abs(mat.val[Matrix3.M22]);
	}

	public static void setFromOpenGLSubMatrix (Matrix3 mat, float[] m) {
		mat.val[Matrix3.M00] = m[0];
		mat.val[Matrix3.M01] = m[4];
		mat.val[Matrix3.M02] = m[8];
		mat.val[Matrix3.M10] = m[1];
		mat.val[Matrix3.M11] = m[5];
		mat.val[Matrix3.M12] = m[9];
		mat.val[Matrix3.M20] = m[2];
		mat.val[Matrix3.M21] = m[6];
		mat.val[Matrix3.M22] = m[10];
	}

	public static void getOpenGLSubMatrix (Matrix3 mat, float[] m) {
		m[0] = mat.val[Matrix3.M00];
		m[1] = mat.val[Matrix3.M10];
		m[2] = mat.val[Matrix3.M20];
		m[3] = 0f;
		m[4] = mat.val[Matrix3.M01];
		m[5] = mat.val[Matrix3.M11];
		m[6] = mat.val[Matrix3.M21];
		m[7] = 0f;
		m[8] = mat.val[Matrix3.M02];
		m[9] = mat.val[Matrix3.M12];
		m[10] = mat.val[Matrix3.M22];
		m[11] = 0f;
	}

	/** Sets rotation matrix from euler angles. The euler angles are applied in ZYX order. This means a vector is first rotated
	 * about X then Y and then Z axis. */
	public static void setEulerZYX (Matrix3 mat, float eulerX, float eulerY, float eulerZ) {
		float ci = (float)Math.cos(eulerX);
		float cj = (float)Math.cos(eulerY);
		float ch = (float)Math.cos(eulerZ);
		float si = (float)Math.sin(eulerX);
		float sj = (float)Math.sin(eulerY);
		float sh = (float)Math.sin(eulerZ);
		float cc = ci * ch;
		float cs = ci * sh;
		float sc = si * ch;
		float ss = si * sh;

		setRow(mat, 0, cj * ch, sj * sc - cs, sj * cc + ss);
		setRow(mat, 1, cj * sh, sj * ss + cc, sj * cs - sc);
		setRow(mat, 2, -sj, cj * si, cj * ci);
	}

	private static float tdotx (Matrix3 mat, Vector3 vec) {
		return mat.val[Matrix3.M00] * vec.x + mat.val[Matrix3.M10] * vec.y + mat.val[Matrix3.M20] * vec.z;
	}

	private static float tdoty (Matrix3 mat, Vector3 vec) {
		return mat.val[Matrix3.M01] * vec.x + mat.val[Matrix3.M11] * vec.y + mat.val[Matrix3.M21] * vec.z;
	}

	private static float tdotz (Matrix3 mat, Vector3 vec) {
		return mat.val[Matrix3.M02] * vec.x + mat.val[Matrix3.M12] * vec.y + mat.val[Matrix3.M22] * vec.z;
	}

	public static void transposeTransform (Vector3 dest, Vector3 vec, Matrix3 mat) {
		float x = tdotx(mat, vec);
		float y = tdoty(mat, vec);
		float z = tdotz(mat, vec);
		dest.x = x;
		dest.y = y;
		dest.z = z;
	}

	public static void setRotation (Matrix3 dest, Quaternion q) {
		float d = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
		assert (d != 0f);
		float s = 2f / d;
		float xs = q.x * s, ys = q.y * s, zs = q.z * s;
		float wx = q.w * xs, wy = q.w * ys, wz = q.w * zs;
		float xx = q.x * xs, xy = q.x * ys, xz = q.x * zs;
		float yy = q.y * ys, yz = q.y * zs, zz = q.z * zs;
		dest.val[Matrix3.M00] = 1f - (yy + zz);
		dest.val[Matrix3.M01] = xy - wz;
		dest.val[Matrix3.M02] = xz + wy;
		dest.val[Matrix3.M10] = xy + wz;
		dest.val[Matrix3.M11] = 1f - (xx + zz);
		dest.val[Matrix3.M12] = yz - wx;
		dest.val[Matrix3.M20] = xz - wy;
		dest.val[Matrix3.M21] = yz + wx;
		dest.val[Matrix3.M22] = 1f - (xx + yy);
	}

	public static void getRotation (Matrix3 mat, Quaternion dest) {
		/*
		 * ArrayPool<float[]> floatArrays = ArrayPool.get(float.class);
		 * 
		 * float trace = mat.val[Matrix3.M00] + mat.val[Matrix3.M11] + mat.val[Matrix3.M22]; float[] temp = floatArrays.getFixed(4);
		 * 
		 * if (trace > 0f) { float s = (float) Math.sqrt(trace + 1f); temp[3] = (s * 0.5f); s = 0.5f / s;
		 * 
		 * temp[0] = ((mat.val[Matrix3.M21] - mat.val[Matrix3.M12]) * s); temp[1] = ((mat.val[Matrix3.M02] - mat.val[Matrix3.M20]) *
		 * s); temp[2] = ((mat.val[Matrix3.M10] - mat.val[Matrix3.M01]) * s); } else { int i = mat.val[Matrix3.M00] <
		 * mat.val[Matrix3.M11] ? (mat.val[Matrix3.M11] < mat.val[Matrix3.M22] ? 2 : 1) : (mat.val[Matrix3.M00] <
		 * mat.val[Matrix3.M22] ? 2 : 0); int j = (i + 1) % 3; int k = (i + 2) % 3;
		 * 
		 * float s = (float) Math.sqrt(mat.getElement(i, i) - mat.getElement(j, j) - mat.getElement(k, k) + 1f); temp[i] = s * 0.5f;
		 * s = 0.5f / s;
		 * 
		 * temp[3] = (mat.getElement(k, j) - mat.getElement(j, k)) * s; temp[j] = (mat.getElement(j, i) + mat.getElement(i, j)) * s;
		 * temp[k] = (mat.getElement(k, i) + mat.getElement(i, k)) * s; } dest.set(temp[0], temp[1], temp[2], temp[3]);
		 * 
		 * floatArrays.release(temp);
		 */
		// FIXME check this is correct
		dest.setFromMatrix(true, mat);
	}

	private static float cofac (Matrix3 mat, int r1, int c1, int r2, int c2) {
		return getElement(mat, r1, c1) * getElement(mat, r2, c2) - getElement(mat, r1, c2) * getElement(mat, r2, c1);
	}

	public static void invert (Matrix3 mat) {
		float co_x = cofac(mat, 1, 1, 2, 2);
		float co_y = cofac(mat, 1, 2, 2, 0);
		float co_z = cofac(mat, 1, 0, 2, 1);

		float det = mat.val[Matrix3.M00] * co_x + mat.val[Matrix3.M01] * co_y + mat.val[Matrix3.M02] * co_z;
		assert (det != 0f);

		float s = 1f / det;
		float m00 = co_x * s;
		float m01 = cofac(mat, 0, 2, 2, 1) * s;
		float m02 = cofac(mat, 0, 1, 1, 2) * s;
		float m10 = co_y * s;
		float m11 = cofac(mat, 0, 0, 2, 2) * s;
		float m12 = cofac(mat, 0, 2, 1, 0) * s;
		float m20 = co_z * s;
		float m21 = cofac(mat, 0, 1, 2, 0) * s;
		float m22 = cofac(mat, 0, 0, 1, 1) * s;

		mat.val[Matrix3.M00] = m00;
		mat.val[Matrix3.M01] = m01;
		mat.val[Matrix3.M02] = m02;
		mat.val[Matrix3.M10] = m10;
		mat.val[Matrix3.M11] = m11;
		mat.val[Matrix3.M12] = m12;
		mat.val[Matrix3.M20] = m20;
		mat.val[Matrix3.M21] = m21;
		mat.val[Matrix3.M22] = m22;
	}

	/** Diagonalizes this matrix by the Jacobi method. rot stores the rotation from the coordinate system in which the matrix is
	 * diagonal to the original coordinate system, i.e., old_this = rot * new_this * rot^T. The iteration stops when all
	 * off-diagonal elements are less than the threshold multiplied by the sum of the absolute values of the diagonal, or when
	 * maxSteps have been executed. Note that this matrix is assumed to be symmetric. */
	// JAVA NOTE: diagonalize method from 2.71
	public static void diagonalize (Matrix3 mat, Matrix3 rot, float threshold, int maxSteps) {
		Stack stack = Stack.enter();
		Vector3 row = stack.allocVector3();

		rot.idt();
		for (int step = maxSteps; step > 0; step--) {
			// find off-diagonal element [p][q] with largest magnitude
			int p = 0;
			int q = 1;
			int r = 2;
			float max = Math.abs(mat.val[Matrix3.M01]);
			float v = Math.abs(mat.val[Matrix3.M02]);
			if (v > max) {
				q = 2;
				r = 1;
				max = v;
			}
			v = Math.abs(mat.val[Matrix3.M12]);
			if (v > max) {
				p = 1;
				q = 2;
				r = 0;
				max = v;
			}

			float t = threshold * (Math.abs(mat.val[Matrix3.M00]) + Math.abs(mat.val[Matrix3.M11]) + Math.abs(mat.val[Matrix3.M22]));
			if (max <= t) {
				if (max <= BulletGlobals.SIMD_EPSILON * t) {
					return;
				}
				step = 1;
			}

			// compute Jacobi rotation J which leads to a zero for element [p][q]
			float mpq = getElement(mat, p, q);
			float theta = (getElement(mat, q, q) - getElement(mat, p, p)) / (2 * mpq);
			float theta2 = theta * theta;
			float cos;
			float sin;
			if ((theta2 * theta2) < (10f / BulletGlobals.SIMD_EPSILON)) {
				t = (theta >= 0f) ? 1f / (theta + (float)Math.sqrt(1f + theta2)) : 1f / (theta - (float)Math.sqrt(1f + theta2));
				cos = 1f / (float)Math.sqrt(1f + t * t);
				sin = cos * t;
			} else {
				// approximation for large theta-value, i.e., a nearly diagonal matrix
				t = 1 / (theta * (2 + 0.5f / theta2));
				cos = 1 - 0.5f * t * t;
				sin = cos * t;
			}

			// apply rotation to matrix (this = J^T * this * J)
			setElement(mat, p, q, 0f);
			setElement(mat, q, p, 0f);
			setElement(mat, p, p, getElement(mat, p, p) - t * mpq);
			setElement(mat, q, q, getElement(mat, q, q) + t * mpq);
			float mrp = getElement(mat, r, p);
			float mrq = getElement(mat, r, q);
			setElement(mat, r, p, cos * mrp - sin * mrq);
			setElement(mat, p, r, cos * mrp - sin * mrq);
			setElement(mat, r, q, cos * mrq + sin * mrp);
			setElement(mat, q, r, cos * mrq + sin * mrp);

			// apply rotation to rot (rot = rot * J)
			for (int i = 0; i < 3; i++) {
				getRow(rot, i, row);

				mrp = VectorUtil.getCoord(row, p);
				mrq = VectorUtil.getCoord(row, q);
				VectorUtil.setCoord(row, p, cos * mrp - sin * mrq);
				VectorUtil.setCoord(row, q, cos * mrq + sin * mrp);
				setRow(rot, i, row);
			}
		}
		stack.leave();
	}

}
