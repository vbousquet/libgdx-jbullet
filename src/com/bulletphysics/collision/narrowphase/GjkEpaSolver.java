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

package com.bulletphysics.collision.narrowphase;

import java.util.Arrays;

import com.badlogic.gdx.math.Matrix3;
import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import com.bulletphysics.BulletGlobals;
import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.QuaternionUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.ArrayPool;
import com.bulletphysics.util.ObjectStackList;
import com.bulletphysics.util.Stack;
import com.bulletphysics.util.Supplier;

/*
GJK-EPA collision solver by Nathanael Presson
Nov.2006
*/

/** GjkEpaSolver contributed under zlib by Nathanael Presson.
 * 
 * @author jezek2 */
public class GjkEpaSolver {

	protected final ArrayPool<float[]> floatArrays = ArrayPool.get(float.class);

	protected final ObjectStackList<Mkv> stackMkv = new ObjectStackList<Mkv>(new Supplier<Mkv>() {
		@Override
		public Mkv get () {
			return new Mkv();
		}
	});

	protected final ObjectStackList<He> stackHe = new ObjectStackList<He>(new Supplier<He>() {
		@Override
		public He get () {
			return new He();
		}
	});

	protected final ObjectStackList<Face> stackFace = new ObjectStackList<Face>(new Supplier<Face>() {
		@Override
		public Face get () {
			return new Face();
		}
	});

	protected void pushStack () {
		stackMkv.push();
		stackHe.push();
		stackFace.push();
	}

	protected void popStack () {
		stackMkv.pop();
		stackHe.pop();
		stackFace.pop();
	}

	public enum ResultsStatus {
		Separated, /* Shapes doesnt penetrate */
		Penetrating, /* Shapes are penetrating */
		GJK_Failed, /* GJK phase fail, no big issue, shapes are probably just 'touching' */
		EPA_Failed, /* EPA phase fail, bigger problem, need to save parameters, and debug */
	}

	public static class Results {
		public ResultsStatus status;
		public final Vector3[] witnesses/* [2] */ = new Vector3[] {new Vector3(), new Vector3()};
		public final Vector3 normal = new Vector3();
		public float depth;
		public int epa_iterations;
		public int gjk_iterations;
	}

	////////////////////////////////////////////////////////////////////////////

	private static final float cstInf = BulletGlobals.SIMD_INFINITY;
	private static final float cstPi = BulletGlobals.SIMD_PI;
	private static final float cst2Pi = BulletGlobals.SIMD_2_PI;
	private static final int GJK_maxiterations = 128;
	private static final int GJK_hashsize = 1 << 6;
	private static final int GJK_hashmask = GJK_hashsize - 1;
	private static final float GJK_insimplex_eps = 0.0001f;
	private static final float GJK_sqinsimplex_eps = GJK_insimplex_eps * GJK_insimplex_eps;
	private static final int EPA_maxiterations = 256;
	private static final float EPA_inface_eps = 0.01f;
	private static final float EPA_accuracy = 0.001f;

	////////////////////////////////////////////////////////////////////////////

	public static class Mkv {
		public final Vector3 w = new Vector3(); // Minkowski vertice
		public final Vector3 r = new Vector3(); // Ray

		public void set (Mkv m) {
			w.set(m.w);
			r.set(m.r);
		}
	}

	public static class He {
		public final Vector3 v = new Vector3();
		public He n;
	}

	protected class GJK {
		// protected final BulletStack stack = BulletStack.get();

		// public btStackAlloc sa;
		// public Block sablock;
		public final He[] table = new He[GJK_hashsize];
		public final Matrix3[] wrotations/* [2] */ = new Matrix3[] {new Matrix3(), new Matrix3()};
		public final Vector3[] positions/* [2] */ = new Vector3[] {new Vector3(), new Vector3()};
		public final ConvexShape[] shapes = new ConvexShape[2];
		public final Mkv[] simplex = new Mkv[5];
		public final Vector3 ray = new Vector3();
		public /* unsigned */ int order;
		public /* unsigned */ int iterations;
		public float margin;
		public boolean failed;

		{
			for (int i = 0; i < simplex.length; i++)
				simplex[i] = new Mkv();
		}

		public GJK () {
		}

		public GJK (/* StackAlloc psa, */
			Matrix3 wrot0, Vector3 pos0, ConvexShape shape0, Matrix3 wrot1, Vector3 pos1, ConvexShape shape1) {
			this(wrot0, pos0, shape0, wrot1, pos1, shape1, 0f);
		}

		public GJK (/* StackAlloc psa, */
			Matrix3 wrot0, Vector3 pos0, ConvexShape shape0, Matrix3 wrot1, Vector3 pos1, ConvexShape shape1, float pmargin) {
			init(wrot0, pos0, shape0, wrot1, pos1, shape1, pmargin);
		}

		public void init (/* StackAlloc psa, */
			Matrix3 wrot0, Vector3 pos0, ConvexShape shape0, Matrix3 wrot1, Vector3 pos1, ConvexShape shape1, float pmargin) {
			pushStack();
			wrotations[0].set(wrot0);
			positions[0].set(pos0);
			shapes[0] = shape0;
			wrotations[1].set(wrot1);
			positions[1].set(pos1);
			shapes[1] = shape1;
			// sa =psa;
			// sablock =sa->beginBlock();
			margin = pmargin;
			failed = false;
		}

		public void destroy () {
			popStack();
		}

		// vdh: very dummy hash
		public /* unsigned */ int Hash (Vector3 v) {
			int h = (int)(v.x * 15461) ^ (int)(v.y * 83003) ^ (int)(v.z * 15473);
			return (h * 169639) & GJK_hashmask;
		}

		public Vector3 LocalSupport (Vector3 d, /* unsigned */ int i, Vector3 out) {
			Stack stack = Stack.enter();
			Vector3 tmp = stack.allocVector3();
			MatrixUtil.transposeTransform(tmp, d, wrotations[i]);

			shapes[i].localGetSupportingVertex(tmp, out);
			out.mul(wrotations[i]);
			out.add(positions[i]);
			stack.leave();
			return out;
		}

		public void Support (Vector3 d, Mkv v) {
			v.r.set(d);
			Stack stack = Stack.enter();
			Vector3 tmp1 = LocalSupport(d, 0, stack.allocVector3());

			Vector3 tmp = stack.allocVector3();
			tmp.set(d);
			tmp.scl(-1);
			Vector3 tmp2 = LocalSupport(tmp, 1, stack.allocVector3());

			v.w.set(tmp1).sub(tmp2);
			v.w.x += margin * d.x;
			v.w.y += margin * d.y;
			v.w.z += margin * d.z;
			stack.leave();
		}

		public boolean FetchSupport () {
			int h = Hash(ray);
			He e = table[h];
			while (e != null) {
				if (e.v.equals(ray)) {
					--order;
					return false;
				} else {
					e = e.n;
				}
			}
			// e = (He*)sa->allocate(sizeof(He));
			// e = new He();
			e = stackHe.get();
			e.v.set(ray);
			e.n = table[h];
			table[h] = e;
			Support(ray, simplex[++order]);
			return (ray.dot(simplex[order].w) > 0);
		}

		public boolean SolveSimplex2 (Vector3 ao, Vector3 ab) {
			if (ab.dot(ao) >= 0) {
				Stack stack = Stack.enter();
				Vector3 cabo = stack.allocVector3();
				cabo.set(ab).crs(ao);
				if (cabo.len2() > GJK_sqinsimplex_eps) {
					ray.set(cabo).crs(ab);
					stack.leave();
				} else {
					stack.leave();
					return true;
				}
			} else {
				order = 0;
				simplex[0].set(simplex[1]);
				ray.set(ao);
			}
			return (false);
		}

		public boolean SolveSimplex3 (Vector3 ao, Vector3 ab, Vector3 ac) {
			Stack stack = Stack.enter();
			Vector3 tmp = stack.allocVector3();
			tmp.set(ab).crs(ac);
			boolean result = (SolveSimplex3a(ao, ab, ac, tmp));
			stack.leave();
			return result;
		}

		public boolean SolveSimplex3a (Vector3 ao, Vector3 ab, Vector3 ac, Vector3 cabc) {
			// TODO: optimize
			Stack stack = Stack.enter();
			Vector3 tmp = stack.allocVector3();
			tmp.set(cabc).crs(ab);

			Vector3 tmp2 = stack.allocVector3();
			tmp2.set(cabc).crs(ac);

			boolean result;
			if (tmp.dot(ao) < -GJK_insimplex_eps) {
				order = 1;
				simplex[0].set(simplex[1]);
				simplex[1].set(simplex[2]);
				result = SolveSimplex2(ao, ab);
			} else if (tmp2.dot(ao) > +GJK_insimplex_eps) {
				order = 1;
				simplex[1].set(simplex[2]);
				result = SolveSimplex2(ao, ac);
			} else {
				float d = cabc.dot(ao);
				if (Math.abs(d) > GJK_insimplex_eps) {
					if (d > 0) {
						ray.set(cabc);
					} else {
						ray.set(cabc).scl(-1);

						Mkv swapTmp = new Mkv();
						swapTmp.set(simplex[0]);
						simplex[0].set(simplex[1]);
						simplex[1].set(swapTmp);
					}
					result = false;
				} else {
					result = true;
				}
			}
			stack.leave();
			return result;
		}

		public boolean SolveSimplex4 (Vector3 ao, Vector3 ab, Vector3 ac, Vector3 ad) {
			// TODO: optimize
			Stack stack = Stack.enter();
			Vector3 crs = stack.allocVector3();

			Vector3 tmp = stack.allocVector3();
			tmp.set(ab).crs(ac);

			Vector3 tmp2 = stack.allocVector3();
			tmp2.set(ac).crs(ad);

			Vector3 tmp3 = stack.allocVector3();
			tmp3.set(ad).crs(ab);

			boolean result;
			if (tmp.dot(ao) > GJK_insimplex_eps) {
				crs.set(tmp);
				order = 2;
				simplex[0].set(simplex[1]);
				simplex[1].set(simplex[2]);
				simplex[2].set(simplex[3]);
				result = SolveSimplex3a(ao, ab, ac, crs);
			} else if (tmp2.dot(ao) > GJK_insimplex_eps) {
				crs.set(tmp2);
				order = 2;
				simplex[2].set(simplex[3]);
				result = SolveSimplex3a(ao, ac, ad, crs);
			} else if (tmp3.dot(ao) > GJK_insimplex_eps) {
				crs.set(tmp3);
				order = 2;
				simplex[1].set(simplex[0]);
				simplex[0].set(simplex[2]);
				simplex[2].set(simplex[3]);
				result = SolveSimplex3a(ao, ad, ab, crs);
			} else {
				result = (true);
			}
			stack.leave();
			return result;
		}

		public boolean SearchOrigin () {
			Stack stack = Stack.enter();
			Vector3 tmp = stack.allocVector3();
			tmp.set(1f, 0f, 0f);
			boolean result = SearchOrigin(tmp);
			stack.leave();
			return result;
		}

		public boolean SearchOrigin (Vector3 initray) {
			Stack stack = Stack.enter();
			Vector3 tmp1 = stack.allocVector3();
			Vector3 tmp2 = stack.allocVector3();
			Vector3 tmp3 = stack.allocVector3();
			Vector3 tmp4 = stack.allocVector3();

			iterations = 0;
			order = -1;
			failed = false;
			ray.set(initray);
			ray.nor();

			Arrays.fill(table, null);

			FetchSupport();
			ray.set(simplex[0].w).scl(-1);
			for (; iterations < GJK_maxiterations; ++iterations) {
				float rl = ray.len();
				ray.scl(1f / (rl > 0f ? rl : 1f));
				if (FetchSupport()) {
					boolean found = false;
					switch (order) {
					case 1: {
						tmp1.set(simplex[1].w).scl(-1);
						tmp2.set(simplex[0].w).sub(simplex[1].w);
						found = SolveSimplex2(tmp1, tmp2);
						break;
					}
					case 2: {
						tmp1.set(simplex[2].w).scl(-1);
						tmp2.set(simplex[1].w).sub(simplex[2].w);
						tmp3.set(simplex[0].w).sub(simplex[2].w);
						found = SolveSimplex3(tmp1, tmp2, tmp3);
						break;
					}
					case 3: {
						tmp1.set(simplex[3].w).scl(-1);
						tmp2.set(simplex[2].w).sub(simplex[3].w);
						tmp3.set(simplex[1].w).sub(simplex[3].w);
						tmp4.set(simplex[0].w).sub(simplex[3].w);
						found = SolveSimplex4(tmp1, tmp2, tmp3, tmp4);
						break;
					}
					}
					if (found) {
						stack.leave();
						return true;
					}
				} else {
					stack.leave();
					return false;
				}
			}
			failed = true;
			stack.leave();
			return false;
		}

		public boolean EncloseOrigin () {
			Stack stack = Stack.enter();
			Vector3 tmp = stack.allocVector3();
			Vector3 tmp1 = stack.allocVector3();
			Vector3 tmp2 = stack.allocVector3();

			switch (order) {
			// Point
			case 0:
				break;
			// Line
			case 1: {
				Vector3 ab = stack.allocVector3();
				ab.set(simplex[1].w).sub(simplex[0].w);

				Vector3[] b = new Vector3[] {stack.allocVector3(), stack.allocVector3(), stack.allocVector3()};
				b[0].set(1f, 0f, 0f);
				b[1].set(0f, 1f, 0f);
				b[2].set(0f, 0f, 1f);

				b[0].set(ab).crs(b[0]);
				b[1].set(ab).crs(b[1]);
				b[2].set(ab).crs(b[2]);

				float m[] = new float[] {b[0].len2(), b[1].len2(), b[2].len2()};

				Quaternion tmpQuat = stack.allocQuaternion();
				tmp.set(ab).nor();
				QuaternionUtil.setRotation(tmpQuat, tmp, cst2Pi / 3f);

				Matrix3 r = stack.allocMatrix3();
				MatrixUtil.setRotation(r, tmpQuat);

				Vector3 w = stack.allocVector3();
				w.set(b[m[0] > m[1] ? m[0] > m[2] ? 0 : 2 : m[1] > m[2] ? 1 : 2]);

				tmp.set(w).nor();
				Support(tmp, simplex[4]);
				w.mul(r);
				tmp.set(w).nor();
				Support(tmp, simplex[2]);
				w.mul(r);
				tmp.set(w).nor();
				Support(tmp, simplex[3]);
				w.mul(r);
				order = 4;
				stack.leave();
				return (true);
			}
				// Triangle
			case 2: {
				tmp1.set(simplex[1].w).sub(simplex[0].w);
				tmp2.set(simplex[2].w).sub(simplex[0].w);
				Vector3 n = stack.allocVector3();
				n.set(tmp1).crs(tmp2);
				n.nor();

				Support(n, simplex[3]);

				tmp.set(n).scl(-1);
				Support(tmp, simplex[4]);
				order = 4;
				stack.leave();
				return (true);
			}
				// Tetrahedron
			case 3:
				stack.leave();
				return (true);
			// Hexahedron
			case 4:
				stack.leave();
				return (true);
			}
			stack.leave();
			return (false);
		}

	}

	////////////////////////////////////////////////////////////////////////////

	private static int[] mod3 = new int[] {0, 1, 2, 0, 1};

	private static final int[][] tetrahedron_fidx/* [4][3] */ = new int[][] {{2, 1, 0}, {3, 0, 1}, {3, 1, 2}, {3, 2, 0}};
	private static final int[][] tetrahedron_eidx/* [6][4] */ = new int[][] {{0, 0, 2, 1}, {0, 1, 1, 1}, {0, 2, 3, 1},
		{1, 0, 3, 2}, {2, 0, 1, 2}, {3, 0, 2, 2}};

	private static final int[][] hexahedron_fidx/* [6][3] */ = new int[][] {{2, 0, 4}, {4, 1, 2}, {1, 4, 0}, {0, 3, 1}, {0, 2, 3},
		{1, 3, 2}};
	private static final int[][] hexahedron_eidx/* [9][4] */ = new int[][] {{0, 0, 4, 0}, {0, 1, 2, 1}, {0, 2, 1, 2}, {1, 1, 5, 2},
		{1, 0, 2, 0}, {2, 2, 3, 2}, {3, 1, 5, 0}, {3, 0, 4, 2}, {5, 1, 4, 1}};

	public static class Face {
		public final Mkv[] v = new Mkv[3];
		public final Face[] f = new Face[3];
		public final int[] e = new int[3];
		public final Vector3 n = new Vector3();
		public float d;
		public int mark;
		public Face prev;
		public Face next;
	}

	protected class EPA {
		// protected final BulletStack stack = BulletStack.get();

		public GJK gjk;
		// public btStackAlloc* sa;
		public Face root;
		public int nfaces;
		public int iterations;
		public final Vector3[][] features = new Vector3[2][3];
		public final Vector3[] nearest/* [2] */ = new Vector3[] {new Vector3(), new Vector3()};
		public final Vector3 normal = new Vector3();
		public float depth;
		public boolean failed;

		{
			for (int i = 0; i < features.length; i++) {
				for (int j = 0; j < features[i].length; j++) {
					features[i][j] = new Vector3();
				}
			}
		}

		public EPA (GJK pgjk) {
			gjk = pgjk;
			// sa = pgjk->sa;
		}

		public Vector3 GetCoordinates (Face face, Vector3 out) {
			Stack stack = Stack.enter();
			Vector3 tmp = stack.allocVector3();
			Vector3 tmp1 = stack.allocVector3();
			Vector3 tmp2 = stack.allocVector3();

			Vector3 o = stack.allocVector3();
			o.set(face.n).scl(-face.d);

			float[] a = floatArrays.getFixed(3);

			tmp1.set(face.v[0].w).sub(o);
			tmp2.set(face.v[1].w).sub(o);
			tmp.set(tmp1).crs(tmp2);
			a[0] = tmp.len();

			tmp1.set(face.v[1].w).sub(o);
			tmp2.set(face.v[2].w).sub(o);
			tmp.set(tmp1).crs(tmp2);
			a[1] = tmp.len();

			tmp1.set(face.v[2].w).sub(o);
			tmp2.set(face.v[0].w).sub(o);
			tmp.set(tmp1).crs(tmp2);
			a[2] = tmp.len();

			float sm = a[0] + a[1] + a[2];

			out.set(a[1], a[2], a[0]);
			out.scl(1f / (sm > 0f ? sm : 1f));

			floatArrays.release(a);
			stack.leave();
			return out;
		}

		public Face FindBest () {
			Face bf = null;
			if (root != null) {
				Face cf = root;
				float bd = cstInf;
				do {
					if (cf.d < bd) {
						bd = cf.d;
						bf = cf;
					}
				} while (null != (cf = cf.next));
			}
			return bf;
		}

		public boolean Set (Face f, Mkv a, Mkv b, Mkv c) {
			Stack stack = Stack.enter();
			Vector3 tmp1 = stack.allocVector3();
			Vector3 tmp2 = stack.allocVector3();
			Vector3 tmp3 = stack.allocVector3();

			Vector3 nrm = stack.allocVector3();
			tmp1.set(b.w).sub(a.w);
			tmp2.set(c.w).sub(a.w);
			nrm.set(tmp1).crs(tmp2);

			float len = nrm.len();

			tmp1.set(a.w).crs(b.w);
			tmp2.set(b.w).crs(c.w);
			tmp3.set(c.w).crs(a.w);

			boolean valid = (tmp1.dot(nrm) >= -EPA_inface_eps) && (tmp2.dot(nrm) >= -EPA_inface_eps)
				&& (tmp3.dot(nrm) >= -EPA_inface_eps);

			f.v[0] = a;
			f.v[1] = b;
			f.v[2] = c;
			f.mark = 0;
			f.n.set(nrm).scl(1f / (len > 0f ? len : cstInf));
			f.d = Math.max(0, -f.n.dot(a.w));
			stack.leave();
			return valid;
		}

		public Face NewFace (Mkv a, Mkv b, Mkv c) {
			// Face pf = new Face();
			Face pf = stackFace.get();
			if (Set(pf, a, b, c)) {
				if (root != null) {
					root.prev = pf;
				}
				pf.prev = null;
				pf.next = root;
				root = pf;
				++nfaces;
			} else {
				pf.prev = pf.next = null;
			}
			return (pf);
		}

		public void Detach (Face face) {
			if (face.prev != null || face.next != null) {
				--nfaces;
				if (face == root) {
					root = face.next;
					root.prev = null;
				} else {
					if (face.next == null) {
						face.prev.next = null;
					} else {
						face.prev.next = face.next;
						face.next.prev = face.prev;
					}
				}
				face.prev = face.next = null;
			}
		}

		public void Link (Face f0, int e0, Face f1, int e1) {
			f0.f[e0] = f1;
			f1.e[e1] = e0;
			f1.f[e1] = f0;
			f0.e[e0] = e1;
		}

		public Mkv Support (Vector3 w) {
			// Mkv v = new Mkv();
			Mkv v = stackMkv.get();
			gjk.Support(w, v);
			return v;
		}

		public int BuildHorizon (int markid, Mkv w, Face f, int e, Face[] cf, Face[] ff) {
			int ne = 0;
			if (f.mark != markid) {
				int e1 = mod3[e + 1];
				if ((f.n.dot(w.w) + f.d) > 0) {
					Face nf = NewFace(f.v[e1], f.v[e], w);
					Link(nf, 0, f, e);
					if (cf[0] != null) {
						Link(cf[0], 1, nf, 2);
					} else {
						ff[0] = nf;
					}
					cf[0] = nf;
					ne = 1;
				} else {
					int e2 = mod3[e + 2];
					Detach(f);
					f.mark = markid;
					ne += BuildHorizon(markid, w, f.f[e1], f.e[e1], cf, ff);
					ne += BuildHorizon(markid, w, f.f[e2], f.e[e2], cf, ff);
				}
			}
			return (ne);
		}

		public float EvaluatePD () {
			return EvaluatePD(EPA_accuracy);
		}

		public float EvaluatePD (float accuracy) {
			pushStack();
			Stack stack = Stack.enter();
			int sp = stack.getSp();
			try {
				Vector3 tmp = stack.allocVector3();

				// btBlock* sablock = sa->beginBlock();
				Face bestface = null;
				int markid = 1;
				depth = -cstInf;
				normal.set(0f, 0f, 0f);
				root = null;
				nfaces = 0;
				iterations = 0;
				failed = false;
				/* Prepare hull */
				if (gjk.EncloseOrigin()) {
					// const U* pfidx = 0;
					int[][] pfidx_ptr = null;
					int pfidx_index = 0;

					int nfidx = 0;
					// const U* peidx = 0;
					int[][] peidx_ptr = null;
					int peidx_index = 0;

					int neidx = 0;
					Mkv[] basemkv = new Mkv[5];
					Face[] basefaces = new Face[6];
					switch (gjk.order) {
					// Tetrahedron
					case 3: {
						// pfidx=(const U*)fidx;
						pfidx_ptr = tetrahedron_fidx;
						pfidx_index = 0;

						nfidx = 4;

						// peidx=(const U*)eidx;
						peidx_ptr = tetrahedron_eidx;
						peidx_index = 0;

						neidx = 6;
					}
						break;
					// Hexahedron
					case 4: {
						// pfidx=(const U*)fidx;
						pfidx_ptr = hexahedron_fidx;
						pfidx_index = 0;

						nfidx = 6;

						// peidx=(const U*)eidx;
						peidx_ptr = hexahedron_eidx;
						peidx_index = 0;

						neidx = 9;
					}
						break;
					}
					int i;

					for (i = 0; i <= gjk.order; ++i) {
						basemkv[i] = new Mkv();
						basemkv[i].set(gjk.simplex[i]);
					}
					for (i = 0; i < nfidx; ++i, pfidx_index++) {
						basefaces[i] = NewFace(basemkv[pfidx_ptr[pfidx_index][0]], basemkv[pfidx_ptr[pfidx_index][1]],
							basemkv[pfidx_ptr[pfidx_index][2]]);
					}
					for (i = 0; i < neidx; ++i, peidx_index++) {
						Link(basefaces[peidx_ptr[peidx_index][0]], peidx_ptr[peidx_index][1], basefaces[peidx_ptr[peidx_index][2]],
							peidx_ptr[peidx_index][3]);
					}
				}
				if (0 == nfaces) {
					// sa->endBlock(sablock);
					return (depth);
				}
				/* Expand hull */
				for (; iterations < EPA_maxiterations; ++iterations) {
					Face bf = FindBest();
					if (bf != null) {
						tmp.set(bf.n).scl(-1);
						Mkv w = Support(tmp);
						float d = bf.n.dot(w.w) + bf.d;
						bestface = bf;
						if (d < -accuracy) {
							Face[] cf = new Face[] {null};
							Face[] ff = new Face[] {null};
							int nf = 0;
							Detach(bf);
							bf.mark = ++markid;
							for (int i = 0; i < 3; ++i) {
								nf += BuildHorizon(markid, w, bf.f[i], bf.e[i], cf, ff);
							}
							if (nf <= 2) {
								break;
							}
							Link(cf[0], 1, ff[0], 2);
						} else {
							break;
						}
					} else {
						break;
					}
				}
				/* Extract contact */
				if (bestface != null) {
					Vector3 b = GetCoordinates(bestface, stack.allocVector3());
					normal.set(bestface.n);
					depth = Math.max(0, bestface.d);
					for (int i = 0; i < 2; ++i) {
						float s = i != 0 ? -1f : 1f;
						for (int j = 0; j < 3; ++j) {
							tmp.set(bestface.v[j].r).scl(s);
							gjk.LocalSupport(tmp, i, features[i][j]);
						}
					}

					Vector3 tmp1 = stack.allocVector3();
					Vector3 tmp2 = stack.allocVector3();
					Vector3 tmp3 = stack.allocVector3();

					tmp1.set(features[0][0]).scl(b.x);
					tmp2.set(features[0][1]).scl(b.y);
					tmp3.set(features[0][2]).scl(b.z);
					VectorUtil.add(nearest[0], tmp1, tmp2, tmp3);

					tmp1.set(features[1][0]).scl(b.x);
					tmp2.set(features[1][1]).scl(b.y);
					tmp3.set(features[1][2]).scl(b.z);
					VectorUtil.add(nearest[1], tmp1, tmp2, tmp3);
				} else {
					failed = true;
				}
				// sa->endBlock(sablock);
				return (depth);
			} finally {
				popStack();
				stack.leave(sp);
			}
		}

	}

	////////////////////////////////////////////////////////////////////////////

	private GJK gjk = new GJK();

	public boolean collide (ConvexShape shape0, Transform wtrs0, ConvexShape shape1, Transform wtrs1,
		float radialmargin/*
								 * , btStackAlloc* stackAlloc
								 */, Results results) {
		Stack stack = Stack.enter();
		int sp = stack.getSp();
		// Initialize
		results.witnesses[0].set(0f, 0f, 0f);
		results.witnesses[1].set(0f, 0f, 0f);
		results.normal.set(0f, 0f, 0f);
		results.depth = 0;
		results.status = ResultsStatus.Separated;
		results.epa_iterations = 0;
		results.gjk_iterations = 0;
		/* Use GJK to locate origin */
		gjk.init(/* stackAlloc, */
			wtrs0.basis, wtrs0.origin, shape0, wtrs1.basis, wtrs1.origin, shape1, radialmargin + EPA_accuracy);
		try {
			boolean collide = gjk.SearchOrigin();
			results.gjk_iterations = gjk.iterations + 1;
			if (collide) {
				/* Then EPA for penetration depth */
				EPA epa = new EPA(gjk);
				float pd = epa.EvaluatePD();
				results.epa_iterations = epa.iterations + 1;
				if (pd > 0) {
					results.status = ResultsStatus.Penetrating;
					results.normal.set(epa.normal);
					results.depth = pd;
					results.witnesses[0].set(epa.nearest[0]);
					results.witnesses[1].set(epa.nearest[1]);
					return (true);
				} else {
					if (epa.failed) {
						results.status = ResultsStatus.EPA_Failed;
					}
				}
			} else {
				if (gjk.failed) {
					results.status = ResultsStatus.GJK_Failed;
				}
			}
			return (false);
		} finally {
			stack.leave(sp);
			gjk.destroy();
		}
	}

}
