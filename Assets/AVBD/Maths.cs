// This file is the C# translation of avbd-demo2d-repo/source/maths.h
// It implements matrix and vector math utilities for the AVBD physics engine.
// Keep in sync with the C++ original for correctness!

namespace avbd
{
    using UnityEngine;

    public static class Maths
    {
        public static Vector2 Rotate(Vector2 v, float angle)
        {
            float c = Mathf.Cos(angle);
            float s = Mathf.Sin(angle);
            return new Vector2(v.x * c - v.y * s, v.x * s + v.y * c);
        }

        /// <summary>
        /// Solves the linear system A * x = b for x, where A is a symmetric positive definite 3x3 matrix.
        /// This is a direct port of the LDL^T decomposition solver from maths.h (C++).
        /// </summary>
        /// <param name="A">Symmetric positive definite 3x3 matrix (row-major)</param>
        /// <param name="b">Right-hand side vector</param>
        /// <returns>Solution vector x</returns>
        public static Vector3 Solve(Matrix3x3 A, Vector3 b)
        {
            // LDL^T decomposition variables:
            // Ld.x = D1, Ld.y = D2, Ld.z = D3 (diagonal)
            // L.x = L21, L.y = L31, L.z = L32 (lower triangle)
            Vector3 Ld = new Vector3();
            Vector3 L = new Vector3();
            Vector3 x = new Vector3();

            // Compute D1 (first diagonal)
            Ld.x = A.m00;
            
            // Compute L21 and L31 (lower triangle)
            L.x = A.m10 / Ld.x; // L21
            L.y = A.m20 / Ld.x; // L31
            // Compute D2 (second diagonal)
            Ld.y = A.m11 - L.x * L.x * Ld.x;
            // Compute L32
            L.z = (A.m21 - L.x * L.y * Ld.x) / Ld.y; // L32
            // Compute D3 (third diagonal)
            Ld.z = A.m22 - L.y * L.y * Ld.x - L.z * L.z * Ld.y;

            // Forward substitution: Solve L * y = b
            x.x = b.x;
            x.y = b.y - L.x * x.x;
            x.z = b.z - L.y * x.x - L.z * x.y;

            // Diagonal solve: Solve D * z = y
            x.x /= Ld.x;
            x.y /= Ld.y;
            x.z /= Ld.z;

            // Backward substitution: Solve L^T * x = z
            float x_z = x.z;
            float x_y = x.y - L.z * x_z;
            float x_x = x.x - L.x * x_y - L.y * x_z;
            return new Vector3(x_x, x_y, x_z);
        }
    }

    public struct Matrix2x2
    {
        public float m00, m01;
        public float m10, m11;

        public Matrix2x2(float angle)
        {
            float c = Mathf.Cos(angle);
            float s = Mathf.Sin(angle);
            m00 = c; m01 = -s;
            m10 = s; m11 = c;
        }

        public static Matrix2x2 Transpose(Matrix2x2 m)
        {
            return new Matrix2x2()
            {
                m00 = m.m00, m01 = m.m10,
                m10 = m.m01, m11 = m.m11
            };
        }

        public static Matrix2x2 Outer(Vector2 a, Vector2 b)
        {
            return new Matrix2x2()
            {
                m00 = b.x * a.x, m01 = b.y * a.x,
                m10 = b.x * a.y, m11 = b.y * a.y
            };
        }

        public static Vector2 operator *(Matrix2x2 m, Vector2 v)
        {
            return new Vector2(m.m00 * v.x + m.m01 * v.y, m.m10 * v.x + m.m11 * v.y);
        }

        public static Vector2 operator *(Vector2 v, Matrix2x2 m)
        {
            return new Vector2(v.x * m.m00 + v.y * m.m10, v.x * m.m01 + v.y * m.m11);
        }

        public static Matrix2x2 operator *(Matrix2x2 a, float s)
        {
            return new Matrix2x2()
            {
                m00 = a.m00 * s, m01 = a.m01 * s,
                m10 = a.m10 * s, m11 = a.m11 * s
            };
        }

        public static Matrix2x2 operator -(Matrix2x2 a, Matrix2x2 b)
        {
            return new Matrix2x2()
            {
                m00 = a.m00 - b.m00, m01 = a.m01 - b.m01,
                m10 = a.m10 - b.m10, m11 = a.m11 - b.m11
            };
        }

        public static Matrix2x2 operator /(Matrix2x2 a, float s)
        {
            return new Matrix2x2()
            {
                m00 = a.m00 / s, m01 = a.m01 / s,
                m10 = a.m10 / s, m11 = a.m11 / s
            };
        }

        public static Matrix2x2 operator *(Matrix2x2 a, Matrix2x2 b)
        {
            return new Matrix2x2()
            {
                m00 = a.m00 * b.m00 + a.m01 * b.m10,
                m01 = a.m00 * b.m01 + a.m01 * b.m11,
                m10 = a.m10 * b.m00 + a.m11 * b.m10,
                m11 = a.m10 * b.m01 + a.m11 * b.m11
            };
        }

        public Vector2 GetColumn(int c)
        {
            if (c == 0) return new Vector2(m00, m10);
            if (c == 1) return new Vector2(m01, m11);
            return Vector2.zero;
        }
    }

    public struct Matrix3x3
    {
        public float m00, m01, m02;
        public float m10, m11, m12;
        public float m20, m21, m22;

        public static Matrix3x3 zero = new Matrix3x3(0, 0, 0, 0, 0, 0, 0, 0, 0);

        public Matrix3x3(float m00, float m01, float m02, float m10, float m11, float m12, float m20, float m21, float m22)
        {
            this.m00 = m00; this.m01 = m01; this.m02 = m02;
            this.m10 = m10; this.m11 = m11; this.m12 = m12;
            this.m20 = m20; this.m21 = m21; this.m22 = m22;
        }

        public static Matrix3x3 Outer(Vector3 a, Vector3 b)
        {
            return new Matrix3x3(
                b.x * a.x, b.y * a.x, b.z * a.x,
                b.x * a.y, b.y * a.y, b.z * a.y,
                b.x * a.z, b.y * a.z, b.z * a.z
            );
        }

        public static Matrix3x3 operator +(Matrix3x3 a, Matrix3x3 b)
        {
            return new Matrix3x3(
                a.m00 + b.m00, a.m01 + b.m01, a.m02 + b.m02,
                a.m10 + b.m10, a.m11 + b.m11, a.m12 + b.m12,
                a.m20 + b.m20, a.m21 + b.m21, a.m22 + b.m22
            );
        }

        public static Matrix3x3 operator *(Matrix3x3 a, float s)
        {
            return new Matrix3x3(
                a.m00 * s, a.m01 * s, a.m02 * s,
                a.m10 * s, a.m11 * s, a.m12 * s,
                a.m20 * s, a.m21 * s, a.m22 * s
            );
        }

        public static Vector3 operator *(Matrix3x3 a, Vector3 v)
        {
            return new Vector3(
                a.m00 * v.x + a.m01 * v.y + a.m02 * v.z,
                a.m10 * v.x + a.m11 * v.y + a.m12 * v.z,
                a.m20 * v.x + a.m21 * v.y + a.m22 * v.z
            );
        }

        public Vector3 GetColumn(int c)
        {
            if (c == 0) return new Vector3(m00, m10, m20);
            if (c == 1) return new Vector3(m01, m11, m21);
            if (c == 2) return new Vector3(m02, m12, m22);
            return Vector3.zero;
        }
    }
}