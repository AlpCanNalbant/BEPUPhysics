using System;

namespace BEPUutilities
{
    /// <summary>
    /// Provides XNA-like quaternion support.
    /// </summary>
    /// <remarks>
    /// Constructs a new Quaternion.
    /// </remarks>
    /// <param name="x">X component of the quaternion.</param>
    /// <param name="y">Y component of the quaternion.</param>
    /// <param name="z">Z component of the quaternion.</param>
    /// <param name="w">W component of the quaternion.</param>
    public struct Quaternion(float x, float y, float z, float w) : IEquatable<Quaternion>
    {
        /// <summary>
        /// X component of the quaternion.
        /// </summary>
        public float X = x;

        /// <summary>
        /// Y component of the quaternion.
        /// </summary>
        public float Y = y;

        /// <summary>
        /// Z component of the quaternion.
        /// </summary>
        public float Z = z;

        /// <summary>
        /// W component of the quaternion.
        /// </summary>
        public float W = w;

        /// <summary>
        /// Adds two quaternions together.
        /// </summary>
        /// <param name="a">First quaternion to add.</param>
        /// <param name="b">Second quaternion to add.</param>
        /// <param name="result">Sum of the addition.</param>
        public static void Add(ref Quaternion a, ref Quaternion b, out Quaternion result)
            => result = new(a.X + b.X, a.Y + b.Y, a.Z + b.Z, a.W + b.W);

        /// <summary>
        /// Multiplies two quaternions.
        /// </summary>
        /// <param name="a">First quaternion to multiply.</param>
        /// <param name="b">Second quaternion to multiply.</param>
        /// <param name="result">Product of the multiplication.</param>
        public static void Multiply(ref Quaternion a, ref Quaternion b, out Quaternion result)
            => result = new(
                a.X * b.W + b.X * a.W + a.Y * b.Z - a.Z * b.Y,
                a.Y * b.W + b.Y * a.W + a.Z * b.X - a.X * b.Z,
                a.Z * b.W + b.Z * a.W + a.X * b.Y - a.Y * b.X,
                a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z);

        /// <summary>
        /// Scales a quaternion.
        /// </summary>
        /// <param name="q">Quaternion to multiply.</param>
        /// <param name="scale">Amount to multiply each component of the quaternion by.</param>
        /// <param name="result">Scaled quaternion.</param>
        public static void Multiply(ref Quaternion q, float scale, out Quaternion result)
            => result = new(q.X * scale, q.Y * scale, q.Z * scale, q.W * scale);

        /// <summary>
        /// Multiplies two quaternions together in opposite order.
        /// </summary>
        /// <param name="a">First quaternion to multiply.</param>
        /// <param name="b">Second quaternion to multiply.</param>
        /// <param name="result">Product of the multiplication.</param>
        public static void Concatenate(ref Quaternion a, ref Quaternion b, out Quaternion result)
            => result = new(
                a.W * b.X + a.X * b.W + a.Z * b.Y - a.Y * b.Z,
                a.W * b.Y + a.Y * b.W + a.X * b.Z - a.Z * b.X,
                a.W * b.Z + a.Z * b.W + a.Y * b.X - a.X * b.Y,
                a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z);

        /// <summary>
        /// Multiplies two quaternions together in opposite order.
        /// </summary>
        /// <param name="a">First quaternion to multiply.</param>
        /// <param name="b">Second quaternion to multiply.</param>
        /// <returns>Product of the multiplication.</returns>
        public static Quaternion Concatenate(Quaternion a, Quaternion b)
        {
            Concatenate(ref a, ref b, out Quaternion result);
            return result;
        }

        /// <summary>
        /// Quaternion representing the identity transform.
        /// </summary>
        public static Quaternion Identity
            => new(0f, 0f, 0f, 1f);

        /// <summary>
        /// Constructs a quaternion from a rotation matrix.
        /// </summary>
        /// <param name="r">Rotation matrix to create the quaternion from.</param>
        /// <param name="q">Quaternion based on the rotation matrix.</param>
        public static void CreateFromRotationMatrix(ref Matrix3x3 r, out Quaternion q)
        {
            float trace = r.M11 + r.M22 + r.M33;
            if (trace >= 0f)
            {
                float S = MathF.Sqrt(trace + 1.0f) * 2f; // S=4*qw
                float inverseS = 1f / S;
                q = new((r.M23 - r.M32) * inverseS, (r.M31 - r.M13) * inverseS, (r.M12 - r.M21) * inverseS, 0.25f * S);
            }
            else if ((r.M11 > r.M22) & (r.M11 > r.M33))
            {
                float S = MathF.Sqrt(1.0f + r.M11 - r.M22 - r.M33) * 2f; // S=4*qx
                float inverseS = 1f / S;
                q = new(0.25f * S, (r.M21 + r.M12) * inverseS, (r.M31 + r.M13) * inverseS, (r.M23 - r.M32) * inverseS);
            }
            else if (r.M22 > r.M33)
            {
                float S = MathF.Sqrt(1.0f + r.M22 - r.M11 - r.M33) * 2f; // S=4*qy
                float inverseS = 1f / S;
                q = new((r.M21 + r.M12) * inverseS, 0.25f * S, (r.M32 + r.M23) * inverseS, (r.M31 - r.M13) * inverseS);
            }
            else
            {
                float S = MathF.Sqrt(1.0f + r.M33 - r.M11 - r.M22) * 2f; // S=4*qz
                float inverseS = 1f / S;
                q = new((r.M31 + r.M13) * inverseS, (r.M32 + r.M23) * inverseS, 0.25f * S, (r.M12 - r.M21) * inverseS);
            }
        }

        /// <summary>
        /// Creates a quaternion from a rotation matrix.
        /// </summary>
        /// <param name="r">Rotation matrix used to create a new quaternion.</param>
        /// <returns>Quaternion representing the same rotation as the matrix.</returns>
        public static Quaternion CreateFromRotationMatrix(Matrix3x3 r)
        {
            CreateFromRotationMatrix(ref r, out Quaternion toReturn);
            return toReturn;
        }

        /// <summary>
        /// Constructs a quaternion from a rotation matrix.
        /// </summary>
        /// <param name="r">Rotation matrix to create the quaternion from.</param>
        /// <param name="q">Quaternion based on the rotation matrix.</param>
        public static void CreateFromRotationMatrix(ref Matrix r, out Quaternion q)
        {
            Matrix3x3.CreateFromMatrix(ref r, out Matrix3x3 downsizedMatrix);
            CreateFromRotationMatrix(ref downsizedMatrix, out q);
        }

        /// <summary>
        /// Creates a quaternion from a rotation matrix.
        /// </summary>
        /// <param name="r">Rotation matrix used to create a new quaternion.</param>
        /// <returns>Quaternion representing the same rotation as the matrix.</returns>
        public static Quaternion CreateFromRotationMatrix(Matrix r)
        {
            CreateFromRotationMatrix(ref r, out Quaternion toReturn);
            return toReturn;
        }


        /// <summary>
        /// Ensures the quaternion has unit length.
        /// </summary>
        /// <param name="quaternion">Quaternion to normalize.</param>
        /// <returns>Normalized quaternion.</returns>
        public static Quaternion Normalize(Quaternion quaternion)
        {
            Normalize(ref quaternion, out Quaternion toReturn);
            return toReturn;
        }

        /// <summary>
        /// Ensures the quaternion has unit length.
        /// </summary>
        /// <param name="quaternion">Quaternion to normalize.</param>
        /// <param name="toReturn">Normalized quaternion.</param>
        public static void Normalize(ref Quaternion quaternion, out Quaternion toReturn)
        {
            float inverse = (1f / MathF.Sqrt(quaternion.X * quaternion.X + quaternion.Y * quaternion.Y + quaternion.Z * quaternion.Z + quaternion.W * quaternion.W));
            toReturn = new(quaternion.X * inverse, quaternion.Y * inverse, quaternion.Z * inverse, quaternion.W * inverse);
        }

        /// <summary>
        /// Scales the quaternion such that it has unit length.
        /// </summary>
        public void Normalize()
        {
            float inverse = (1f / MathF.Sqrt(X * X + Y * Y + Z * Z + W * W));
            X *= inverse;
            Y *= inverse;
            Z *= inverse;
            W *= inverse;
        }

        /// <summary>
        /// Computes the squared length of the quaternion.
        /// </summary>
        /// <returns>Squared length of the quaternion.</returns>
        public readonly float LengthSquared()
            => X * X + Y * Y + Z * Z + W * W;

        /// <summary>
        /// Computes the length of the quaternion.
        /// </summary>
        /// <returns>Length of the quaternion.</returns>
        public readonly float Length()
            => MathF.Sqrt(X * X + Y * Y + Z * Z + W * W);


        /// <summary>
        /// Blends two quaternions together to get an intermediate state.
        /// </summary>
        /// <param name="start">Starting point of the interpolation.</param>
        /// <param name="end">Ending point of the interpolation.</param>
        /// <param name="interpolationAmount">Amount of the end point to use.</param>
        /// <param name="result">Interpolated intermediate quaternion.</param>
        public static void Slerp(ref Quaternion start, ref Quaternion end, float interpolationAmount, out Quaternion result)
        {
            float cosHalfTheta = start.W * end.W + start.X * end.X + start.Y * end.Y + start.Z * end.Z;
            if (cosHalfTheta < 0f)
            {
                //Negating a quaternion results in the same orientation,
                //but we need cosHalfTheta to be positive to get the shortest path.
                end = new(-end.X, -end.Y, -end.Z, -end.W);
                cosHalfTheta = -cosHalfTheta;
            }
            // If the orientations are similar enough, then just pick one of the inputs.
            if (cosHalfTheta > (1f - 1e-12f))
            {
                result = new(start.X, start.Y, start.Z, start.W);
                return;
            }
            // Calculate temporary values.
            float halfTheta = MathF.Acos(cosHalfTheta);
            float sinHalfTheta = MathF.Sqrt(1.0f - cosHalfTheta * cosHalfTheta);

            float aFraction = MathF.Sin((1f - interpolationAmount) * halfTheta) / sinHalfTheta;
            float bFraction = MathF.Sin(interpolationAmount * halfTheta) / sinHalfTheta;

            //Blend the two quaternions to get the result!
            result = new((start.X * aFraction + end.X * bFraction), (start.Y * aFraction + end.Y * bFraction), (start.Z * aFraction + end.Z * bFraction), (start.W * aFraction + end.W * bFraction));
        }

        /// <summary>
        /// Blends two quaternions together to get an intermediate state.
        /// </summary>
        /// <param name="start">Starting point of the interpolation.</param>
        /// <param name="end">Ending point of the interpolation.</param>
        /// <param name="interpolationAmount">Amount of the end point to use.</param>
        /// <returns>Interpolated intermediate quaternion.</returns>
        public static Quaternion Slerp(Quaternion start, Quaternion end, float interpolationAmount)
        {
            Slerp(ref start, ref end, interpolationAmount, out Quaternion toReturn);
            return toReturn;
        }


        /// <summary>
        /// Computes the conjugate of the quaternion.
        /// </summary>
        /// <param name="quaternion">Quaternion to conjugate.</param>
        /// <param name="result">Conjugated quaternion.</param>
        public static void Conjugate(ref Quaternion quaternion, out Quaternion result)
            => result = new(-quaternion.X, -quaternion.Y, -quaternion.Z, quaternion.W);

        /// <summary>
        /// Computes the conjugate of the quaternion.
        /// </summary>
        /// <param name="quaternion">Quaternion to conjugate.</param>
        /// <returns>Conjugated quaternion.</returns>
        public static Quaternion Conjugate(Quaternion quaternion)
        {
            Conjugate(ref quaternion, out Quaternion toReturn);
            return toReturn;
        }



        /// <summary>
        /// Computes the inverse of the quaternion.
        /// </summary>
        /// <param name="quaternion">Quaternion to invert.</param>
        /// <param name="result">Result of the inversion.</param>
        public static void Inverse(ref Quaternion quaternion, out Quaternion result)
        {
            float inverseSquaredNorm = quaternion.X * quaternion.X + quaternion.Y * quaternion.Y + quaternion.Z * quaternion.Z + quaternion.W * quaternion.W;
            result = new(-quaternion.X * inverseSquaredNorm, -quaternion.Y * inverseSquaredNorm, -quaternion.Z * inverseSquaredNorm, quaternion.W * inverseSquaredNorm);
        }

        /// <summary>
        /// Computes the inverse of the quaternion.
        /// </summary>
        /// <param name="quaternion">Quaternion to invert.</param>
        /// <returns>Result of the inversion.</returns>
        public static Quaternion Inverse(Quaternion quaternion)
        {
            Inverse(ref quaternion, out Quaternion result);
            return result;
        }

        /// <summary>
        /// Tests components for equality.
        /// </summary>
        /// <param name="a">First quaternion to test for equivalence.</param>
        /// <param name="b">Second quaternion to test for equivalence.</param>
        /// <returns>Whether or not the quaternions' components were equal.</returns>
        public static bool operator ==(Quaternion a, Quaternion b)
            => a.X == b.X && a.Y == b.Y && a.Z == b.Z && a.W == b.W;

        /// <summary>
        /// Tests components for inequality.
        /// </summary>
        /// <param name="a">First quaternion to test for equivalence.</param>
        /// <param name="b">Second quaternion to test for equivalence.</param>
        /// <returns>Whether the quaternions' components were not equal.</returns>
        public static bool operator !=(Quaternion a, Quaternion b)
            => a.X != b.X || a.Y != b.Y || a.Z != b.Z || a.W != b.W;

        /// <summary>
        /// Negates the components of a quaternion.
        /// </summary>
        /// <param name="a">Quaternion to negate.</param>
        /// <param name="b">Negated result.</param>
        public static void Negate(ref Quaternion a, out Quaternion b)
            => b = new(-a.X, -a.Y, -a.Z, -a.W);

        /// <summary>
        /// Negates the components of a quaternion.
        /// </summary>
        /// <param name="q">Quaternion to negate.</param>
        /// <returns>Negated result.</returns>
        public static Quaternion Negate(Quaternion q)
        {
            Negate(ref q, out var result);
            return result;
        }

        /// <summary>
        /// Negates the components of a quaternion.
        /// </summary>
        /// <param name="q">Quaternion to negate.</param>
        /// <returns>Negated result.</returns>
        public static Quaternion operator -(Quaternion q)
        {
            Negate(ref q, out var result);
            return result;
        }

        /// <summary>
        /// Indicates whether the current object is equal to another object of the same type.
        /// </summary>
        /// <returns>
        /// true if the current object is equal to the <paramref name="other"/> parameter; otherwise, false.
        /// </returns>
        /// <param name="other">An object to compare with this object.</param>
        public readonly bool Equals(Quaternion other)
            => X == other.X && Y == other.Y && Z == other.Z && W == other.W;

        /// <summary>
        /// Indicates whether this instance and a specified object are equal.
        /// </summary>
        /// <returns>
        /// true if <paramref name="obj"/> and this instance are the same type and represent the same value; otherwise, false.
        /// </returns>
        /// <param name="obj">Another object to compare to. </param><filterpriority>2</filterpriority>
        public override readonly bool Equals(object obj)
        {
            if (obj is Quaternion quaternion)
            {
                return Equals(quaternion);
            }
            return false;
        }

        /// <summary>
        /// Returns the hash code for this instance.
        /// </summary>
        /// <returns>
        /// A 32-bit signed integer that is the hash code for this instance.
        /// </returns>
        /// <filterpriority>2</filterpriority>
        public override readonly int GetHashCode()
            => X.GetHashCode() + Y.GetHashCode() + Z.GetHashCode() + W.GetHashCode();

        /// <summary>
        /// Transforms the vector using a quaternion.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="rotation">Rotation to apply to the vector.</param>
        /// <param name="result">Transformed vector.</param>
        public static void Transform(ref Vector3 v, ref Quaternion rotation, out Vector3 result)
        {
            //This operation is an optimized-down version of v' = q * v * q^-1.
            //The expanded form would be to treat v as an 'axis only' quaternion
            //and perform standard quaternion multiplication.  Assuming q is normalized,
            //q^-1 can be replaced by a conjugation.
            float x2 = rotation.X + rotation.X;
            float y2 = rotation.Y + rotation.Y;
            float z2 = rotation.Z + rotation.Z;
            float xx2 = rotation.X * x2;
            float xy2 = rotation.X * y2;
            float xz2 = rotation.X * z2;
            float yy2 = rotation.Y * y2;
            float yz2 = rotation.Y * z2;
            float zz2 = rotation.Z * z2;
            float wx2 = rotation.W * x2;
            float wy2 = rotation.W * y2;
            float wz2 = rotation.W * z2;
            //Defer the component setting since they're used in computation.
            result = new(
                v.X * (1f - yy2 - zz2) + v.Y * (xy2 - wz2) + v.Z * (xz2 + wy2),
                v.X * (xy2 + wz2) + v.Y * (1f - xx2 - zz2) + v.Z * (yz2 - wx2),
                v.X * (xz2 - wy2) + v.Y * (yz2 + wx2) + v.Z * (1f - xx2 - yy2));
        }

        /// <summary>
        /// Transforms the vector using a quaternion.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="rotation">Rotation to apply to the vector.</param>
        /// <returns>Transformed vector.</returns>
        public static Vector3 Transform(Vector3 v, Quaternion rotation)
        {
            Transform(ref v, ref rotation, out Vector3 toReturn);
            return toReturn;
        }

        /// <summary>
        /// Transforms a vector using a quaternion. Specialized for x,0,0 vectors.
        /// </summary>
        /// <param name="x">X component of the vector to transform.</param>
        /// <param name="rotation">Rotation to apply to the vector.</param>
        /// <param name="result">Transformed vector.</param>
        public static void TransformX(float x, ref Quaternion rotation, out Vector3 result)
        {
            //This operation is an optimized-down version of v' = q * v * q^-1.
            //The expanded form would be to treat v as an 'axis only' quaternion
            //and perform standard quaternion multiplication.  Assuming q is normalized,
            //q^-1 can be replaced by a conjugation.
            float y2 = rotation.Y + rotation.Y;
            float z2 = rotation.Z + rotation.Z;
            //Defer the component setting since they're used in computation.
            result = new(
                x * (1f - (rotation.Y * y2) - (rotation.Z * z2)),
                x * ((rotation.X * y2) + (rotation.W * z2)),
                x * ((rotation.X * z2) - (rotation.W * y2))
            );
        }

        /// <summary>
        /// Transforms a vector using a quaternion. Specialized for 0,y,0 vectors.
        /// </summary>
        /// <param name="y">Y component of the vector to transform.</param>
        /// <param name="rotation">Rotation to apply to the vector.</param>
        /// <param name="result">Transformed vector.</param>
        public static void TransformY(float y, ref Quaternion rotation, out Vector3 result)
        {
            //This operation is an optimized-down version of v' = q * v * q^-1.
            //The expanded form would be to treat v as an 'axis only' quaternion
            //and perform standard quaternion multiplication.  Assuming q is normalized,
            //q^-1 can be replaced by a conjugation.
            float x2 = rotation.X + rotation.X;
            float z2 = rotation.Z + rotation.Z;
            //Defer the component setting since they're used in computation.
            result = new(y * ((rotation.X * (rotation.Y + rotation.Y)) - (rotation.W * z2)), y * (1f - (rotation.X * x2) - (rotation.Z * z2)), y * ((rotation.Y * z2) + (rotation.W * x2)));
        }

        /// <summary>
        /// Transforms a vector using a quaternion. Specialized for 0,0,z vectors.
        /// </summary>
        /// <param name="z">Z component of the vector to transform.</param>
        /// <param name="rotation">Rotation to apply to the vector.</param>
        /// <param name="result">Transformed vector.</param>
        public static void TransformZ(float z, ref Quaternion rotation, out Vector3 result)
        {
            //This operation is an optimized-down version of v' = q * v * q^-1.
            //The expanded form would be to treat v as an 'axis only' quaternion
            //and perform standard quaternion multiplication.  Assuming q is normalized,
            //q^-1 can be replaced by a conjugation.
            float x2 = rotation.X + rotation.X;
            float y2 = rotation.Y + rotation.Y;
            float z2 = rotation.Z + rotation.Z;
            //Defer the component setting since they're used in computation.
            result = new(z * ((rotation.X * z2) + (rotation.W * y2)), z * ((rotation.Y * z2) - (rotation.W * x2)), z * (1f - (rotation.X * x2) - (rotation.Y * y2)));
        }


        /// <summary>
        /// Multiplies two quaternions.
        /// </summary>
        /// <param name="a">First quaternion to multiply.</param>
        /// <param name="b">Second quaternion to multiply.</param>
        /// <returns>Product of the multiplication.</returns>
        public static Quaternion operator *(Quaternion a, Quaternion b)
        {
            Multiply(ref a, ref b, out Quaternion toReturn);
            return toReturn;
        }

        /// <summary>
        /// Creates a quaternion from an axis and angle.
        /// </summary>
        /// <param name="axis">Axis of rotation.</param>
        /// <param name="angle">Angle to rotate around the axis.</param>
        /// <returns>Quaternion representing the axis and angle rotation.</returns>
        public static Quaternion CreateFromAxisAngle(Vector3 axis, float angle)
        {
            float halfAngle = angle * 0.5f;
            float s = MathF.Sin(halfAngle);
            return new Quaternion((axis.X * s), (axis.Y * s), (axis.Z * s), MathF.Cos(halfAngle));
        }

        /// <summary>
        /// Creates a quaternion from an axis and angle.
        /// </summary>
        /// <param name="axis">Axis of rotation.</param>
        /// <param name="angle">Angle to rotate around the axis.</param>
        /// <param name="q">Quaternion representing the axis and angle rotation.</param>
        public static void CreateFromAxisAngle(ref Vector3 axis, float angle, out Quaternion q)
        {
            float halfAngle = angle * 0.5f;
            float s = MathF.Sin(halfAngle);
            q = new((axis.X * s), (axis.Y * s), (axis.Z * s), MathF.Cos(halfAngle));
        }

        /// <summary>
        /// Constructs a quaternion from yaw, pitch, and roll.
        /// </summary>
        /// <param name="yaw">Yaw of the rotation.</param>
        /// <param name="pitch">Pitch of the rotation.</param>
        /// <param name="roll">Roll of the rotation.</param>
        /// <returns>Quaternion representing the yaw, pitch, and roll.</returns>
        public static Quaternion CreateFromYawPitchRoll(float yaw, float pitch, float roll)
        {
            CreateFromYawPitchRoll(yaw, pitch, roll, out Quaternion toReturn);
            return toReturn;
        }

        /// <summary>
        /// Constructs a quaternion from yaw, pitch, and roll.
        /// </summary>
        /// <param name="yaw">Yaw of the rotation.</param>
        /// <param name="pitch">Pitch of the rotation.</param>
        /// <param name="roll">Roll of the rotation.</param>
        /// <param name="q">Quaternion representing the yaw, pitch, and roll.</param>
        public static void CreateFromYawPitchRoll(float yaw, float pitch, float roll, out Quaternion q)
        {
            float halfRoll = roll * 0.5f;
            float halfPitch = pitch * 0.5f;
            float halfYaw = yaw * 0.5f;

            float sinRoll = MathF.Sin(halfRoll);
            float sinPitch = MathF.Sin(halfPitch);
            float sinYaw = MathF.Sin(halfYaw);

            float cosRoll = MathF.Cos(halfRoll);
            float cosPitch = MathF.Cos(halfPitch);
            float cosYaw = MathF.Cos(halfYaw);

            float cosYawCosPitch = cosYaw * cosPitch;
            float cosYawSinPitch = cosYaw * sinPitch;
            float sinYawCosPitch = sinYaw * cosPitch;
            float sinYawSinPitch = sinYaw * sinPitch;

            q = new(
                (cosYawSinPitch * cosRoll + sinYawCosPitch * sinRoll),
                (sinYawCosPitch * cosRoll - cosYawSinPitch * sinRoll),
                (cosYawCosPitch * sinRoll - sinYawSinPitch * cosRoll),
                (cosYawCosPitch * cosRoll + sinYawSinPitch * sinRoll));
        }

        /// <summary>
        /// Computes the angle change represented by a normalized quaternion.
        /// </summary>
        /// <param name="q">Quaternion to be converted.</param>
        /// <returns>Angle around the axis represented by the quaternion.</returns>
        public static float GetAngleFromQuaternion(ref Quaternion q)
        {
            float qw = MathF.Abs(q.W);
            if (qw > 1f)
                return 0f;
            return 2f * MathF.Acos(qw);
        }

        /// <summary>
        /// Computes the axis angle representation of a normalized quaternion.
        /// </summary>
        /// <param name="q">Quaternion to be converted.</param>
        /// <param name="axis">Axis represented by the quaternion.</param>
        /// <param name="angle">Angle around the axis represented by the quaternion.</param>
        public static void GetAxisAngleFromQuaternion(ref Quaternion q, out Vector3 axis, out float angle)
        {
            float qw = q.W;
            if (qw > 0f)
            {
                axis = new(q.X, q.Y, q.Z);
            }
            else
            {
                axis = new(-q.X, -q.Y, -q.Z);
                qw = -qw;
            }

            float lengthSquared = axis.LengthSquared();
            if (lengthSquared > 1e-14f)
            {
                Vector3.Divide(ref axis, MathF.Sqrt(lengthSquared), out axis);
                angle = 2f * MathF.Acos(MathHelper.Clamp(qw, -1f, 1f));
            }
            else
            {
                axis = Toolbox.UpVector;
                angle = 0f;
            }
        }

        /// <summary>
        /// Computes the quaternion rotation between two normalized vectors.
        /// </summary>
        /// <param name="v1">First unit-length vector.</param>
        /// <param name="v2">Second unit-length vector.</param>
        /// <param name="q">Quaternion representing the rotation from v1 to v2.</param>
        public static void GetQuaternionBetweenNormalizedVectors(ref Vector3 v1, ref Vector3 v2, out Quaternion q)
        {
            Vector3.Dot(ref v1, ref v2, out float dot);
            //For non-normal vectors, the multiplying the axes length squared would be necessary:
            //float w = dot + MathF.Sqrt(v1.LengthSquared() * v2.LengthSquared());
            if (dot < -0.9999f) //parallel, opposing direction
            {
                //If this occurs, the rotation required is ~180 degrees.
                //The problem is that we could choose any perpendicular axis for the rotation. It's not uniquely defined.
                //The solution is to pick an arbitrary perpendicular axis.
                //Project onto the plane which has the lowest component magnitude.
                //On that 2d plane, perform a 90 degree rotation.
                float absX = MathF.Abs(v1.X);
                float absY = MathF.Abs(v1.Y);
                float absZ = MathF.Abs(v1.Z);
                if (absX < absY && absX < absZ)
                    q = new Quaternion(0f, -v1.Z, v1.Y, 0f);
                else if (absY < absZ)
                    q = new Quaternion(-v1.Z, 0f, v1.X, 0f);
                else
                    q = new Quaternion(-v1.Y, v1.X, 0f, 0f);
            }
            else
            {
                Vector3.Cross(ref v1, ref v2, out Vector3 axis);
                q = new Quaternion(axis.X, axis.Y, axis.Z, dot + 1f);
            }
            q.Normalize();
        }

        //The following two functions are highly similar, but it's a bit of a brain teaser to phrase one in terms of the other.
        //Providing both simplifies things.

        /// <summary>
        /// Computes the rotation from the start orientation to the end orientation such that end = Quaternion.Concatenate(start, relative).
        /// </summary>
        /// <param name="start">Starting orientation.</param>
        /// <param name="end">Ending orientation.</param>
        /// <param name="relative">Relative rotation from the start to the end orientation.</param>
        public static void GetRelativeRotation(ref Quaternion start, ref Quaternion end, out Quaternion relative)
        {
            Conjugate(ref start, out Quaternion startInverse);
            Concatenate(ref startInverse, ref end, out relative);
        }


        /// <summary>
        /// Transforms the rotation into the local space of the target basis such that rotation = Quaternion.Concatenate(localRotation, targetBasis)
        /// </summary>
        /// <param name="rotation">Rotation in the original frame of reference.</param>
        /// <param name="targetBasis">Basis in the original frame of reference to transform the rotation into.</param>
        /// <param name="localRotation">Rotation in the local space of the target basis.</param>
        public static void GetLocalRotation(ref Quaternion rotation, ref Quaternion targetBasis, out Quaternion localRotation)
        {
            Conjugate(ref targetBasis, out Quaternion basisInverse);
            Concatenate(ref rotation, ref basisInverse, out localRotation);
        }

        /// <summary>
        /// Gets a string representation of the quaternion.
        /// </summary>
        /// <returns>String representing the quaternion.</returns>
        public override readonly string ToString()
            => "{ X: " + X + ", Y: " + Y + ", Z: " + Z + ", W: " + W + "}";
    }
}
